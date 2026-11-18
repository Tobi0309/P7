import math

import do_mpc
import numpy as np
import rclpy

from casadi import cos, interpolant, sin, sqrt
from scipy.interpolate import BSpline as SciPyBSpline
from scipy.interpolate import CubicSpline
from nav_msgs.msg import Path
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from traj_utils.msg import Bspline
from tf2_ros import Buffer, TransformListener


class ModelPredictiveControlDroneFollower(Node):
    def __init__(self):
        super().__init__("drone_velocity_mpc")

        self.declare_parameter("path_topic", "/drone_path")
        self.declare_parameter("bspline_topic", "/drone_0_planning/bspline")
        self.declare_parameter("world_frame", "camera_init")
        self.declare_parameter("drone_frame", "base_link")
        self.declare_parameter("body_yaw_offset", np.pi)
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("use_bspline", True)
        self.declare_parameter("bspline_sample_count", 200)
 
        self.MPC_SETUP_PARAMS = {
            'n_horizon': 30,
            't_step': 0.1,
            'state_discretization': 'collocation',
            'store_full_solution': True,
            'nlpsol_opts': {
                'ipopt.max_iter': 100,
                'ipopt.max_cpu_time': 0.08,
                'ipopt.print_level': 0,
                'ipopt.sb': 'yes',
                'print_time': 0,
            },
        }

        self.PATH_RESOLUTION = 1000

        # Contouring MPC weights. The MPC picks its own progress speed v_s along
        # the path, trading the progress reward against contour error, so it
        # slows down in curves it cannot follow at speed.
        self.CONTOUR_WEIGHT = 40.0          # cost on horizontal distance off the path
        self.VERTICAL_CONTOUR_WEIGHT = 5.0  # cost on vertical distance off the path
        # Lag must stay above the contour weight, otherwise the solver can hide
        # contour error by sliding the reference point along the path.
        self.LAG_WEIGHT = 80.0              # cost on reference point being ahead/behind the drone
        self.HEADING_WEIGHT = 3.0           # cost on heading vs path tangent
        # Vertical damping: makes z react smoothly instead of chasing small errors.
        self.VZ_WEIGHT = 0.5                # cost on vz^2
        self.VZ_CHANGE_WEIGHT = 2.0         # cost on (delta vz)^2 between steps
        self.PROGRESS_REWARD = 0.5    # q_v: reward per m/s of progress (higher = more aggressive)
        self.MAX_PROGRESS_SPEED = 1.0 # v_max: upper bound on v_s [m/s]
        self.PROGRESS_SPEED_CHANGE_WEIGHT = 0.1  # smoothing on v_s
        # Small reward on s itself at every stage. Without it, once the path end
        # is inside the horizon the total v_s reward is fixed, so the drone has
        # no reason to hurry and crawls the last metres.
        self.PROGRESS_POSITION_REWARD = 0.1
        self.PATH_END_EXTENSION = 2.0  # [m] straight extension of the interpolants past the end

        self.path_spline = None
        self.path_length = 0
        self.s_samples = []
        self.tangent_samples = []
        self.path_points = []
        self.waypoints = []

        self.model = None
        self.mpc = None
        self.has_path = False
        self.initial_guess_set = False
        self.last_psi = None

        self.latest_body_command = (0.0, 0.0, 0.0)
        self.latest_attitude = VehicleAttitude()
        self.latest_attitude.q = [1.0, 0.0, 0.0, 0.0]
        self.latest_vehicle_status = VehicleStatus()
        self.last_nav_state = None
        self.offboard_requested = False
        self.setpoint_count = 0
        self.last_mode_command_us = 0
        self.last_arm_command_us = 0
        self.command_interval_us = 200_000

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.step_timer = self.create_timer(0.1, self.step)
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)

        self.drone_path_subscriber = self.create_subscription(
            msg_type=Path,
            topic=self.get_parameter("path_topic").get_parameter_value().string_value,
            callback=self.drone_path_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.bspline_subscriber = self.create_subscription(
            msg_type=Bspline,
            topic=self.get_parameter("bspline_topic").get_parameter_value().string_value,
            callback=self.bspline_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            10,
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            10,
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            10,
        )
        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos_profile_sensor_data,
        )
        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_sensor_data,
        )

    def drone_path_callback(self, msg):
        if self.get_parameter("use_bspline").get_parameter_value().bool_value:
            return

        self.waypoints = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in msg.poses])

        self.update_path()

    def bspline_callback(self, msg):
        if not self.get_parameter("use_bspline").get_parameter_value().bool_value:
            return

        control_points = np.array(
            [[point.x, point.y, point.z] for point in msg.pos_pts],
            dtype=float,
        )

        if control_points.shape[0] < 2:
            self.get_logger().warn("Ignored Bspline: not enough control points.")
            return

        knots = np.array(msg.knots, dtype=float)
        order = int(msg.order)
        sample_count = max(50, int(self.get_parameter("bspline_sample_count").value))

        if knots.size >= control_points.shape[0] + order + 1:
            start_param = knots[order]
            end_param = knots[-order - 1]

            if end_param <= start_param:
                self.get_logger().warn("Ignored Bspline: invalid knot parameter range.")
                return

            parameters = np.linspace(start_param, end_param, sample_count)

            sampled_points = np.column_stack([
                SciPyBSpline(knots, control_points[:, axis], order, extrapolate=False)(parameters)
                for axis in range(3)
            ])

            valid_samples = np.isfinite(sampled_points).all(axis=1)
            self.waypoints = sampled_points[valid_samples]
        else:
            self.waypoints = control_points

        self.update_path()

    def update_path(self):
        if self.waypoints is None or len(self.waypoints) < 2:
            self.has_path = False
            self.get_logger().warn("Path update ignored: need at least two waypoints.")
            return

        segment_lengths = np.linalg.norm(np.diff(self.waypoints, axis=0), axis=1)
        total_length = float(np.sum(segment_lengths))

        if total_length < 1e-3:
            self.has_path = False
            self.get_logger().warn("Path update ignored: path length is too small.")
            return

        s_waypoints = np.concatenate(([0.0], np.cumsum(segment_lengths)))

        self.path_length = s_waypoints[-1]
        self.path_spline = CubicSpline(s_waypoints, self.waypoints, axis=0)

        self.s_samples = np.linspace(0.0, self.path_length, self.PATH_RESOLUTION)
        self.tangent_samples = self.path_spline.derivative()(self.s_samples)

        self.path_points = self.path_spline(self.s_samples)

        self._build_model()
        self.has_path = True

    def _build_model(self):
        # Unit tangent of the path, used to split the position error into
        # lag (along the path) and contour (off the path) parts.
        tangent_norms = np.linalg.norm(self.tangent_samples, axis=1, keepdims=True)
        unit_tangents = self.tangent_samples / np.maximum(tangent_norms, 1e-6)
        psi_samples = np.unwrap(np.arctan2(self.tangent_samples[:, 1], self.tangent_samples[:, 0]))

        # Extend the interpolation grid past the end with a straight line along
        # the final tangent. s is still bounded to path_length; this only keeps
        # the interpolants smooth at the end so the solver converges there.
        extension_s = np.linspace(0.0, self.PATH_END_EXTENSION, 21)[1:]
        grid_s = np.concatenate((self.s_samples, self.path_length + extension_s))
        extension_points = self.path_points[-1] + np.outer(extension_s, unit_tangents[-1])
        points = np.vstack((self.path_points, extension_points))
        unit_tangents = np.vstack((unit_tangents, np.repeat(unit_tangents[-1:], extension_s.size, axis=0)))
        psi_samples = np.concatenate((psi_samples, np.full(extension_s.size, psi_samples[-1])))
        x_samples = points[:, 0]
        y_samples = points[:, 1]
        z_samples = points[:, 2]

        self.model = do_mpc.model.Model('continuous')
        px = self.model.set_variable('_x', 'px')
        py = self.model.set_variable('_x', 'py')
        pz = self.model.set_variable('_x', 'pz')
        psi = self.model.set_variable('_x', 'psi')
        s = self.model.set_variable('_x', 's')

        vx = self.model.set_variable('_u', 'vx')
        vz = self.model.set_variable('_u', 'vz')
        omega = self.model.set_variable('_u', 'omega')
        # Progress speed of the reference point along the path. Internal to the
        # optimisation only; never sent to PX4. Declared last so u = [vx, vz, omega, v_s].
        v_s = self.model.set_variable('_u', 'v_s')

        path_x = interpolant('path_x', 'bspline', [grid_s], x_samples)
        path_y = interpolant('path_y', 'bspline', [grid_s], y_samples)
        path_z = interpolant('path_z', 'bspline', [grid_s], z_samples)
        path_psi = interpolant('path_psi', 'bspline', [grid_s], psi_samples)
        path_tx = interpolant('path_tx', 'bspline', [grid_s], unit_tangents[:, 0])
        path_ty = interpolant('path_ty', 'bspline', [grid_s], unit_tangents[:, 1])
        path_tz = interpolant('path_tz', 'bspline', [grid_s], unit_tangents[:, 2])

        self.model.set_rhs('px', vx * cos(psi))
        self.model.set_rhs('py', vx * sin(psi))
        self.model.set_rhs('pz', vz)
        self.model.set_rhs('psi', omega)
        self.model.set_rhs('s', v_s)

        ex = px - path_x(s)
        ey = py - path_y(s)
        ez = pz - path_z(s)
        tx = path_tx(s)
        ty = path_ty(s)
        tz = path_tz(s)
        psi_path = path_psi(s)

        # Lag: how far the drone is ahead of (+) / behind (-) the reference point.
        lag_error = tx * ex + ty * ey + tz * ez
        # Contour: distance from the path, perpendicular to the tangent, split
        # into horizontal and vertical parts so they can be weighted separately.
        contour_error_sq = (
                (ex - lag_error * tx) ** 2 +
                (ey - lag_error * ty) ** 2
        )
        vertical_contour_error_sq = (ez - lag_error * tz) ** 2
        heading_error_sq = 2.0 * (1.0 - cos(psi - psi_path))

        self.model.set_expression('contour_error_sq', contour_error_sq)
        self.model.set_expression('vertical_contour_error_sq', vertical_contour_error_sq)
        self.model.set_expression('lag_error_sq', lag_error ** 2)
        self.model.set_expression('heading_error_sq', heading_error_sq)

        self.model.setup()

        self.mpc = do_mpc.controller.MPC(self.model)

        self.mpc.set_param(**self.MPC_SETUP_PARAMS)

        contour_error_sq = self.model.aux['contour_error_sq']
        vertical_contour_error_sq = self.model.aux['vertical_contour_error_sq']
        lag_error_sq = self.model.aux['lag_error_sq']
        heading_error_sq = self.model.aux['heading_error_sq']

        tracking_cost = (
                self.CONTOUR_WEIGHT * contour_error_sq +
                self.VERTICAL_CONTOUR_WEIGHT * vertical_contour_error_sq +
                self.LAG_WEIGHT * lag_error_sq +
                self.HEADING_WEIGHT * heading_error_sq
        )

        lterm = (
                tracking_cost +
                0.01 * vx ** 2 +
                self.VZ_WEIGHT * vz ** 2 +
                0.5 * omega ** 2 - 
                self.PROGRESS_REWARD * v_s -
                self.PROGRESS_POSITION_REWARD * s
        )

        mterm = tracking_cost

        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        self.mpc.set_rterm(
            vx=0.01,
            vz=self.VZ_CHANGE_WEIGHT,
            omega=1.0,
            v_s=self.PROGRESS_SPEED_CHANGE_WEIGHT,
        )

        self.mpc.bounds['lower', '_x', 's'] = 0.0
        self.mpc.bounds['upper', '_x', 's'] = self.path_length

        self.mpc.bounds['lower', '_u', 'v_s'] = 0.0
        self.mpc.bounds['upper', '_u', 'v_s'] = self.MAX_PROGRESS_SPEED

        self.mpc.bounds['lower', '_u', 'vx'] = 0.0
        self.mpc.bounds['upper', '_u', 'vx'] = 2.0

        self.mpc.bounds['lower', '_u', 'vz'] = -1.0
        self.mpc.bounds['upper', '_u', 'vz'] = 1.0

        self.mpc.bounds['lower', '_u', 'omega'] = -1.0
        self.mpc.bounds['upper', '_u', 'omega'] = 1.0

        self.mpc.setup()

        self.initial_guess_set = False
        self._try_set_initial_guess(self._get_drone_state())

    def _try_set_initial_guess(self, state):
        if state is None or self.mpc is None:
            return False

        self.mpc.x0 = np.array(state, dtype=float).reshape(-1, 1)
        self.mpc.set_initial_guess()
        self.initial_guess_set = True
        return True

    def step(self):
        if not self.has_path or self.mpc is None:
            return

        x = self._get_drone_state()
        if x is None:
            return

        if not self.initial_guess_set:
            self._try_set_initial_guess(x)

        start = self.get_clock().now()
        u = self.mpc.make_step(np.array(x, dtype=float).reshape(-1, 1))
        solve_time = (self.get_clock().now() - start).nanoseconds * 1e-9

        stats = self.mpc.solver_stats
        if not stats.get('success', False):
            self.get_logger().warn(
                f"MPC solve did not converge ({stats.get('return_status')}, "
                f"{solve_time:.3f} s); using best iterate.",
                throttle_duration_sec=1.0,
            )

        self._send_drone_velocity(u)

        flattened = np.array(u, dtype=float).reshape(-1)
        self.get_logger().info(
            f"state x={x[0]:.2f} y={x[1]:.2f} z={x[2]:.2f} psi={x[3]:.2f} "
            f"s={x[4]:.2f}/{self.path_length:.2f} -> "
            f"vx={flattened[0]:.2f} vz={flattened[1]:.2f} omega={flattened[2]:.2f} "
            f"v_s={flattened[3]:.2f} "
            f"(solve {solve_time:.3f} s)",
            throttle_duration_sec=1.0,
        )

    def _get_drone_state(self):
        try:
            t_world_body = self.tf_buffer.lookup_transform(
                self.get_parameter("world_frame").get_parameter_value().string_value,
                self.get_parameter("drone_frame").get_parameter_value().string_value,
                Time(),
            )
        except Exception as error:
            self.get_logger().warn(f"TF unavailable for drone state: {error}")
            return None

        position = t_world_body.transform.translation
        orientation = t_world_body.transform.rotation

        s = self._get_closest_s(position)

        psi = np.arctan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                     1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

        self.get_logger().info(f"Drone state: x={position.x:.2f} y={position.y:.2f} z={position.z:.2f} psi={psi:.2f} s={s:.2f}/{self.path_length:.2f}")

        # Keep psi continuous (no 2*pi jump when crossing +-180 deg) so the
        # MPC's warm start from the previous solution stays valid.
        if self.last_psi is not None:
            psi = self.last_psi + math.atan2(math.sin(psi - self.last_psi), math.cos(psi - self.last_psi))
        self.last_psi = psi

        state_vector = [position.x, position.y, position.z, psi, s]

        return state_vector

    def _get_closest_s(self, drone_position):
        p_drone = np.array([drone_position.x, drone_position.y, drone_position.z])

        distances = np.linalg.norm(self.path_points - p_drone, axis=1)

        closest_index = np.argmin(distances)

        # No look-ahead offset needed: the MPC advances the reference itself via v_s.
        s = self.s_samples[closest_index]

        return s

    def _send_drone_velocity(self, u):
        flattened = np.array(u, dtype=float).reshape(-1)
        vx = float(flattened[0]) if flattened.size > 0 else 0.0
        vz = float(flattened[1]) if flattened.size > 1 else 0.0
        yaw_rate = float(flattened[2]) if flattened.size > 2 else 0.0

        self.latest_body_command = (vx, vz, yaw_rate)
        if self.get_parameter("auto_arm").get_parameter_value().bool_value:
            self.offboard_requested = True

    def attitude_callback(self, message: VehicleAttitude):
        self.latest_attitude = message

    def vehicle_status_callback(self, message: VehicleStatus):
        self.latest_vehicle_status = message

        if message.nav_state == self.last_nav_state:
            return

        self.last_nav_state = message.nav_state
        self.get_logger().info(
            "PX4 nav_state=%d, accepts_offboard_setpoints=%s, failsafe=%s"
            % (
                message.nav_state,
                message.accepts_offboard_setpoints,
                message.failsafe,
            )
        )

    def publish_setpoint(self):
        timestamp = self.get_clock().now().nanoseconds // 1000

        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = timestamp
        offboard_control_mode.velocity = True
        offboard_control_mode.position = False
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False
        
        self.offboard_control_mode_publisher.publish(offboard_control_mode)
        self.setpoint_count += 1

        vx, vz, yaw_rate = self.latest_body_command
        if not self.has_path:
            vx, vz, yaw_rate = 0.0, 0.0, 0.0

        body_velocity = (vx, 0.0, -vz)
        ned_velocity = self.body_to_ned_velocity(body_velocity)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = [math.nan, math.nan, math.nan]
        setpoint.velocity = [float(value) for value in ned_velocity]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = math.nan
        setpoint.yawspeed = -yaw_rate 
        self.trajectory_setpoint_publisher.publish(setpoint)

        if not self.offboard_requested or self.setpoint_count < 40:
            return

        if (
            self.latest_vehicle_status.nav_state
            != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and timestamp - self.last_mode_command_us >= self.command_interval_us
        ):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
                param3=0.0,
            )
            self.last_mode_command_us = timestamp

        if (
            self.latest_vehicle_status.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
            and timestamp - self.last_arm_command_us >= self.command_interval_us
        ):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=VehicleCommand.ARMING_ACTION_ARM,
            )
            self.last_arm_command_us = timestamp

    def publish_vehicle_command(self, command: int, **parameters: float):
        message = VehicleCommand()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.command = command
        message.param1 = float(parameters.get("param1", 0.0))
        message.param2 = float(parameters.get("param2", 0.0))
        message.param3 = float(parameters.get("param3", 0.0))
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.vehicle_command_publisher.publish(message)

    def body_to_ned_velocity(self, velocity):
        """Rotate a velocity from PX4's FRD body frame into NED using yaw only.

        Roll and pitch are ignored so body x/y stay horizontal and body z stays
        vertical, matching the MPC model (vx horizontal, vz vertical).

        body_yaw_offset is added to PX4's yaw here (NED convention, clockwise
        positive) to account for the MPC's forward axis differing from
        PX4's body x-axis.
        """
        q = self.latest_attitude.q
        if len(q) < 4 or math.isclose(sum(value * value for value in q), 0.0):
            return velocity

        w, x, y, z = q[:4]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw += self.get_parameter("body_yaw_offset").get_parameter_value().double_value
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        body_x, body_y, body_z = velocity
        return (
            cos_yaw * body_x - sin_yaw * body_y,
            sin_yaw * body_x + cos_yaw * body_y,
            body_z,
        )



def main(args=None):
    rclpy.init(args=args)
    mpc_node = ModelPredictiveControlDroneFollower()

    try:
        rclpy.spin(mpc_node)
    finally:
        mpc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
