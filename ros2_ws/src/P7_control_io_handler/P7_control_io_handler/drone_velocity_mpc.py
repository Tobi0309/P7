import do_mpc
import numpy as np
import matplotlib.pyplot as plt
import rclpy

from casadi import *
from scipy.interpolate import CubicSpline
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


class ModelPredictiveControlDroneFollower(Node):
    def __init__(self):
        super().__init__("drone_velocity_mpc")

        self.declare_parameter("path_topic", "/drone_path")

        self.MPC_SETUP_PARAMS = {
            'n_horizon': 30,
            't_step': 0.1,
            'state_discretization': 'collocation',
            'store_full_solution': True
        }

        self.PATH_RESOLUTION = 1000

        self.path_spline = None
        self.path_length = 0
        self.s_samples = []
        self.tangent_samples = []
        self.path_points = []
        self.waypoints = []

        self.model = None
        self.mpc = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.step_timer = self.create_timer(0.1, self.step)

        self.drone_path_subscriber = self.create_subscription(
            msg_type=Path,
            topic=self.get_parameter("path_topic").get_parameter_value().string_value,
            callback=self.drone_path_callback,
            qos_profile=qos_profile_sensor_data
        )


    def drone_path_callback(self, msg):
        self.waypoints = np.array([[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in msg.poses])
                
        self.update_path()

    def update_path(self):
        segment_lengths = np.linalg.norm(np.diff(self.waypoints, axis=0), axis=1)

        s_waypoints = np.concatenate(([0.0], np.cumsum(segment_lengths)))

        self.path_length = s_waypoints[-1]
        self.path_spline = CubicSpline(s_waypoints, self.waypoints, axis=0)

        self.s_samples = np.linspace(0.0, self.path_length, self.PATH_RESOLUTION)
        self.tangent_samples = self.path_spline.derivative()(self.s_samples)

        self.path_points = self.path_spline(self.s_samples)

        self._build_model()

    def _build_model(self):
        x_samples = self.path_points[:, 0]
        y_samples = self.path_points[:, 1]
        z_samples = self.path_points[:, 2]
        psi_samples = np.unwrap(np.arctan2(self.tangent_samples[:, 1], self.tangent_samples[:, 0]))

        self.model = do_mpc.model.Model('continuous')
        px = self.model.set_variable('_x', 'px')
        py = self.model.set_variable('_x', 'py')
        pz = self.model.set_variable('_x', 'pz')
        psi = self.model.set_variable('_x', 'psi')
        s = self.model.set_variable('_x', 's')

        vx = self.model.set_variable('_u', 'vx')
        vz = self.model.set_variable('_u', 'vz')
        omega = self.model.set_variable('_u', 'omega')

        path_x = interpolant('path_x', 'bspline', [self.s_samples], x_samples)
        path_y = interpolant('path_y', 'bspline', [self.s_samples], y_samples)
        path_z = interpolant('path_z', 'bspline', [self.s_samples], z_samples)
        path_psi = interpolant('path_psi', 'bspline', [self.s_samples], psi_samples)

        self.model.set_rhs('px', vx * cos(psi))
        self.model.set_rhs('py', vx * sin(psi))
        self.model.set_rhs('pz', vz)
        self.model.set_rhs('psi', omega)
        self.model.set_rhs('s', sqrt(vx ** 2 + vz ** 2))

        px_path = path_x(s)
        py_path = path_y(s)
        pz_path = path_z(s)
        psi_path = path_psi(s)

        position_error = ((px - px_path) ** 2 + (py - py_path) ** 2 + (pz - pz_path) ** 2)
        heading_error = atan2(sin(psi - psi_path), cos(psi - psi_path))

        self.model.set_expression('pos_error', position_error)
        self.model.set_expression('heading_error', heading_error)

        self.model.setup()

        self.mpc = do_mpc.controller.MPC(self.model)

        self.mpc.set_param(**self.MPC_SETUP_PARAMS)

        pos_error = self.model.aux['pos_error']
        heading_error = self.model.aux['heading_error']

        lterm = (
                25.0 * pos_error +
                50.0 * heading_error ** 2 +
                0.01 * vx ** 2 +
                0.01 * vz ** 2 +
                0.01 * omega ** 2
        )

        mterm = (
                50.0 * pos_error +
                75.0 * heading_error ** 2 -
                5.0 * s
        )

        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        self.mpc.set_rterm(vx=0.01, vz=0.01, omega=0.01)

        self.mpc.bounds['lower', '_x', 's'] = 0.0
        self.mpc.bounds['upper', '_x', 's'] = self.path_length

        self.mpc.bounds['lower', '_u', 'vx'] = 0.0
        self.mpc.bounds['upper', '_u', 'vx'] = 2.0

        self.mpc.bounds['lower', '_u', 'vz'] = -1.0
        self.mpc.bounds['upper', '_u', 'vz'] = 1.0

        self.mpc.bounds['lower', '_u', 'omega'] = -2.0
        self.mpc.bounds['upper', '_u', 'omega'] = 2.0

        self.mpc.setup()

        self.mpc.x0 = self._get_drone_state()
        self.mpc.set_initial_guess()

    def step(self):
        x = self._get_drone_state()
        #u = self.mpc.make_step(x)
        #self._send_drone_velocity(u)

    def _get_drone_state(self):
        t_world_body = self.tf_buffer.lookup_transform("world", "drone", rclpy.Time())

        position = t_world_body.transform.translation
        orientation = t_world_body.transform.rotation

        s = self._get_closest_s(position)

        psi = np.arctan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                     1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))

        state_vector = [position.x, position.y, position.z, psi, s]

        self.get_logger().info(f"Drone position: {state_vector}")

        return state_vector

    def _get_closest_s(self, drone_position):
        p_drone = np.array([drone_position.x, drone_position.y, drone_position.z])

        distances = np.linalg.norm(self.path_points - p_drone, axis=1)

        closest_index = np.argmin(distances)

        s = self.s_samples[closest_index]

        return s

    def _send_drone_velocity(self, u):
        print(f"Sent {u} to drone")


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
    
