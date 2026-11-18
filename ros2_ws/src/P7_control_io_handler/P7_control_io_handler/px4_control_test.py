import math
import time
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from quadrotor_msgs.msg import PositionCommand


class PosCmdToPx4(Node):
    """Transform EGO setpoints into PX4 NED using paired live odometry."""

    def __init__(self):
        super().__init__("pos_cmd_to_px4")

        self.declare_parameter("position_command_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("ego_odometry_topic", "/Odometry_base_link")
        self.declare_parameter("px4_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("alignment_mode", "paired_pose")
        self.declare_parameter("max_pair_skew_sec", 0.05)
        self.declare_parameter("max_alignment_age_sec", 0.25)
        self.max_pair_skew_sec = float(self.get_parameter("max_pair_skew_sec").value)
        self.max_alignment_age_sec = float(self.get_parameter("max_alignment_age_sec").value)
        self.alignment_mode = str(self.get_parameter("alignment_mode").value)

        self.latest_position_command = None
        self.latest_vehicle_status = VehicleStatus()
        self.last_nav_state = None
        self.last_mode_command_us = 0
        self.last_arm_command_us = 0
        self.command_interval_us = 200_000
        self.offboard_requested = False
        self.ego_poses = deque(maxlen=40)
        self.px4_poses = deque(maxlen=40)
        self.alignment = None
        self.px4_reset_counter = None
        self.px4_time_anchor = None
        self.last_alignment_warning = 0.0

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self.position_command_subscription = self.create_subscription(
            PositionCommand,
            str(self.get_parameter("position_command_topic").value),
            self.position_command_callback,
            10,
        )
        self.ego_odometry_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("ego_odometry_topic").value),
            self.ego_odometry_callback,
            qos_profile_sensor_data,
        )
        self.px4_odometry_subscription = self.create_subscription(
            VehicleOdometry,
            str(self.get_parameter("px4_odometry_topic").value),
            self.px4_odometry_callback,
            qos_profile_sensor_data,
        )
        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_sensor_data,
        )
        self.setpoint_timer = self.create_timer(0.02, self.publish_setpoint)

    def position_command_callback(self, message: PositionCommand):
        self.latest_position_command = message
        self.offboard_requested = True

    def ego_odometry_callback(self, message: Odometry):
        pose = self.pose_from_ego(message)
        if pose is None:
            return
        self.ego_poses.append((time.monotonic(), message.header.frame_id, pose))
        self.update_alignment()

    def px4_odometry_callback(self, message: VehicleOdometry):
        if message.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.warn_alignment(
                "PX4 odometry pose_frame is not NED; refusing to transform setpoints"
            )
            return
        if self.px4_reset_counter is None:
            self.px4_reset_counter = message.reset_counter
        elif message.reset_counter != self.px4_reset_counter:
            self.px4_reset_counter = message.reset_counter
            self.alignment = None
            self.get_logger().warning(
                "PX4 odometry reset detected; recalibrating frame alignment"
            )
        pose = self.pose_from_px4(message)
        if pose is None:
            self.warn_alignment("PX4 odometry has an invalid pose")
            return
        ros_time_us = self.get_clock().now().nanoseconds // 1000
        self.px4_time_anchor = (ros_time_us, int(message.timestamp))
        self.px4_poses.append((time.monotonic(), pose))
        self.update_alignment()

    @staticmethod
    def pose_from_ego(message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        position = (float(p.x), float(p.y), float(p.z))
        quaternion = (float(q.w), float(q.x), float(q.y), float(q.z))
        return PosCmdToPx4.normalized_pose(position, quaternion)

    @staticmethod
    def pose_from_px4(message):
        position = tuple(float(value) for value in message.position)
        quaternion = tuple(float(value) for value in message.q)
        return PosCmdToPx4.normalized_pose(position, quaternion)

    @staticmethod
    def normalized_pose(position, quaternion):
        if not all(math.isfinite(value) for value in position + quaternion):
            return None
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-6:
            return None
        return position, tuple(value / norm for value in quaternion)

    def update_alignment(self):
        # The transform between odometry frames is fixed. Re-estimating it
        # while PX4 tracks a recorded EGO trajectory makes the setpoint move
        # with the difference between the two vehicles' motion.
        if self.alignment is not None:
            return
        if not self.ego_poses or not self.px4_poses:
            return
        ego_time, ego_frame, ego_pose = self.ego_poses[-1]
        px4_time, px4_pose = self.px4_poses[-1]
        if abs(ego_time - px4_time) > self.max_pair_skew_sec:
            return

        ego_position, ego_quaternion = ego_pose
        px4_position, px4_quaternion = px4_pose
        if self.alignment_mode == "fixed_180_z":
            # Test mode: assume camera_init and PX4 NED differ by a half-turn
            # around Z. Keep a translation from the paired positions so their
            # origins align at calibration time.
            q_align = (0.0, 0.0, 0.0, 1.0)
        elif self.alignment_mode == "paired_pose":
            # EGO odometry uses base_link FLU axes. PX4 uses body FRD axes.
            q_flu_frd = (0.0, 1.0, 0.0, 0.0)
            q_align = self.quaternion_multiply(
                self.quaternion_multiply(px4_quaternion, q_flu_frd),
                self.quaternion_conjugate(ego_quaternion),
            )
        else:
            self.warn_alignment(
                "Unknown alignment_mode '%s'; use paired_pose or fixed_180_z"
                % self.alignment_mode
            )
            return
        rotated_ego_position = self.rotate_vector(q_align, ego_position)
        translation = tuple(
            px4_position[i] - rotated_ego_position[i] for i in range(3)
        )
        self.alignment = (q_align, translation, ego_frame, max(ego_time, px4_time))
        self.get_logger().info(
            "Locked EGO-to-PX4 alignment mode=%s; translation=(%.3f, %.3f, %.3f) m"
            % ((self.alignment_mode,) + translation)
        )

    def publish_setpoint(self):
        if self.latest_position_command is None:
            return
        if not self.alignment or self.px4_time_anchor is None:
            self.warn_alignment("Waiting for paired odometry and PX4 clock")
            return

        q_align, translation, ego_frame, _alignment_time = self.alignment
        now = time.monotonic()
        if (
            not self.ego_poses
            or not self.px4_poses
            or now - self.ego_poses[-1][0] > self.max_alignment_age_sec
            or now - self.px4_poses[-1][0] > self.max_alignment_age_sec
        ):
            self.warn_alignment("Odometry alignment is stale; withholding setpoints")
            return

        command = self.latest_position_command
        if command.header.frame_id and command.header.frame_id != ego_frame:
            self.warn_alignment(
                "Command frame '%s' does not match EGO odometry frame '%s'"
                % (command.header.frame_id, ego_frame)
            )
            return

        timestamp = self.px4_timestamp_us()
        if timestamp is None:
            self.warn_alignment("Waiting for a valid PX4 timestamp anchor")
            return
        position = (command.position.x, command.position.y, command.position.z)
        velocity = (command.velocity.x, command.velocity.y, command.velocity.z)
        acceleration = (
            command.acceleration.x,
            command.acceleration.y,
            command.acceleration.z,
        )
        use_velocity = any(abs(value) > 1e-6 for value in velocity)
        use_acceleration = any(abs(value) > 1e-6 for value in acceleration)

        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        mode.velocity = use_velocity
        mode.acceleration = use_acceleration
        mode.attitude = False
        mode.body_rate = False
        self.offboard_control_mode_publisher.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = self.flu_to_frd(command.position.x, command.position.y, command.position.z)
        setpoint.velocity = (
            self.flu_to_frd(*velocity_vector) if use_velocity else [math.nan, math.nan, math.nan]
        )
        setpoint.acceleration = (
            self.flu_to_frd(*acceleration_vector) if use_acceleration else [math.nan, math.nan, math.nan]
        )
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = self.flu_yaw_to_frd(command.yaw)
        setpoint.yawspeed = self.flu_yaw_rate_to_frd(command.yaw_dot)
        self.trajectory_setpoint_publisher.publish(setpoint)

        self.publish_offboard_commands(timestamp)

    def publish_offboard_commands(self, timestamp):
        if not self.offboard_requested:
            return
        if (
            self.latest_vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and timestamp - self.last_mode_command_us >= self.command_interval_us
        ):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                timestamp,
                param1=1.0,
                param2=6.0,
            )
            self.last_mode_command_us = timestamp
        if (
            self.latest_vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED
            and timestamp - self.last_arm_command_us >= self.command_interval_us
        ):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                timestamp,
                param1=VehicleCommand.ARMING_ACTION_ARM,
            )
            self.last_arm_command_us = timestamp

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

    def px4_timestamp_us(self):
        if self.px4_time_anchor is None:
            return None
        ros_anchor_us, px4_anchor_us = self.px4_time_anchor
        ros_now_us = self.get_clock().now().nanoseconds // 1000
        elapsed_us = max(0, ros_now_us - ros_anchor_us)
        return px4_anchor_us + elapsed_us

    def publish_vehicle_command(self, command: int, timestamp: int, **parameters: float):
        message = VehicleCommand()
        message.timestamp = timestamp
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

    @staticmethod
    def flu_to_frd(x: float, y: float, z: float):
        # Planner commands are in camera_init (z-up), computed from
        # /Odometry_base_link. The PX4 odometry bridge feeds PX4 that same
        # base_link odometry with camera_init rotated 180 deg about X
        # (z-down) and base_link FLU -> FRD, so PX4's local frame is
        # camera_init with y and z negated.
        return [float(x), float(-y), float(-z)]

    @staticmethod
    def flu_yaw_to_frd(yaw: float):
        # 180 deg rotation about X keeps the heading reference (x axis) but
        # flips the yaw direction: FLU yaw is CCW-positive, FRD is CW-positive.
        return -float(yaw)

    @staticmethod
    def flu_yaw_rate_to_frd(yaw_rate: float):
        return -float(yaw_rate)

def main(args=None):
    rclpy.init(args=args)
    node = PosCmdToPx4()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
