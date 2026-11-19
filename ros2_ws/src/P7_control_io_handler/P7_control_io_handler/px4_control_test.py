import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
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
    """Convert camera_init position commands to PX4 FRD setpoints."""

    def __init__(self):
        super().__init__("pos_cmd_to_px4")

        self.declare_parameter("position_command_topic", "/drone_0_planning/pos_cmd")
        self.declare_parameter("goal_topic", "/move_base_simple/goal")
        self.declare_parameter("px4_odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("px4_odom_timeout_sec", 0.25)
        self.px4_odom_timeout_sec = float(self.get_parameter("px4_odom_timeout_sec").value)

        self.latest_position_command = None
        self.latest_vehicle_status = VehicleStatus()
        self.last_nav_state = None
        self.last_mode_command_us = 0
        self.last_arm_command_us = 0
        self.command_interval_us = 200_000
        self.offboard_requested = False
        self.goal_received = False
        self.last_px4_odom_wall_time = None
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
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("goal_topic").value),
            self.goal_callback,
            10,
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

    def goal_callback(self, _message: PoseStamped):
        if not self.goal_received:
            self.get_logger().info("Goal received by EGO-Planner; PX4 offboard may be requested.")
        self.goal_received = True
        self.offboard_requested = True

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
            self.get_logger().warning(
                "PX4 odometry reset detected; resetting PX4 timestamp anchor"
            )
        ros_time_us = self.get_clock().now().nanoseconds // 1000
        self.px4_time_anchor = (ros_time_us, int(message.timestamp))
        self.last_px4_odom_wall_time = time.monotonic()

    def publish_setpoint(self):
        if self.latest_position_command is None:
            return
        if self.px4_time_anchor is None:
            self.warn_alignment("Waiting for PX4 odometry clock")
            return

        if (self.last_px4_odom_wall_time is None or
                time.monotonic() - self.last_px4_odom_wall_time > self.px4_odom_timeout_sec):
            self.warn_alignment("PX4 odometry is stale; withholding setpoints")
            return

        command = self.latest_position_command
        if command.header.frame_id != "camera_init":
            self.warn_alignment(
                "Command frame '%s' is not camera_init; withholding setpoints"
                % command.header.frame_id
            )
            return

        timestamp = self.px4_timestamp_us()
        if timestamp is None:
            self.warn_alignment("Waiting for a valid PX4 timestamp anchor")
            return
        position = self.camera_init_to_px4_frd(
            command.position.x, command.position.y, command.position.z
        )
        velocity = self.camera_init_to_px4_frd(
            command.velocity.x, command.velocity.y, command.velocity.z
        )
        acceleration = self.camera_init_to_px4_frd(
            command.acceleration.x,
            command.acceleration.y,
            command.acceleration.z,
        )
        use_velocity = any(abs(value) > 1e-6 for value in velocity)
        use_acceleration = any(abs(value) > 1e-6 for value in acceleration)

        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = True
        mode.acceleration = True
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False
        self.offboard_control_mode_publisher.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = [float(value) for value in position]
        setpoint.velocity = (
            [float(value) for value in velocity] if use_velocity else [math.nan, math.nan, math.nan]
        )
        setpoint.acceleration = (
            [float(value) for value in acceleration] if use_acceleration else [math.nan, math.nan, math.nan]
        )
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = self.camera_init_yaw_to_px4_frd(command.yaw)
        setpoint.yawspeed = self.camera_init_yaw_rate_to_px4_frd(command.yaw_dot)
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

    def warn_alignment(self, message: str):
        now = time.monotonic()
        if now - self.last_alignment_warning >= 2.0:
            self.get_logger().warning(message)
            self.last_alignment_warning = now

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
    def camera_init_to_px4_frd(x: float, y: float, z: float):
        # camera_init -> px4_frd is a 180 degree rotation about X.
        return [float(x), float(-y), float(-z)]

    @staticmethod
    def camera_init_yaw_to_px4_frd(yaw: float):
        # The vehicle's forward axis is opposite the planner's yaw-zero axis.
        # Flip yaw handedness for FRD, then rotate the heading by 180 degrees.
        px4_yaw = -float(yaw) + math.pi
        return math.atan2(math.sin(px4_yaw), math.cos(px4_yaw))

    @staticmethod
    def camera_init_yaw_rate_to_px4_frd(yaw_rate: float):
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
