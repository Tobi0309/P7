import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)
from quadrotor_msgs.msg import PositionCommand


class PosCmdToPx4(Node):
    """Translate planner position commands to PX4 offboard setpoints."""

    def __init__(self):
        super().__init__("pos_cmd_to_px4")

        self.latest_position_command = None
        self.latest_vehicle_status = VehicleStatus()
        self.last_nav_state = None
        self.last_mode_command_us = 0
        self.last_arm_command_us = 0
        self.command_interval_us = 200_000
        self.offboard_requested = False

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

        self.position_command_subscription = self.create_subscription(
            PositionCommand,
            "/drone_0_planning/pos_cmd",
            self.position_command_callback,
            10,
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

    def vehicle_status_callback(self, message: VehicleStatus):
        self.latest_vehicle_status = message

        if message.nav_state == self.last_nav_state:
            return

        self.last_nav_state = message.nav_state
        self.get_logger().info(
            "PX4 nav_state=%d, accepts_offboard_setpoints=%s, failsafe=%s",
            message.nav_state,
            message.accepts_offboard_setpoints,
            message.failsafe,
        )

    def publish_setpoint(self):
        if self.latest_position_command is None:
            return

        timestamp = self.get_clock().now().nanoseconds // 1000
        command = self.latest_position_command

        velocity_vector = (
            command.velocity.x,
            command.velocity.y,
            command.velocity.z,
        )
        acceleration_vector = (
            command.acceleration.x,
            command.acceleration.y,
            command.acceleration.z,
        )
        use_velocity = any(abs(value) > 1e-6 for value in velocity_vector)
        use_acceleration = any(abs(value) > 1e-6 for value in acceleration_vector)

        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = timestamp
        offboard_control_mode.position = True
        offboard_control_mode.velocity = use_velocity
        offboard_control_mode.acceleration = use_acceleration
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False
        self.offboard_control_mode_publisher.publish(offboard_control_mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position = self.enu_to_ned(command.position.x, command.position.y, command.position.z)
        setpoint.velocity = (
            self.enu_to_ned(*velocity_vector) if use_velocity else [math.nan, math.nan, math.nan]
        )
        setpoint.acceleration = (
            self.enu_to_ned(*acceleration_vector) if use_acceleration else [math.nan, math.nan, math.nan]
        )
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = self.enu_yaw_to_ned(command.yaw)
        setpoint.yawspeed = self.enu_yaw_rate_to_ned(command.yaw_dot)
        self.trajectory_setpoint_publisher.publish(setpoint)

        if not self.offboard_requested:
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

    @staticmethod
    def enu_to_ned(x: float, y: float, z: float):
        # Planner/world commands are treated as ENU (x=east, y=north, z=up),
        # while PX4 TrajectorySetpoint expects NED (x=north, y=east, z=down).
        return [float(y), float(x), float(-z)]

    @staticmethod
    def enu_yaw_to_ned(yaw: float):
        # ENU yaw is CCW-positive; PX4 NED yaw is opposite in sign.
        return -float(yaw)

    @staticmethod
    def enu_yaw_rate_to_ned(yaw_rate: float):
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
