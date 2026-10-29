import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleAttitude
from px4_msgs.msg import VehicleCommand


class JoyToDroneVelocities(Node):
	"""Convert joystick input into PX4 offboard velocity setpoints."""

	def __init__(self):
		super().__init__("joy_to_drone_velocities")

		self.max_speed = self.declare_parameter("max_speed", 3.0).value
		self.max_vertical_speed = self.declare_parameter(
			"max_vertical_speed", 1.0
		).value
		self.max_turn_rate = self.declare_parameter(
			"max_turn_rate", 1.0
		).value

		self.latest_joy = Joy()
		self.latest_attitude = VehicleAttitude()
		self.latest_attitude.q = [1.0, 0.0, 0.0, 0.0]
		self.arm_button_was_pressed = False

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

		self.joy_subscription = self.create_subscription(
			Joy,
			"/joy",
			self.joy_callback,
			10,
		)
		self.attitude_subscription = self.create_subscription(
			VehicleAttitude,
			"/fmu/out/vehicle_attitude",
			self.attitude_callback,
			qos_profile_sensor_data,
		)
		self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)

	def joy_callback(self, message: Joy):
		self.latest_joy = message

		arm_button_is_pressed = self.button_value(message, 1) == 1
		if arm_button_is_pressed and not self.arm_button_was_pressed:
			self.publish_vehicle_command(
				VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
				param1=1.0,
				param2=6.0,
				param3=0.0,
			)
			self.publish_vehicle_command(
				VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
				param1=VehicleCommand.ARMING_ACTION_ARM,
			)

		self.arm_button_was_pressed = arm_button_is_pressed

	def attitude_callback(self, message: VehicleAttitude):
		self.latest_attitude = message

	def publish_setpoint(self):
		timestamp = self.get_clock().now().nanoseconds // 1000

		offboard_control_mode = OffboardControlMode()
		offboard_control_mode.timestamp = timestamp
		offboard_control_mode.velocity = True
		self.offboard_control_mode_publisher.publish(offboard_control_mode)

		left_right = self.axis_value(self.latest_joy, 0)
		front_back = self.axis_value(self.latest_joy, 1)
		throttle = self.axis_value(self.latest_joy, 2)
		turning = self.axis_value(self.latest_joy, 3)
		body_velocity = (
			front_back * self.max_speed,
			left_right * self.max_speed,
			-throttle * self.max_vertical_speed,
		)
		ned_velocity = self.body_to_ned_velocity(body_velocity)

		setpoint = TrajectorySetpoint()
		setpoint.timestamp = timestamp
		setpoint.position = [math.nan, math.nan, math.nan]
		setpoint.velocity = list(ned_velocity)
		setpoint.acceleration = [math.nan, math.nan, math.nan]
		setpoint.jerk = [math.nan, math.nan, math.nan]
		setpoint.yaw = math.nan
		setpoint.yawspeed = turning * self.max_turn_rate
		self.trajectory_setpoint_publisher.publish(setpoint)

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
	def axis_value(message: Joy, index: int) -> float:
		if index >= len(message.axes):
			return 0.0
		return float(message.axes[index])

	@staticmethod
	def button_value(message: Joy, index: int) -> int:
		if index >= len(message.buttons):
			return 0
		return int(message.buttons[index])

	def body_to_ned_velocity(self, velocity):
		"""Rotate a velocity from PX4's FRD body frame into NED."""
		q = self.latest_attitude.q
		if len(q) < 4 or math.isclose(sum(value * value for value in q), 0.0):
			return velocity

		w, x, y, z = q[:4]
		body_x, body_y, body_z = velocity
		return (
			(1 - 2 * (y * y + z * z)) * body_x
			+ 2 * (x * y - w * z) * body_y
			+ 2 * (x * z + w * y) * body_z,
			2 * (x * y + w * z) * body_x
			+ (1 - 2 * (x * x + z * z)) * body_y
			+ 2 * (y * z - w * x) * body_z,
			2 * (x * z - w * y) * body_x
			+ 2 * (y * z + w * x) * body_y
			+ (1 - 2 * (x * x + y * y)) * body_z,
		)


def main(args=None):
	rclpy.init(args=args)
	node = JoyToDroneVelocities()

	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
