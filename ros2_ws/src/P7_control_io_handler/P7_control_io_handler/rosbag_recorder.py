from datetime import datetime

import rclpy
from px4_msgs.msg import RcChannels
from rclpy.node import Node
from rosbag2_interfaces.srv import Record, Stop


class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('control_io_rosbag_recorder')

        self.RC_CHANNEL = RcChannels.FUNCTION_RETURN

        self._last_aux2_value = None
        self.record_status = True
        
        self._record_client = self.create_client(
            Record, '/rosbag2_recorder/record')
        self._stop_client = self.create_client(
            Stop, '/rosbag2_recorder/stop')
        
        self.create_subscription(
            RcChannels,
            '/fmu/out/rc_channels',
            self._rc_channels_callback,
            10,
        )

    def _rc_channels_callback(self, message):
        if len(message.channels) <= self.RC_CHANNEL:
            self.get_logger().warning('RcChannels has no FUNCTION_RETURN value')
            return

        aux2_value = message.channels[self.RC_CHANNEL]
        previous_value = self._last_aux2_value
        self._last_aux2_value = aux2_value

        if previous_value is None:
            return

        if previous_value >= -0.5 and aux2_value < -0.5:
            self._start_recording()
        elif previous_value <= 0.5 and aux2_value > 0.5:
            self._stop_recording()

    def _start_recording(self):
        if self.record_status:
            return

        if not self._record_client.service_is_ready():
            self.get_logger().warning('Record service is not available')
            return

        self.record_status = True
        
        name = datetime.now().strftime('rosbag_%Y%m%d_%H%M%S')
        
        request = Record.Request()
        request.uri = f'rosbags/{name}/'
        
        self._record_client.call_async(request)
        self.get_logger().info(f'Starting recording: {request.uri}')

    def _stop_recording(self):
        if not self.record_status:
            return
        
        if not self._stop_client.service_is_ready():
            self.get_logger().warning('Stop service is not available')
            return

        self.record_status = False

        self._stop_client.call_async(Stop.Request())
        self.get_logger().info('Stopping recording')


def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
