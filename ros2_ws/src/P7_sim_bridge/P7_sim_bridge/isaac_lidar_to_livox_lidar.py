import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from livox_ros_driver2.msg import CustomMsg, CustomPoint


EMITTER_ELEVATIONS_DEG = [
    -7.0,
    -2.575,
    1.849,
    6.274,
]

# Used until two scans have arrived, and if the measured period is implausible.
DEFAULT_SCAN_PERIOD_NS = 50_000_000  # the Isaac LiDAR publishes at 20 Hz
MAX_SCAN_PERIOD_NS = 200_000_000


class IsaacToLivox(Node):
    def __init__(self):
        super().__init__("isaac_to_livox")

        self.last_stamp_ns = None
        self.scan_period_ns = DEFAULT_SCAN_PERIOD_NS

        self.publisher = self.create_publisher(
            CustomMsg,
            "/livox/lidar",
            10,
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            "/isaac/lidar",
            self.pointcloud_callback,
            10,
        )

    @staticmethod
    def calculate_line(x: float, y: float, z: float) -> int:
        horizontal_distance = math.hypot(x, y)

        if horizontal_distance == 0.0:
            return 0

        elevation = math.degrees(
            math.atan2(z, horizontal_distance)
        )

        return min(
            range(len(EMITTER_ELEVATIONS_DEG)),
            key=lambda index: abs(
                EMITTER_ELEVATIONS_DEG[index] - elevation
            ),
        )

    def pointcloud_callback(self, message: PointCloud2):
        # Isaac stamps the scan at its end; FAST-LIO expects the start, with
        # per-point offsets covering one scan period.
        stamp_ns = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        if self.last_stamp_ns is not None:
            period_ns = stamp_ns - self.last_stamp_ns
            if period_ns == 0:
                return  # duplicate scan
            if 0 < period_ns <= MAX_SCAN_PERIOD_NS:
                self.scan_period_ns = period_ns
        self.last_stamp_ns = stamp_ns
        start_ns = max(0, stamp_ns - self.scan_period_ns)

        output = CustomMsg()
        output.header = message.header
        output.header.stamp.sec = start_ns // 1_000_000_000
        output.header.stamp.nanosec = start_ns % 1_000_000_000
        output.timebase = start_ns
        output.lidar_id = 0
        output.rsvd = [0, 0, 0]

        points = []

        valid_points = list(
            point_cloud2.read_points(
                message,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        )

        for point_index, point in enumerate(valid_points):
            x, y, z, intensity = point

            converted_point = CustomPoint()
            converted_point.x = float(x)
            converted_point.y = float(y)
            converted_point.z = float(z)

            converted_point.reflectivity = max(
                0,
                min(255, int(float(intensity))),
            )

            converted_point.line = self.calculate_line(x, y, z)

            # Approximate point timing in nanoseconds.
            # Replace this with real per-point timing if available.
            converted_point.offset_time = self.calculate_offset_time(
                point_index,
                len(valid_points),
            )

            # 0x00 is accepted by FAST-LIO's Livox parser.
            converted_point.tag = 0x00

            points.append(converted_point)

        output.points = points
        output.point_num = len(points)

        self.publisher.publish(output)

    def calculate_offset_time(
        self,
        point_index: int,
        point_count: int,
    ) -> int:
        if point_count <= 1:
            return 0

        return int(point_index * self.scan_period_ns / (point_count - 1))


def main():
    rclpy.init()
    node = IsaacToLivox()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()