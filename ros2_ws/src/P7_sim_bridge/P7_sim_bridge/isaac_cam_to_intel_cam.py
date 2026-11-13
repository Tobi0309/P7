#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class IsaacCamToIntelCam(Node):
    def __init__(self):
        super().__init__('isaac_cam_to_intel_cam')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/SIM/UAV/rgbd_camera/color/image_raw',
            self.image_callback,
            10,
        )

        self.publisher = self.create_publisher(
            CompressedImage,
            '/UAV/rgbd_camera/color/image_raw/compressed',
            10,
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover
            self.get_logger().warn(f'Failed to convert image: {exc}')
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
        ok, encoded = cv2.imencode('.jpg', cv_image, encode_param)

        if not ok:
            self.get_logger().warn('JPEG encoding failed')
            return

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = 'jpeg'
        compressed.data = np.array(encoded).tobytes()

        self.publisher.publish(compressed)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacCamToIntelCam()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
