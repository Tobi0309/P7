from collections.abc import Buffer

import rclpy
import numpy as np
import cv2

from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo


class LocalObstacleMapper(Node):
    def __init__(self):
        super().__init__('local_obstacle_mapper')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.buffer_length = 10000
        self.max_distance = 5.0
        self.buffer = np.zeros((self.buffer_length, 4))
        self.buffer[:, 3] = 1.0

        self.pointcloud2_subscriber = self.create_subscription(
            PointCloud2,
            '/cloud_registered_body',
            self.pointcloud2_callback,
            10
        )

        

    def pointcloud2_callback(self, pointcloud2_msg):
        tf_camera_body = self.get_tf_matrix('camera_init', 'body')
        tf_camera_drone = self.get_tf_matrix('camera_init', 'drone')

        if tf_camera_body is None or tf_camera_drone is None:
            return
        
        coordinates = np.array(
            list(
                point_cloud2.read_points(
                    pointcloud2_msg,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            )
        )

        coordinates_formatted = np.array(coordinates.tolist())

        mask = np.linalg.norm(coordinates_formatted, axis=1) <= self.max_distance

        coordinates_filtered = coordinates_formatted[mask]

        coordinates_filtered = np.column_stack((coordinates_filtered, np.ones(coordinates_filtered.shape[0])))

        self.buffer = np.roll(self.buffer, -coordinates_filtered.shape[0], axis=0)
        self.buffer[-coordinates_filtered.shape[0]:] = (tf_camera_body @ coordinates_filtered.T).T

        points = (np.linalg.inv(tf_camera_drone) @ self.buffer.T).T
        
        theta = np.arctan2(points[:, 1], points[:, 0])
        phi = np.arctan2(points[:, 2], np.sqrt(points[:, 0]**2 + points[:, 1]**2))

        hist, _, _ = np.histogram2d(theta, phi, bins=(90, 45), range=[[-np.pi, np.pi], [-np.pi/2, np.pi/2]])
        self.show_histogram(hist)

    def show_histogram(self, hist):
        # Convert to float for normalization
        hist_img = hist.astype(np.float32)

        # Normalize to 0-255
        hist_img = cv2.normalize(
            hist_img,
            None,
            alpha=0,
            beta=255,
            norm_type=cv2.NORM_MINMAX
        )

        hist_img = hist_img.astype(np.uint8)

        # Apply colormap for better visibility
        hist_img = cv2.applyColorMap(hist_img, cv2.COLORMAP_JET)

        # Make it larger for display
        hist_img = cv2.resize(
            hist_img,
            (900, 450),
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow("Occupancy Histogram", hist_img)
        cv2.waitKey(1)
    def get_tf_matrix(self, parent, child):
        try:
            tf_camera_body = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time()
            )

            matrix = np.eye(4)
            translation = tf_camera_body.transform.translation
            rotation = tf_camera_body.transform.rotation

            matrix[0:3, 3] = [translation.x, translation.y, translation.z]
            matrix[0:3, 0:3] = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()

            return matrix

        except Exception as e:
            self.get_logger().error(f"Could not get transform from {parent} to {child}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LocalObstacleMapper()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()