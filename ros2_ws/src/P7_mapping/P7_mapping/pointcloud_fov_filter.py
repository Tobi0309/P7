import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo


class PointCloudFOVFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_fov_filter')

        self.camera_extrinsic = np.array([
            [-1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
        ])

        self.camera_intrinsic = np.zeros((3, 3))
        self.width = 1280
        self.height = 720

        self.pointcloud2_publisher = self.create_publisher(PointCloud2, '/cloud_registered_body_fov_filtered', 10)

        self.pointcloud2_subscriber = self.create_subscription(
            PointCloud2,
            '/cloud_registered_body',
            self.pointcloud2_callback,
            10
        )

        self.camera_intrinsic_subscriber = self.create_subscription(
            CameraInfo,
            "/UAV/rgbd_camera/color/camera_info",
            self.camera_intrinsic_callback,
            10
        )

    def pointcloud2_callback(self, pointcloud2_msg):
        coordinates = np.array(
            list(point_cloud2.read_points(
                pointcloud2_msg,
                field_names=("x","y","z"),
                skip_nans=True))
        )

        coordinates_formatted = np.array(coordinates.tolist())

        
        transformed_coordinates = (self.camera_intrinsic @ self.camera_extrinsic @ np.hstack((coordinates_formatted, np.ones((coordinates_formatted.shape[0], 1)))).T).T

        z = transformed_coordinates[:, 2]

        pixels_x = transformed_coordinates[:, 0] / (z + 1e-5)
        pixels_y = transformed_coordinates[:, 1] / (z + 1e-5)

        mask = (
            (z > 0.1) &
            (pixels_x >= 0) & 
            (pixels_x < self.width) &
            (pixels_y >= 0) & 
            (pixels_y < self.height)
        )

        masked_coordinates = coordinates_formatted[mask]

        masked_pointcloud = point_cloud2.create_cloud_xyz32(
            pointcloud2_msg.header,
            masked_coordinates.tolist(),
        )

        self.pointcloud2_publisher.publish(masked_pointcloud)

    def camera_intrinsic_callback(self, camera_info_msg):
        self.camera_intrinsic = np.array(camera_info_msg.k).reshape(3, 3)
        
        self.width = camera_info_msg.width
        self.height = camera_info_msg.height


def main():
    rclpy.init()
    node = PointCloudFOVFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
