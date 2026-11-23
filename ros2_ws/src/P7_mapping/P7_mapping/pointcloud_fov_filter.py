import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class PointCloudFOVFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_fov_filter')

        # /cloud_registered_body is in FAST-LIO's `body` frame, which is the
        # Mid-360 IMU frame, i.e. `body_imu` in the URDF. Look the camera up
        # from `body_imu` rather than `body`: the two are only joined through
        # camera_init, so TF would add the LIO/IMU-propagation pose offset.
        self.imu_frame = self.declare_parameter('imu_frame', 'body_imu').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_color_optical_frame').value

        # 3x4 [R|t] mapping IMU-frame points into camera_frame, filled from
        # the URDF static TF (robot_state_publisher). camera_frame must be an
        # optical frame (x right, y down, z forward), as CameraInfo K assumes.
        self.camera_extrinsic = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

    def lookup_camera_extrinsic(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.imu_frame, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'Waiting for {self.imu_frame} -> {self.camera_frame} TF: {e}',
                throttle_duration_sec=5.0)
            return False

        t = transform.transform.translation
        rotation = quaternion_to_matrix(transform.transform.rotation)
        translation = np.array([t.x, t.y, t.z])

        self.camera_extrinsic = np.hstack((rotation, translation.reshape(3, 1)))
        self.get_logger().info(
            f'Camera extrinsic ({self.imu_frame} -> {self.camera_frame}):\n'
            f'{np.array2string(self.camera_extrinsic, precision=4, suppress_small=True)}')
        return True

    def pointcloud2_callback(self, pointcloud2_msg):
        if self.camera_extrinsic is None and not self.lookup_camera_extrinsic():
            return

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

        in_front = z > 0.1
        mask = (
            in_front &
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



def main(args=None):
	rclpy.init(args=args)
	node = PointCloudFOVFilter()

	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()