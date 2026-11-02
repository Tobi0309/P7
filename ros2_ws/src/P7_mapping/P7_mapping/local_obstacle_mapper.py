from collections.abc import Buffer

import rclpy
import numpy as np
import cv2

from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage


# # AI CREATED FUNCTION:
# def overlay_depth_map_on_camera(
#     depth_map,
#     camera_image,
#     camera_intrinsic,
#     lidar_to_camera,
#     alpha=0.5,
# ):
#     depth_map = np.asarray(depth_map, dtype=np.float32)
#     camera_image = np.asarray(camera_image)
#     camera_intrinsic = np.asarray(camera_intrinsic, dtype=np.float32)
#     lidar_to_camera = np.asarray(lidar_to_camera, dtype=np.float32)

#     if depth_map.ndim != 2:
#         raise ValueError("depth_map must be a 2D array")
#     if camera_image.ndim not in (2, 3):
#         raise ValueError("camera_image must be a grayscale or color image")
#     if camera_intrinsic.shape != (3, 3):
#         raise ValueError("camera_intrinsic must have shape (3, 3)")
#     if lidar_to_camera.shape != (4, 4):
#         raise ValueError("lidar_to_camera must have shape (4, 4)")
#     if not 0.0 <= alpha <= 1.0:
#         raise ValueError("alpha must be between 0 and 1")

#     # The mapper creates depth_map as (phi, theta).
#     phi_edges = np.linspace(-np.pi / 2, np.pi / 2, depth_map.shape[0] + 1)
#     theta_edges = np.linspace(-np.pi, np.pi, depth_map.shape[1] + 1)

#     valid = np.isfinite(depth_map) & (depth_map > 0)
#     phi_index, theta_index = np.nonzero(valid)
#     depths = depth_map[valid]
#     theta_corners = np.stack(
#         (
#             theta_edges[theta_index],
#             theta_edges[theta_index + 1],
#             theta_edges[theta_index + 1],
#             theta_edges[theta_index],
#         ),
#         axis=1,
#     )
#     phi_corners = np.stack(
#         (
#             phi_edges[phi_index],
#             phi_edges[phi_index],
#             phi_edges[phi_index + 1],
#             phi_edges[phi_index + 1],
#         ),
#         axis=1,
#     )

#     lidar_points = np.vstack(
#         (
#             (depths[:, None] * np.cos(phi_corners) * np.cos(theta_corners)).ravel(),
#             (depths[:, None] * np.cos(phi_corners) * np.sin(theta_corners)).ravel(),
#             (depths[:, None] * np.sin(phi_corners)).ravel(),
#             np.ones(depths.shape[0] * 4),
#         )
#     )

#     height, width = camera_image.shape[:2]
#     overlay = camera_image.copy()
#     depth_colors = cv2.normalize(
#         depths,
#         None,
#         alpha=0,
#         beta=255,
#         norm_type=cv2.NORM_MINMAX,
#     ).astype(np.uint8)
#     depth_colors = cv2.applyColorMap(depth_colors.reshape(-1, 1), cv2.COLORMAP_JET)[:, 0]

#     for bin_index, color in enumerate(depth_colors):
#         lidar_bin_corners = lidar_points[:, bin_index * 4:(bin_index + 1) * 4]
#         camera_bin_corners = lidar_to_camera @ lidar_bin_corners
#         depths_in_front = camera_bin_corners[2, :] > 0
#         if not np.any(depths_in_front):
#             continue

#         projected = camera_intrinsic @ camera_bin_corners[:3, depths_in_front]
#         pixels = np.rint(
#             projected[:2, :] / projected[2:3, :]
#         ).astype(np.int32)
#         in_image = (
#             (pixels[0, :] >= -width)
#             & (pixels[0, :] <= 2 * width)
#             & (pixels[1, :] >= -height)
#             & (pixels[1, :] <= 2 * height)
#         )
#         pixels = pixels[:, in_image]
#         if pixels.shape[1] < 3:
#             continue

#         polygon = pixels.T.reshape((-1, 1, 2))
#         if overlay.ndim == 2:
#             gray_color = cv2.cvtColor(
#                 color.reshape(1, 1, 3), cv2.COLOR_BGR2GRAY
#             )[0, 0]
#             cv2.fillPoly(overlay, [polygon], int(gray_color))
#         else:
#             cv2.fillPoly(overlay, [polygon], color.tolist())

#     return cv2.addWeighted(camera_image, 1.0 - alpha, overlay, alpha, 0)


class LocalObstacleMapper(Node):
    def __init__(self):
        super().__init__('local_obstacle_mapper')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.buffer_length = 10000
        self.max_distance = 5.0
        self.bin_count = [180, 90]

        self.buffer = np.zeros((4, self.buffer_length))
        self.buffer[3, :] = 1.0

        self.pointcloud2_subscriber = self.create_subscription(
            PointCloud2,
            '/cloud_registered_body',
            self.pointcloud2_callback,
            10
        )

        self._test_img = np.zeros((480, 640, 3), dtype=np.uint8)

        self._test_camera_feed_subscriber = self.create_subscription(
            CompressedImage,
            '/UAV/rgbd_camera/color/image_raw/compressed',
            self._test_camera_feed_callback,
            10
        )

    def _test_camera_feed_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self._test_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

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

        coordinates_filtered = np.column_stack((coordinates_filtered, np.ones(coordinates_filtered.shape[0]))).T

        self.buffer = np.roll(self.buffer, -coordinates_filtered.shape[1], axis=1)

        self.buffer[:, -coordinates_filtered.shape[1]:] = tf_camera_body @ coordinates_filtered
        
        points = np.linalg.inv(tf_camera_drone) @ self.buffer   

        theta = np.arctan2(points[1, :], points[0, :])
        phi = np.arctan2(points[2, :], np.sqrt(points[0, :]**2 + points[1, :]**2))

        dist = np.linalg.norm(points[0:3, :], axis=0)

        theta_idx = np.digitize(theta, np.linspace(-np.pi, np.pi, self.bin_count[0])) - 1
        phi_idx = np.digitize(phi, np.linspace(-np.pi/2, np.pi/2, self.bin_count[1])) - 1

        theta_idx = np.clip(theta_idx, 0, self.bin_count[0] - 1)
        phi_idx = np.clip(phi_idx, 0, self.bin_count[1] - 1)

        depth_map = np.full((self.bin_count[1], self.bin_count[0]), self.max_distance)
        np.minimum.at(depth_map, (phi_idx, theta_idx), dist)

    #     K = np.array([
    #         [634.7300415039062,   0.0,              640.40478515625],
    #         [0.0,                 634.0554809570312, 370.4395446777344],
    #         [0.0,                 0.0,                1.0]
    #     ])

    #     T_camera_lidar = np.array([
    #         [-1.0, 0.0, 0.0,  0.0],
    #         [ 0.0, 0.0, 1.0,  0.0769],
    #         [ 0.0, 1.0, 0.0, -0.080836],
    #         [ 0.0, 0.0, 0.0,  1.0],
    #     ])


    #     depth_overlay = overlay_depth_map_on_camera(
    #         depth_map,
    #         self._test_img,
    #         K,
    #         T_camera_lidar,
    #         alpha=0.5
    #     )

    #     cv2.imshow("Depth Overlay", depth_overlay)
    #     cv2.waitKey(1)

    # def show_histogram(self, hist):
    #     # Convert to float for normalization
    #     hist_img = hist.astype(np.float32)

    #     # Normalize to 0-255
    #     hist_img = cv2.normalize(
    #         hist_img,
    #         None,
    #         alpha=255,
    #         beta=0,
    #         norm_type=cv2.NORM_MINMAX
    #     )

    #     hist_img = np.rot90(hist_img.astype(np.uint8))

    #     # Apply colormap for better visibility
    #     hist_img = cv2.applyColorMap(hist_img, cv2.COLORMAP_JET)

    #     # Make it larger for display
    #     hist_img = cv2.resize(
    #         hist_img,
    #         (900, 450),
    #         interpolation=cv2.INTER_NEAREST
    #     )

    #     cv2.imshow("Occupancy Histogram", hist_img)
    #     cv2.waitKey(1)

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