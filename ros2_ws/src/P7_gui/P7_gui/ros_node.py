"""ROS side of the P7 GUI.

All callbacks run in the executor thread; the Qt thread reads a snapshot under `lock`.
Times used for the mission timer come from the node clock, so they follow /clock when
use_sim_time is true and wall time otherwise.
"""

import math
import threading
import time

import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, UInt32
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

try:
    from frontier_viewpoint_optimizer.msg import OptimizedFrontierArray
except ImportError:  # GUI still works without the frontier message package
    OptimizedFrontierArray = None


def _stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)


class GuiNode(Node):
    def __init__(self):
        super().__init__('p7_gui')
        self.state_topic = self.declare_parameter('state_topic', '/frontier_viewpoint_selection/state').value
        self.odom_topic = self.declare_parameter('odometry_topic', '/Odometry_base_link').value
        self.start_service = self.declare_parameter('start_service', '/frontier_viewpoint_selection/start').value
        self.finish_service = self.declare_parameter('finish_service', '/frontier_viewpoint_selection/finish').value
        self.viewpoints_topic = self.declare_parameter('viewpoints_topic', '/frontier_viewpoints').value
        self.batch_topic = self.declare_parameter('batch_request_topic', '/frontier_navigation/batch_request').value
        self.launch_package = self.declare_parameter('launch_package', 'P7_launch').value
        self.launch_file = self.declare_parameter('launch_file', 'complete.launch.py').value
        self.topdown_prefix = self.declare_parameter('topdown_map_prefix', '/topdown_map').value
        self.heatmap_resolution = float(self.declare_parameter('heatmap_resolution', 0.1).value)
        # Gaussian spread of every trajectory sample in the heatmap [m].
        self.heatmap_sigma = float(self.declare_parameter('heatmap_sigma', 0.5).value)
        # Minimum time between stored trajectory samples (seconds, odometry stamp time).
        self.sample_period = float(self.declare_parameter('sample_period', 0.1).value)
        # A gap larger than this between odometry samples is not counted as time spent there.
        self.max_sample_gap = float(self.declare_parameter('max_sample_gap', 1.0).value)
        # A jump larger than this between samples is not counted as flown distance (relocalisation).
        self.max_jump = float(self.declare_parameter('max_position_jump', 5.0).value)

        self.lock = threading.Lock()
        self.reset_trajectory()
        self.state = None
        self.state_received = None  # time.monotonic() of the last state message
        self.state_history = []  # (mission time or None, wall clock string, state)
        self.goals_reached = 0
        self.replans = 0
        self.batch_ids = set()
        self.viewpoints = np.empty((0, 2))
        self.messages = []  # service results etc. for the GUI log
        self.record_only_during_mission = False
        self.topdown_image = None
        self.topdown_grid = None
        self.topdown_trees = []
        self.topdown_received = None
        self.coverage = None  # rows: x, y, seconds in camera view
        self.coverage_frame = ''
        self.coverage_received = None

        self.mission_start = None  # rclpy Time
        self.mission_end = None
        self.mission_start_pending = False  # waiting for /clock to start ticking
        self.wall_start = None
        self.wall_end = None

        self.create_subscription(String, self.state_topic, self._state_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(UInt32, self.batch_topic, self._batch_cb, latched)
        if OptimizedFrontierArray is not None:
            self.create_subscription(OptimizedFrontierArray, self.viewpoints_topic, self._viewpoints_cb, latched)
        self.create_subscription(Image, self.topdown_prefix + '/image', self._topdown_image_cb, latched)
        self.create_subscription(OccupancyGrid, self.topdown_prefix + '/grid', self._topdown_grid_cb, latched)
        self.create_subscription(MarkerArray, self.topdown_prefix + '/trees', self._topdown_trees_cb, latched)
        self.create_subscription(PointCloud2, self.topdown_prefix + '/coverage', self._coverage_cb, latched)
        self.reset_coverage_client = self.create_client(Trigger, self.topdown_prefix + '/reset_coverage')
        self.start_client = self.create_client(Trigger, self.start_service)
        self.finish_client = self.create_client(Trigger, self.finish_service)
        self.create_timer(0.2, self._mission_tick)

    # ------------------------------------------------------------------ helpers
    def log(self, text):
        with self.lock:
            self.messages.append(f'[{time.strftime("%H:%M:%S")}] {text}')
        self.get_logger().info(text)

    def take_messages(self):
        with self.lock:
            messages, self.messages = self.messages, []
        return messages

    def use_sim_time(self):
        return bool(self.get_parameter('use_sim_time').value)

    def set_use_sim_time(self, enabled):
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, bool(enabled))])

    def clock_running(self):
        return self.get_clock().now().nanoseconds > 0

    def reset_trajectory(self):
        with getattr(self, 'lock', threading.Lock()):
            self.samples = []  # rows: x, y, z, weight [s], stamp [s]
            self.last_sample_stamp = None
            self.last_position = None
            self.distance_3d = 0.0
            self.distance_2d = 0.0
            self.max_altitude = -math.inf
            self.pose = None  # (x, y, z, yaw)
            self.speed = 0.0
            self.odom_frame = ''
            self.odom_received = None

    # ------------------------------------------------------------------ mission
    def mission_running(self):
        return (self.mission_start is not None or self.mission_start_pending) and self.mission_end is None

    def start_mission_timer(self):
        with self.lock:
            self.mission_end = None
            self.wall_end = None
            self.state_history = []
            self.goals_reached = 0
            self.replans = 0
            if self.clock_running():
                self.mission_start = self.get_clock().now()
                self.wall_start = time.monotonic()
                self.mission_start_pending = False
            else:
                self.mission_start = None
                self.mission_start_pending = True

    def stop_mission_timer(self, reason):
        with self.lock:
            if not self.mission_running():
                return
            if self.mission_start is None:  # never got /clock
                self.mission_start_pending = False
                return
            self.mission_end = self.get_clock().now()
            self.wall_end = time.monotonic()
            elapsed = (self.mission_end - self.mission_start).nanoseconds * 1e-9
        self.log(f'Mission timer stopped ({reason}) at {elapsed:.1f} s.')

    def mission_elapsed(self):
        """(mission seconds, wall seconds) or (None, None) before the start."""
        if self.mission_start is None:
            return None, None
        end = self.mission_end if self.mission_end is not None else self.get_clock().now()
        wall_end = self.wall_end if self.wall_end is not None else time.monotonic()
        return max(0.0, (end - self.mission_start).nanoseconds * 1e-9), wall_end - self.wall_start

    def _mission_tick(self):
        if self.mission_start_pending and self.clock_running():
            with self.lock:
                self.mission_start = self.get_clock().now()
                self.wall_start = time.monotonic()
                self.mission_start_pending = False
            self.log('/clock received; mission timer started.')

    def call_start(self):
        self._call(self.start_client, self.start_service, 'Start', start_timer=True)

    def call_finish(self):
        self._call(self.finish_client, self.finish_service, 'Finish', start_timer=False)

    def call_reset_coverage(self):
        self._call(self.reset_coverage_client, self.topdown_prefix + '/reset_coverage', 'Reset coverage',
                   start_timer=False)

    def _call(self, client, name, label, start_timer):
        if not client.service_is_ready():
            self.log(f'{label}: service {name} is not available (is frontier navigation running?).')
            return
        future = client.call_async(Trigger.Request())

        def done(fut):
            try:
                response = fut.result()
            except Exception as error:  # noqa: BLE001
                self.log(f'{label}: service call failed: {error}')
                return
            self.log(f'{label}: success={response.success} "{response.message}"')
            if start_timer and response.success and not self.mission_running() \
                    and 'already' not in response.message.lower():
                self.start_mission_timer()
                self.log('Mission timer started (%s time).' % ('sim' if self.use_sim_time() else 'wall'))

        future.add_done_callback(done)

    # ------------------------------------------------------------------ callbacks
    def _state_cb(self, msg):
        new_state = msg.data
        with self.lock:
            previous = self.state
            self.state = new_state
            self.state_received = time.monotonic()
        if new_state == previous:
            return
        # Exploration started outside the GUI (e.g. ros2 service call): start the timer too.
        if previous == 'WAITING' and new_state not in ('WAITING', 'FINISHED') and not self.mission_running():
            self.start_mission_timer()
            self.log('Exploration left WAITING; mission timer started.')
        mission_time, _ = self.mission_elapsed()
        with self.lock:
            self.state_history.append((mission_time, time.strftime('%H:%M:%S'), new_state))
            if new_state == 'GOAL_REACHED':
                self.goals_reached += 1
            elif new_state == 'REPLAN':
                self.replans += 1
        if new_state == 'FINISHED':
            self.stop_mission_timer('frontier navigation FINISHED')

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
            return
        stamp = _stamp_seconds(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        v = msg.twist.twist.linear
        with self.lock:
            self.pose = (p.x, p.y, p.z, _yaw(msg.pose.pose.orientation))
            self.speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            self.odom_frame = msg.header.frame_id
            self.odom_received = time.monotonic()
            if self.record_only_during_mission and not self.mission_running():
                self.last_sample_stamp = None
                self.last_position = None
                return
            if self.last_sample_stamp is not None:
                dt = stamp - self.last_sample_stamp
                if 0.0 <= dt < self.sample_period:
                    return
                if dt < 0.0 or dt > self.max_sample_gap:  # time jump or gap: do not count it
                    dt = self.sample_period
            else:
                dt = self.sample_period
            position = np.array([p.x, p.y, p.z])
            if self.last_position is not None:
                step = position - self.last_position
                if np.linalg.norm(step) <= self.max_jump:
                    self.distance_3d += float(np.linalg.norm(step))
                    self.distance_2d += float(np.linalg.norm(step[:2]))
            self.last_position = position
            self.last_sample_stamp = stamp
            self.max_altitude = max(self.max_altitude, p.z)
            self.samples.append((p.x, p.y, p.z, dt, stamp))

    def _batch_cb(self, msg):
        with self.lock:
            self.batch_ids.add(msg.data)

    def _viewpoints_cb(self, msg):
        points = [(v.viewpoint.pose.position.x, v.viewpoint.pose.position.y) for v in msg.viewpoints]
        with self.lock:
            self.viewpoints = np.array(points) if points else np.empty((0, 2))

    def _topdown_image_cb(self, msg):
        if msg.encoding != 'rgb8':
            return
        image = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, :3 * msg.width]
        with self.lock:
            self.topdown_image = image.reshape(msg.height, msg.width, 3).copy()
            self.topdown_received = time.monotonic()

    def _topdown_grid_cb(self, msg):
        classes = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        with self.lock:
            self.topdown_grid = {
                'classes': classes,
                'origin': (msg.info.origin.position.x, msg.info.origin.position.y),
                'resolution': float(msg.info.resolution),
                'frame': msg.header.frame_id,
            }

    def _topdown_trees_cb(self, msg):
        trees = [(m.pose.position.x, m.pose.position.y, 0.5 * m.scale.x)
                 for m in msg.markers if m.action == Marker.ADD]
        with self.lock:
            self.topdown_trees = trees

    def _coverage_cb(self, msg):
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'intensity'), skip_nans=True)
        with self.lock:
            self.coverage = np.asarray(points, dtype=np.float64).reshape(-1, 3)
            self.coverage_frame = msg.header.frame_id
            self.coverage_received = time.monotonic()

    def topdown_map(self):
        """Latest image + grid of the same size, or None."""
        with self.lock:
            image, grid = self.topdown_image, self.topdown_grid
            if image is None or grid is None or image.shape[:2] != grid['classes'].shape:
                return None
            return dict(grid, image=image, trees=list(self.topdown_trees), received=self.topdown_received)

    def trajectory(self):
        with self.lock:
            return np.array(self.samples) if self.samples else np.empty((0, 5))
