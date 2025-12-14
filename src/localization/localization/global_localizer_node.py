#!/usr/bin/env python3

"""
ROS2 global localizer node (PF version)

This node performs **particle-filter-based global localization** in the map frame.

- It subscribes to:
    - /map  (nav_msgs/OccupancyGrid)
    - /scan (sensor_msgs/LaserScan)
    - /clock (rosgraph_msgs/Clock)  [for simulation time]
    - TF: odom -> base  (from odom_localizer_node)

- It maintains a set of particles in the map frame: (x, y, theta).

- Motion model:
    - Uses odom->base TF difference between timesteps (Laser odometry from odom_localizer)
    - Applies base_prev->base_curr motion to each particle in its own local frame + noise.

- Sensor model:
    - Uses /scan -> 2D point cloud (scan_to_pcd)
    - For each particle, transforms points into map frame and scores them against occupancy grid.

- Output:
    - /go1_pose (geometry_msgs/PoseStamped), frame_id = "map"
    - TF: map -> odom
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock

from tf2_ros import (
    TransformBroadcaster,
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
import tf_transformations

from utils import pose_to_matrix, transform_to_matrix, scan_to_pcd


class GlobalLocalizerNode(Node):
    def __init__(self):
        super().__init__('global_localizer')
        self.get_logger().info('Global localizer node (PF) initialized')

        # ─────────────────────────────────────
        # Parameters
        # ─────────────────────────────────────
        # initial pose (map frame), provided by launch file
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 1.0)
        self.declare_parameter('yaw', 0.0)

        self.init_x = float(self.get_parameter('x').value)
        self.init_y = float(self.get_parameter('y').value)
        self.init_yaw = float(self.get_parameter('yaw').value)

        # robot base height (z)
        self.base_z = 0.33  # example height from original code

        # PF parameters
        self.declare_parameter('num_particles', 300)
        self.declare_parameter('resample_ratio', 0.5)
        self.declare_parameter('scan_subsample', 4)

        self.num_particles = int(self.get_parameter('num_particles').value)
        self.resample_ratio = float(self.get_parameter('resample_ratio').value)
        self.scan_subsample = int(self.get_parameter('scan_subsample').value)

        # motion noise (will be scaled by sqrt(dt))
        self.trans_noise = 0.02  # m
        self.rot_noise = 0.01    # rad

        # sensor model scores
        self.occ_score = 2.0
        self.free_score = -1.0
        self.unknown_score = 0.0

        # ─────────────────────────────────────
        # Map data
        # ─────────────────────────────────────
        self.map: OccupancyGrid | None = None
        self.map_data = None  # 2D numpy array (H, W)
        self.map_res = None
        self.map_width = None
        self.map_height = None
        self.map_origin_x = None
        self.map_origin_y = None

        # ─────────────────────────────────────
        # Particle filter state
        # ─────────────────────────────────────
        self.particles = None   # shape (N, 3): [x, y, theta]
        self.weights = None     # shape (N,)
        self.initialized = False

        # motion model uses odom->base TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_T_odom_to_base_se2 = None  # 3x3 SE2 matrix
        self.last_scan_time: Time | None = None

        # result transform
        self.T_map_to_odom = None  # 4x4 matrix

        # ─────────────────────────────────────
        # Time / TF broadcaster / publishers
        # ─────────────────────────────────────
        self.current_time: Time | None = None
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_interval = 0.05  # 20 Hz
        self.tf_timer = self.create_timer(self.tf_interval, self.tf_timer_callback)

        self.pose_pub = self.create_publisher(PoseStamped, '/go1_pose', 10)

        # ─────────────────────────────────────
        # Subscribers
        # ─────────────────────────────────────
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info(
            f'PF params: N={self.num_particles}, resample_ratio={self.resample_ratio}, '
            f'scan_subsample={self.scan_subsample}'
        )

    # ─────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────
    def clock_callback(self, msg: Clock):
        self.current_time = Time.from_msg(msg.clock)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
        self.map_res = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape(self.map_height, self.map_width)
        self.map_data = data

        self.get_logger().info(
            f'Map received: {self.map_width}x{self.map_height}, res={self.map_res}'
        )

        # 맵이 들어온 뒤 파티클 초기화
        self.initialize_particles()

    def initialize_particles(self):
        if self.map_data is None:
            return

        N = self.num_particles
        self.particles = np.zeros((N, 3), dtype=np.float32)
        # 초기 pose 주변에 Gaussian
        self.particles[:, 0] = np.random.normal(self.init_x, 0.2, size=N)
        self.particles[:, 1] = np.random.normal(self.init_y, 0.2, size=N)
        self.particles[:, 2] = np.random.normal(self.init_yaw, 0.1, size=N)

        self.weights = np.ones(N, dtype=np.float32) / N
        self.initialized = True

        self.get_logger().info(
            f'Particles initialized around ({self.init_x:.2f}, {self.init_y:.2f}, yaw={self.init_yaw:.2f})'
        )

    def scan_callback(self, scan: LaserScan):
        """
        Main PF update loop triggered by LaserScan.
        1) Get odom->base transform and compute base_prev->base_curr motion.
        2) Motion update for particles.
        3) Sensor update with scan + map.
        4) Resampling.
        5) Pose estimation (map frame).
        6) Update T_map_to_odom and publish /go1_pose.
        """
        if not self.initialized or self.map_data is None:
            return

        # 1) 현재 odom->base TF 가져오기
        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                'odom', 'base', rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform from odom to base: {e}')
            return

        # 4x4 행렬로 변환 후 2D SE2로 투영
        T_odom_to_base_4 = transform_to_matrix(tf_odom_base.transform)
        T_odom_to_base = self.se3_to_se2(T_odom_to_base_4)

        # 첫 스캔이면 prev만 저장
        if self.prev_T_odom_to_base_se2 is None:
            self.prev_T_odom_to_base_se2 = T_odom_to_base
            self.last_scan_time = self.get_clock().now()
            return

        # ΔT_base = base_prev -> base_curr
        T_base_prev_to_base_curr = np.linalg.inv(self.prev_T_odom_to_base_se2) @ T_odom_to_base
        self.prev_T_odom_to_base_se2 = T_odom_to_base

        # dt (정보용)
        now = self.get_clock().now()
        if self.last_scan_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_scan_time).nanoseconds / 1e9
        self.last_scan_time = now

        # 2) Motion update (odom 기반)
        self.motion_update(T_base_prev_to_base_curr, dt)

        # 3) Sensor update (scan + map)
        self.sensor_update(scan)

        # 4) Resample
        self.resample_if_needed()

        # 5) Pose estimate
        est_x, est_y, est_theta = self.estimate_pose()

        # 6) map->odom 계산 & /go1_pose publish
        #    T_map_to_base_est, T_odom_to_base_4 를 이용
        self.update_map_to_odom_and_publish(est_x, est_y, est_theta, T_odom_to_base_4)

    # ─────────────────────────────────────────
    # Motion update
    # ─────────────────────────────────────────
    def motion_update(self, T_base_prev_to_base_curr: np.ndarray, dt: float):
        """
        Apply base_prev->base_curr motion (in previous base frame) to all particles in map frame.
        """
        if self.particles is None:
            return

        # base_prev->base_curr를 (Δx_r, Δy_r, Δθ)로 분해 (로봇 local frame)
        dx_r = float(T_base_prev_to_base_curr[0, 2])
        dy_r = float(T_base_prev_to_base_curr[1, 2])
        dtheta = math.atan2(T_base_prev_to_base_curr[1, 0], T_base_prev_to_base_curr[0, 0])

        # 노이즈 스케일
        trans_noise = self.trans_noise * math.sqrt(max(dt, 1e-3))
        rot_noise = self.rot_noise * math.sqrt(max(dt, 1e-3))

        # 각 파티클에 대해 적용
        x = self.particles[:, 0]
        y = self.particles[:, 1]
        theta = self.particles[:, 2]

        # 파티클의 heading 기준으로 Δx_r, Δy_r를 map frame으로 변환
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)

        # base local motion (dx_r, dy_r)를 map frame으로 투영
        # [dx_global, dy_global] = R(theta) * [dx_r, dy_r]
        dx_global = dx_r * cos_t - dy_r * sin_t
        dy_global = dx_r * sin_t + dy_r * cos_t

        # 노이즈 추가
        dx_noise = np.random.normal(0.0, trans_noise, size=len(x))
        dy_noise = np.random.normal(0.0, trans_noise, size=len(x))
        dtheta_noise = np.random.normal(0.0, rot_noise, size=len(x))

        self.particles[:, 0] = x + dx_global + dx_noise
        self.particles[:, 1] = y + dy_global + dy_noise
        self.particles[:, 2] = self.normalize_angle(theta + dtheta + dtheta_noise)

    # ─────────────────────────────────────────
    # Sensor update (scan + map)
    # ─────────────────────────────────────────
    def sensor_update(self, scan: LaserScan):
        if self.particles is None or self.map_data is None:
            return

        # LaserScan -> 2D pcd in base frame
        pcd = scan_to_pcd(scan)  # shape (M, 2) in robot(base) frame

        if pcd.shape[0] == 0:
            return

        # subsample
        if self.scan_subsample > 1:
            pcd = pcd[::self.scan_subsample]

        scores = np.zeros(self.particles.shape[0], dtype=np.float32)

        for i, (px, py, pth) in enumerate(self.particles):
            score = 0.0
            cos_t = math.cos(pth)
            sin_t = math.sin(pth)

            # base frame pcd -> map frame
            # [x_m] = px + cos_t * x_b - sin_t * y_b
            # [y_m] = py + sin_t * x_b + cos_t * y_b
            x_b = pcd[:, 0]
            y_b = pcd[:, 1]

            x_m = px + cos_t * x_b - sin_t * y_b
            y_m = py + sin_t * x_b + cos_t * y_b

            # map index
            mx = ((x_m - self.map_origin_x) / self.map_res).astype(int)
            my = ((y_m - self.map_origin_y) / self.map_res).astype(int)

            valid = (mx >= 0) & (mx < self.map_width) & (my >= 0) & (my < self.map_height)
            mx = mx[valid]
            my = my[valid]

            if mx.size == 0:
                scores[i] = -1e3  # 매우 나쁜 점수
                continue

            occ_vals = self.map_data[my, mx]

            # 점수 계산
            score += np.sum(
                np.where(
                    occ_vals == -1,
                    self.unknown_score,
                    np.where(occ_vals >= 50, self.occ_score, self.free_score),
                )
            )

            scores[i] = score

        # 점수 -> weight (softmax)
        max_score = np.max(scores)
        weights = np.exp(scores - max_score)
        sum_w = np.sum(weights) + 1e-9
        self.weights = weights / sum_w

    # ─────────────────────────────────────────
    # Resampling
    # ─────────────────────────────────────────
    def resample_if_needed(self):
        if self.particles is None or self.weights is None:
            return

        N = self.particles.shape[0]
        neff = 1.0 / np.sum(self.weights ** 2)

        if neff > self.resample_ratio * N:
            return

        # systematic resampling
        positions = (np.arange(N) + np.random.uniform(0, 1)) / N
        indexes = np.zeros(N, dtype=np.int32)
        cumsum = np.cumsum(self.weights)
        i, j = 0, 0
        while i < N:
            if positions[i] < cumsum[j]:
                indexes[i] = j
                i += 1
            else:
                j += 1

        self.particles = self.particles[indexes]
        self.weights = np.ones(N, dtype=np.float32) / N

    # ─────────────────────────────────────────
    # Pose estimate & map->odom + /go1_pose
    # ─────────────────────────────────────────
    def estimate_pose(self):
        x = np.average(self.particles[:, 0], weights=self.weights)
        y = np.average(self.particles[:, 1], weights=self.weights)

        sin_t = np.average(np.sin(self.particles[:, 2]), weights=self.weights)
        cos_t = np.average(np.cos(self.particles[:, 2]), weights=self.weights)
        theta = math.atan2(sin_t, cos_t)

        return x, y, theta

    def update_map_to_odom_and_publish(self, x, y, theta, T_odom_to_base_4):
        """
        1) Build T_map_to_base_est from PF pose.
        2) Compute T_map_to_odom = T_map_to_base_est * inv(T_odom_to_base).
        3) Store T_map_to_odom (used by tf_timer).
        4) Publish /go1_pose (map frame).
        """
        # 1) T_map_to_base_est (4x4)
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, theta)
        T_map_to_base = pose_to_matrix(
            [x, y, self.base_z, q[0], q[1], q[2], q[3]]
        )

        # 2) T_map_to_odom
        T_base_to_odom = np.linalg.inv(T_odom_to_base_4)
        self.T_map_to_odom = T_map_to_base @ T_base_to_odom

        # 3) /go1_pose publish
        pose_msg = PoseStamped()
        # 시간: /clock가 있으면 그걸 사용, 없으면 now
        if self.current_time is not None:
            pose_msg.header.stamp = self.current_time.to_msg()
        else:
            pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(self.base_z)
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        self.pose_pub.publish(pose_msg)

    # ─────────────────────────────────────────
    # TF timer: map -> odom publish
    # ─────────────────────────────────────────
    def tf_timer_callback(self):
        if self.T_map_to_odom is None or self.current_time is None:
            return

        translation = self.T_map_to_odom[:3, 3]
        quaternion = tf_transformations.quaternion_from_matrix(self.T_map_to_odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.current_time.to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(quaternion[0])
        tf_msg.transform.rotation.y = float(quaternion[1])
        tf_msg.transform.rotation.z = float(quaternion[2])
        tf_msg.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(tf_msg)

    # ─────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────
    @staticmethod
    def se3_to_se2(T4: np.ndarray) -> np.ndarray:
        """
        4x4 SE(3) -> 3x3 SE(2) (x,y,yaw만)
        """
        x = T4[0, 3]
        y = T4[1, 3]
        yaw = math.atan2(T4[1, 0], T4[0, 0])
        c = math.cos(yaw)
        s = math.sin(yaw)
        T2 = np.array(
            [
                [c, -s, x],
                [s,  c, y],
                [0,  0, 1],
            ],
            dtype=np.float32,
        )
        return T2

    @staticmethod
    def normalize_angle(a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
