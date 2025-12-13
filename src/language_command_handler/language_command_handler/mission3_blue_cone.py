#!/usr/bin/env python3
"""
Mission 3: Move to the blue cone and bark.

Behavior (simple & deterministic):
1) Visit three fixed cone sites in order: site1 -> site2 -> site3
2) At each site, wait briefly and check perception labels for the target color
3) If the target color is detected at the current site, bark and stop (mission success)
4) If not detected, proceed to the next site; if none matched, stop (mission fail/timeout)

Topics (aligned with your existing missions):
- Sub: /odom (nav_msgs/Odometry)                 robot pose
- Sub: /detections/labels (std_msgs/String)      perception labels (must include color keywords)
- Pub: /cmd_vel (geometry_msgs/Twist)            low-level motion
- Pub: /robot_dog/speech (std_msgs/String)       "bark"

YOU MUST EDIT ONLY THE TODO COORDINATES BELOW.
"""

import math
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


TARGET_COLOR = "blue"  # fixed for this file

# Keywords used to detect target color in /detections/labels
COLOR_KEYWORDS = {
    "red":   ["red", "red_cone", "cone_red", "redcone", "cone(red)"],
    "green": ["green", "green_cone", "cone_green", "greencone", "cone(green)"],
    "blue":  ["blue", "blue_cone", "cone_blue", "bluecone", "cone(blue)"],
}

# ============================================================
# TODO: Fill the three fixed cone sites (absolute coordinates).
# - Use the same frame as /odom (or whatever your controller uses).
# - yaw is optional; the controller below only uses x,y for driving.
# ============================================================
CONE_SITES: Dict[str, List[float]] = {
    "site1": [0.0, 0.0, 0.0],  # TODO: [x1, y1, yaw1]
    "site2": [0.0, 0.0, 0.0],  # TODO: [x2, y2, yaw2]
    "site3": [0.0, 0.0, 0.0],  # TODO: [x3, y3, yaw3]
}
SCAN_ORDER = ["site1", "site2", "site3"]


class Mission3Cone(Node):
    def __init__(self):
        super().__init__(f"mission3_{TARGET_COLOR}_cone")

        # ROS I/O
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.speech_pub = self.create_publisher(String, "/robot_dog/speech", 10)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.labels_sub = self.create_subscription(String, "/detections/labels", self._labels_cb, 10)

        # Pose state
        self.have_odom = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Perception
        self.last_labels = ""

        # Mission state machine
        self.site_idx = 0
        self.state = "WAIT_ODOM"   # WAIT_ODOM -> NAV -> OBSERVE -> BARK -> DONE
        self.state_t0 = time.time()

        # Tunables (safe defaults)
        self.xy_tolerance = 0.35
        self.observe_seconds = 1.2
        self.max_linear = 0.25
        self.k_lin = 0.6
        self.k_ang = 0.8

        self.timer = self.create_timer(0.1, self._loop)  # 10 Hz

        self.get_logger().info(f"Mission3Cone started. TARGET_COLOR={TARGET_COLOR}")

    # -------------------------
    # Callbacks
    # -------------------------
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.have_odom = True

    def _labels_cb(self, msg: String):
        self.last_labels = (msg.data or "").strip().lower()

    # -------------------------
    # Helpers
    # -------------------------
    def _norm_angle(self, a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _dist_to(self, gx: float, gy: float) -> float:
        return math.hypot(gx - self.x, gy - self.y)

    def _angle_to(self, gx: float, gy: float) -> float:
        return math.atan2(gy - self.y, gx - self.x)

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _goto_xy(self, gx: float, gy: float):
        dist = self._dist_to(gx, gy)
        ang = self._angle_to(gx, gy)
        err = self._norm_angle(ang - self.yaw)

        cmd = Twist()
        if dist > 0.15:
            cmd.linear.x = min(self.max_linear, self.k_lin * dist)
            cmd.angular.z = self.k_ang * err
        self.cmd_pub.publish(cmd)

    def _arrived_xy(self, gx: float, gy: float) -> bool:
        return self._dist_to(gx, gy) < self.xy_tolerance

    def _target_seen(self) -> bool:
        kws = COLOR_KEYWORDS.get(TARGET_COLOR, [])
        txt = self.last_labels or ""
        return any(kw in txt for kw in kws)

    def _bark(self, times: int = 5, interval: float = 1.0):
        msg = String()
        msg.data = "bark"
        for i in range(times):
            self.speech_pub.publish(msg)
            self.get_logger().info(f"BARK ({i+1}/{times})")
            time.sleep(interval)

    def _set_state(self, s: str):
        self.state = s
        self.state_t0 = time.time()

    # -------------------------
    # Main loop
    # -------------------------
    def _loop(self):
        if self.state == "WAIT_ODOM":
            if self.have_odom:
                self.get_logger().info("Odometry received. Starting site scan.")
                self._set_state("NAV")
            return

        if self.site_idx >= len(SCAN_ORDER):
            self.get_logger().warn("Target cone NOT found at any site. Stopping.")
            self._stop()
            self._set_state("DONE")
            return

        site = SCAN_ORDER[self.site_idx]
        gx, gy, _ = CONE_SITES[site]

        if self.state == "NAV":
            self._goto_xy(gx, gy)
            if self._arrived_xy(gx, gy):
                self._stop()
                self.get_logger().info(f"Arrived at {site}. Observing labels...")
                self._set_state("OBSERVE")
            return

        if self.state == "OBSERVE":
            if self._target_seen():
                self.get_logger().info(f"FOUND {TARGET_COLOR} at {site} (labels='{self.last_labels}').")
                self._stop()
                self._set_state("BARK")
                return

            if time.time() - self.state_t0 >= self.observe_seconds:
                self.get_logger().info(f"{TARGET_COLOR} not seen at {site} (labels='{self.last_labels}'). Next site.")
                self.site_idx += 1
                self._set_state("NAV")
            return

        if self.state == "BARK":
            self._bark(times=5, interval=1.0)
            self._stop()
            self.get_logger().info("Mission3 success. Done.")
            self._set_state("DONE")
            return

        if self.state == "DONE":
            self._stop()
            return


def main(args=None):
    rclpy.init(args=args)
    node = Mission3Cone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
