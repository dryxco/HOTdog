#!/usr/bin/env python3
"""
Mission 3: Move to the red cone and bark (Corrected Frames)

Key idea:
- /goal_pose is expressed in the map frame (absolute goal on the map).
- /go1_pose is provided by the localization module in the map frame (corrected robot pose).
- This mission node must NOT republish /go1_pose (avoid frame mismatch and topic conflicts).

Notes:
- CONE_SITES must be set to the 3 *exact* scan waypoints in map frame.
"""

import math
import time
from typing import Dict, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool

# -----------------------------
# Mission parameters
# -----------------------------
XY_TOL = 0.35            # meters
OBSERVE_SECONDS = 1.2    # seconds

# ============================================================
# [TODO] Replace the placeholders with the 3 exact scan points (map frame)
# Format: [x, y, yaw]
# ============================================================
CONE_SITES = [
    [0.0, 0.0, 0.0],  # TODO: replace
    [0.0, 0.0, 0.0],  # TODO: replace
    [0.0, 0.0, 0.0],  # TODO: replace
]
SCAN_ORDER = [0, 1, 2]

COLOR_KEYWORDS: Dict[str, List[str]] = {
    "red": ["red", "red_cone", "red cone", "cone_red", "red-cone"],
}

class Mission3RedCone(Node):
    def __init__(self):
        super().__init__("mission3_red_cone")

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.emergency_pub = self.create_publisher(Bool, "/emergency", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)  # safety stop
        self.speech_pub = self.create_publisher(String, "/robot_dog/speech", 10)

        # Subscribers (Localization pose is the source of truth)
        self.pose_sub = self.create_subscription(PoseStamped, "/go1_pose", self._pose_cb, 10)
        self.label_sub = self.create_subscription(String, "/detections/labels", self._label_cb, 10)

        # Internal pose (map frame)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_pose = False

        self.detected_label = ""

        self.state = "WAIT_POSE"
        self.state_enter_time = time.time()
        self.last_goal_pub_time = 0.0

        self.site_cursor = 0
        self.current_goal = CONE_SITES[SCAN_ORDER[self.site_cursor]]

        self.timer = self.create_timer(0.1, self._loop)
        self.get_logger().info("Mission 3 (Map Frame Version) initialized.")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def _pose_cb(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.have_pose = True

    def _label_cb(self, msg: String):
        self.detected_label = (msg.data or "").strip().lower()

    # -----------------------------
    # Helpers
    # -----------------------------
    def _set_tracker_active(self, active: bool):
        # /emergency: True = stop, False = run
        b = Bool()
        b.data = (not active)
        self.emergency_pub.publish(b)

        # Safety: explicitly stop /cmd_vel when we disable tracking
        if not active:
            self.cmd_pub.publish(Twist())

    def _publish_goal_throttled(self, gx: float, gy: float):
        now = time.time()
        if now - self.last_goal_pub_time > 0.5:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = float(gx)
            ps.pose.position.y = float(gy)
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            self.goal_pub.publish(ps)
            self.last_goal_pub_time = now

    def _distance_to(self, gx: float, gy: float) -> float:
        return math.hypot(gx - self.x, gy - self.y)

    def _label_has_red_cone(self) -> bool:
        txt = self.detected_label or ""
        return any(k in txt for k in COLOR_KEYWORDS["red"])

    def _set_state(self, s: str):
        self.state = s
        self.state_enter_time = time.time()
        self.get_logger().info(f"State changed to: {s}")

    def _bark(self, times: int = 3, interval: float = 0.5):
        msg = String()
        msg.data = "bark"
        for _ in range(times):
            self.speech_pub.publish(msg)
            time.sleep(interval)

    # -----------------------------
    # Main loop
    # -----------------------------
    def _loop(self):
        if self.state == "WAIT_POSE":
            if self.have_pose:
                self.get_logger().info("Localization pose received. Starting Mission 3.")
                self._set_state("NAV")
            return

        if self.state == "NAV":
            gx, gy, _ = self.current_goal
            self._set_tracker_active(True)
            self._publish_goal_throttled(gx, gy)

            if self._distance_to(gx, gy) < XY_TOL:
                # Stop tracker before observation
                self._set_tracker_active(False)
                self._set_state("OBSERVE")
            return

        if self.state == "OBSERVE":
            # Ensure tracker remains stopped
            self._set_tracker_active(False)

            if self._label_has_red_cone():
                self.get_logger().info("Red cone detected. Barking.")
                self._set_state("BARK")
                return

            if (time.time() - self.state_enter_time) > OBSERVE_SECONDS:
                # Move to next site
                self.site_cursor += 1
                if self.site_cursor >= len(SCAN_ORDER):
                    self.get_logger().warn("Red cone not found in any site. Finishing mission.")
                    self._set_state("DONE")
                    return

                self.current_goal = CONE_SITES[SCAN_ORDER[self.site_cursor]]
                self._set_state("NAV")
            return

        if self.state == "BARK":
            self._bark(times=3, interval=0.5)
            self._set_state("DONE")
            return

        if self.state == "DONE":
            self._set_tracker_active(False)
            return


def main(args=None):
    rclpy.init(args=args)
    node = Mission3RedCone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure tracker stop on shutdown
        stop_msg = Bool()
        stop_msg.data = True
        node.emergency_pub.publish(stop_msg)
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
