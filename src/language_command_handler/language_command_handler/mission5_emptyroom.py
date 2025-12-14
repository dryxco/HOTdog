#!/usr/bin/env python3
"""
Mission 5: Navigate to Empty Room (Corrected Frames; uses Localization /go1_pose)

Goal:
  1) Go to Room 1 front -> Scan for Stop Sign
  2) Go to Room 2 front -> Scan for Stop Sign
  3) Enter the room WITHOUT Stop Sign

Key idea:
- /goal_pose: map frame goal (absolute)
- /go1_pose: localization-provided robot pose in map frame (subscribe only)
- Do NOT publish /go1_pose from this node.
"""

import math
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool

# ============================================================
# [TODO] Replace with actual map coordinates
# ============================================================
ROOM1_FRONT  = [1.0, 0.0, 0.0]
ROOM2_FRONT  = [1.0, 2.0, 0.0]
ROOM1_INSIDE = [2.0, 0.0, 0.0]
ROOM2_INSIDE = [2.0, 2.0, 0.0]

STOP_KEYWORDS = ["stop_sign", "stop sign", "stopsign", "stop-sign"]

class Mission5EmptyRoom(Node):
    def __init__(self):
        super().__init__("mission5_emptyroom")

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.emergency_pub = self.create_publisher(Bool, "/emergency", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.speech_pub = self.create_publisher(String, "/robot_dog/speech", 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, "/go1_pose", self.pose_callback, 10)
        self.label_sub = self.create_subscription(String, "/detections/labels", self.label_callback, 10)

        # Internal State (map frame)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_pose = False
        self.detected_label = ""

        self.room1_has_stop: Optional[bool] = None
        self.room2_has_stop: Optional[bool] = None

        self.state = "WAIT_POSE"
        self.state_enter_time = time.time()
        self.last_goal_pub_time = 0.0

        self.xy_tolerance = 0.4

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Mission 5 (Map Frame Version) initialized.")

    # -------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------
    def pose_callback(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.have_pose = True

    def label_callback(self, msg: String):
        self.detected_label = (msg.data or "").strip().lower()

    # -------------------------------------------------------------
    # Helper Methods
    # -------------------------------------------------------------
    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.robot_x, y - self.robot_y)

    def set_tracker_active(self, active: bool):
        msg = Bool()
        msg.data = not active  # /emergency True => stop
        self.emergency_pub.publish(msg)
        if not active:
            self.cmd_pub.publish(Twist())

    def publish_goal(self, coords: List[float]):
        now = time.time()
        if now - self.last_goal_pub_time > 0.5:
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(coords[0])
            goal.pose.position.y = float(coords[1])
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.last_goal_pub_time = now

    def set_state(self, s: str):
        self.state = s
        self.state_enter_time = time.time()
        self.get_logger().info(f"State changed to: {s}")

    def has_stop_sign(self) -> bool:
        txt = self.detected_label or ""
        return any(k in txt for k in STOP_KEYWORDS)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # -------------------------------------------------------------
    # Main Control Loop
    # -------------------------------------------------------------
    def control_loop(self):
        if self.state == "WAIT_POSE":
            if self.have_pose:
                self.get_logger().info("Localization pose received. Starting Mission 5.")
                self.set_state("NAV_R1")
            return

        if self.state == "NAV_R1":
            self.set_tracker_active(True)
            self.publish_goal(ROOM1_FRONT)
            if self.distance_to(ROOM1_FRONT[0], ROOM1_FRONT[1]) < self.xy_tolerance:
                self.set_tracker_active(False)
                self.stop_robot()
                self.set_state("SCAN_R1")
            return

        if self.state == "SCAN_R1":
            self.set_tracker_active(False)
            if time.time() - self.state_enter_time < 2.5:
                tw = Twist()
                tw.angular.z = 0.6
                self.cmd_pub.publish(tw)
                if self.has_stop_sign():
                    self.room1_has_stop = True
            else:
                self.stop_robot()
                if self.room1_has_stop is None:
                    self.room1_has_stop = False
                self.get_logger().info(f"Room 1 Check Result: Stop Sign = {self.room1_has_stop}")
                self.set_state("NAV_R2")
            return

        if self.state == "NAV_R2":
            self.set_tracker_active(True)
            self.publish_goal(ROOM2_FRONT)
            if self.distance_to(ROOM2_FRONT[0], ROOM2_FRONT[1]) < self.xy_tolerance:
                self.set_tracker_active(False)
                self.stop_robot()
                self.set_state("SCAN_R2")
            return

        if self.state == "SCAN_R2":
            self.set_tracker_active(False)
            if time.time() - self.state_enter_time < 2.5:
                tw = Twist()
                tw.angular.z = 0.6
                self.cmd_pub.publish(tw)
                if self.has_stop_sign():
                    self.room2_has_stop = True
            else:
                self.stop_robot()
                if self.room2_has_stop is None:
                    self.room2_has_stop = False
                self.get_logger().info(f"Room 2 Check Result: Stop Sign = {self.room2_has_stop}")
                self.set_state("ENTER")
            return

        if self.state == "ENTER":
            target = ROOM1_INSIDE
            if self.room1_has_stop and not self.room2_has_stop:
                target = ROOM2_INSIDE
                self.get_logger().info("Decision: Enter Room 2")
            elif not self.room1_has_stop:
                target = ROOM1_INSIDE
                self.get_logger().info("Decision: Enter Room 1")
            else:
                target = ROOM1_INSIDE
                self.get_logger().info("Decision: Default to Room 1")

            self.set_tracker_active(True)
            self.publish_goal(target)

            if self.distance_to(target[0], target[1]) < self.xy_tolerance:
                self.set_tracker_active(False)
                self.stop_robot()
                self.get_logger().info("SUCCESS: Robot entered the empty room.")
                self.set_state("DONE")
            return

        if self.state == "DONE":
            self.set_tracker_active(False)
            self.stop_robot()
            return

def main(args=None):
    rclpy.init(args=args)
    node = Mission5EmptyRoom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Bool()
        stop_msg.data = True
        node.emergency_pub.publish(stop_msg)
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
