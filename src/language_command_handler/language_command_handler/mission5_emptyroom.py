#!/usr/bin/env python3
"""
ROS2 node: navigate_to_empty_room (Mission 5)

Goal:
- Move to the empty room WITHOUT a stop sign.
- There are 2 candidate rooms. Stop sign location can change:
  - in front of room1 or room2
  - distance from entrance can vary
- Strategy:
  1) Go to ROOM1_FRONT, scan for stop_sign by yaw-sweeping and reading /detections/labels.
  2) Go to ROOM2_FRONT, scan similarly.
  3) Enter the room whose entrance does NOT have stop_sign by navigating to its INSIDE pose.
  4) Success condition: robot body inside -> we stop at INSIDE pose (distance threshold).

Topics (aligned with sample navigate_to_toilet):
- Sub: /odom (nav_msgs/Odometry)
- Sub: /detections/labels (std_msgs/String)
- Pub: /cmd_vel (geometry_msgs/Twist)
- Pub (optional): /robot_dog/speech (std_msgs/String)

IMPORTANT:
- Fill the four poses below using absolute coordinates in your working frame (/odom for this controller).
"""

import math
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ============================================================
# [TODO] Fill these poses (x, y, yaw[radian]) in /odom (or consistent frame)
# ============================================================
ROOM1_FRONT  = [0.0, 0.0, 0.0]  # entrance viewpoint
ROOM2_FRONT  = [0.0, 0.0, 0.0]
ROOM1_INSIDE = [0.0, 0.0, 0.0]  # inside point (robot body in room)
ROOM2_INSIDE = [0.0, 0.0, 0.0]

STOP_KEYWORDS = ["stop_sign", "stop sign", "stopsign", "stop-sign"]


class NavigateToEmptyRoom(Node):
    def __init__(self):
        super().__init__("navigate_to_empty_room")

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.label_sub = self.create_subscription(String, "/detections/labels", self.label_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.speech_pub = self.create_publisher(String, "/robot_dog/speech", 10)

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False
        self.detected_label = ""

        self.room1_has_stop: Optional[bool] = None
        self.room2_has_stop: Optional[bool] = None

        self.state = "WAIT_ODOM"  # WAIT_ODOM -> NAV_R1 -> SCAN_R1 -> NAV_R2 -> SCAN_R2 -> ENTER -> DONE
        self.state_enter_time = time.time()

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("NavigateToEmptyRoom initialized (Mission 5).")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.have_odom = True

    def label_callback(self, msg: String):
        self.detected_label = (msg.data or "").strip().lower()

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.robot_x, y - self.robot_y)

    def angle_to(self, x: float, y: float) -> float:
        return math.atan2(y - self.robot_y, x - self.robot_x)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def set_state(self, s: str):
        self.state = s
        self.state_enter_time = time.time()

    def at_goal_xy(self, goal: List[float], tol: float = 0.35) -> bool:
        return self.distance_to(goal[0], goal[1]) < tol

    def has_stop_sign(self) -> bool:
        txt = self.detected_label or ""
        return any(k in txt for k in STOP_KEYWORDS)

    def goto_xy(self, goal: List[float]):
        xg, yg, _ = goal
        dist = self.distance_to(xg, yg)
        goal_angle = self.angle_to(xg, yg)
        angle_error = self.normalize_angle(goal_angle - self.robot_yaw)

        twist = Twist()
        if dist > 0.15:
            twist.linear.x = min(0.25, 0.6 * dist)
            twist.angular.z = 0.8 * angle_error
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def scan_by_yaw_sweep(self, duration: float = 2.5) -> bool:
        seen_stop = False
        start = time.time()
        while time.time() - start < duration and rclpy.ok():
            tw = Twist()
            tw.angular.z = 0.6
            self.cmd_pub.publish(tw)
            if self.has_stop_sign():
                seen_stop = True
            time.sleep(0.1)
        self.stop()
        return seen_stop

    def control_loop(self):
        if self.state == "WAIT_ODOM":
            if self.have_odom:
                self.get_logger().info("Odometry received. Start Mission 5.")
                self.set_state("NAV_R1")
            return

        if self.state == "NAV_R1":
            self.goto_xy(ROOM1_FRONT)
            if self.at_goal_xy(ROOM1_FRONT):
                self.stop()
                self.set_state("SCAN_R1")
            return

        if self.state == "SCAN_R1":
            if time.time() - self.state_enter_time < 0.3:
                return
            self.get_logger().info("[Scan] Room1 entrance...")
            self.room1_has_stop = self.scan_by_yaw_sweep(duration=2.5)
            self.get_logger().info(f"[Scan] Room1 stop_sign={self.room1_has_stop} labels='{self.detected_label}'")
            self.set_state("NAV_R2")
            return

        if self.state == "NAV_R2":
            self.goto_xy(ROOM2_FRONT)
            if self.at_goal_xy(ROOM2_FRONT):
                self.stop()
                self.set_state("SCAN_R2")
            return

        if self.state == "SCAN_R2":
            if time.time() - self.state_enter_time < 0.3:
                return
            self.get_logger().info("[Scan] Room2 entrance...")
            self.room2_has_stop = self.scan_by_yaw_sweep(duration=2.5)
            self.get_logger().info(f"[Scan] Room2 stop_sign={self.room2_has_stop} labels='{self.detected_label}'")
            self.set_state("ENTER")
            return

        if self.state == "ENTER":
            if self.room1_has_stop is None or self.room2_has_stop is None:
                self.get_logger().warn("Scan results missing; default to Room1.")
                target_inside = ROOM1_INSIDE
            else:
                if (not self.room1_has_stop) and self.room2_has_stop:
                    target_inside = ROOM1_INSIDE
                elif (not self.room2_has_stop) and self.room1_has_stop:
                    target_inside = ROOM2_INSIDE
                elif (not self.room1_has_stop) and (not self.room2_has_stop):
                    self.get_logger().warn("No stop sign detected at either entrance. Choosing Room1.")
                    target_inside = ROOM1_INSIDE
                else:
                    self.get_logger().warn("Stop sign detected at BOTH entrances (noise?). Choosing Room2.")
                    target_inside = ROOM2_INSIDE

            self.get_logger().info(f"[Enter] Navigating to inside pose: {target_inside}")
            self.goto_xy(target_inside)

            if self.at_goal_xy(target_inside, tol=0.40):
                self.stop()
                self.get_logger().info("SUCCESS: Robot inside the selected empty room.")
                self.set_state("DONE")
            return

        if self.state == "DONE":
            self.stop()
            return


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToEmptyRoom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
