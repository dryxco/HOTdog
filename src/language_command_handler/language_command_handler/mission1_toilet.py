#!/usr/bin/env python3
"""
ROS2 mission node: mission1_toilet
- Publish goal_pose once
- Monitor robot pose via /go1_pose (map frame)
- Bark when goal is reached
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math


class Mission1Toilet(Node):
    def __init__(self):
        super().__init__('mission1_toilet')

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/go1_pose',
            self.pose_callback,
            10
        )

        # -----------------------------
        # Publishers
        # -----------------------------
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.speech_pub = self.create_publisher(
            String,
            '/robot_dog/speech',
            10
        )

        # -----------------------------
        # Mission parameters (map frame)
        # -----------------------------
        self.goal_x = 10.0
        self.goal_y = 5.0

        self.arrival_threshold = 0.5  # meters

        # -----------------------------
        # Internal state
        # -----------------------------
        self.robot_x = None
        self.robot_y = None

        self.goal_sent = False
        self.mission_done = False

        self.state = "SEND_GOAL"

        self.get_logger().info("🚽 Mission1Toilet node initialized")

        self.timer = self.create_timer(0.2, self.control_loop)

    # -----------------------------
    # Callbacks
    # -----------------------------
    def pose_callback(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

    # -----------------------------
    # Helpers
    # -----------------------------
    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"📍 Goal published → ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def distance_to_goal(self):
        if self.robot_x is None:
            return None
        return math.hypot(
            self.goal_x - self.robot_x,
            self.goal_y - self.robot_y
        )

    # -----------------------------
    # FSM
    # -----------------------------
    def control_loop(self):

        if self.state == "SEND_GOAL":
            self.publish_goal()
            self.goal_sent = True
            self.state = "WAIT_ARRIVAL"

        elif self.state == "WAIT_ARRIVAL":
            dist = self.distance_to_goal()
            if dist is None:
                return

            self.get_logger().info(f"⏳ Distance to goal: {dist:.2f} m")

            if dist < self.arrival_threshold:
                self.get_logger().info("✅ Goal reached")
                self.state = "BARK"

        elif self.state == "BARK":
            msg = String()
            msg.data = "bark"
            self.speech_pub.publish(msg)

            self.get_logger().info("🐶 BARK! Mission complete")
            self.state = "DONE"

        elif self.state == "DONE":
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Mission1Toilet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()