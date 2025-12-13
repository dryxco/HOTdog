#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math


class Mission6Nurse(Node):
    def __init__(self):
        super().__init__('mission6_nurse')

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/go1_pose',
            self.pose_callback,
            10
        )

        self.label_sub = self.create_subscription(
            String,
            '/detections/labels',
            self.label_callback,
            10
        )
        self.distance_sub = self.create_subscription(
            Float32,
            '/detections/distance',
            self.distance_callback,
            10
        )
        self.center_sub = self.create_subscription(
            Float32,
            '/detections/bbox_center',
            self.center_callback,
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
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # -----------------------------
        # State
        # -----------------------------
        self.state = "SEND_GOAL"

        self.robot_x = None
        self.robot_y = None

        self.detected_label = None
        self.distance = None
        self.center_x = None

        # 목표 위치 (map frame)
        self.goal_x = 2.0
        self.goal_y = 1.0
        self.arrival_threshold = 0.5

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("👩‍⚕️ Mission6 Nurse node started")

    # -----------------------------
    # Callbacks
    # -----------------------------
    def pose_callback(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y

    def label_callback(self, msg: String):
        self.detected_label = msg.data

    def distance_callback(self, msg: Float32):
        self.distance = msg.data

    def center_callback(self, msg: Float32):
        self.center_x = msg.data

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
        self.get_logger().info("📍 Nurse mission goal published")

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

        # 0) 전역 이동
        if self.state == "SEND_GOAL":
            self.publish_goal()
            self.state = "WAIT_ARRIVAL"

        elif self.state == "WAIT_ARRIVAL":
            dist = self.distance_to_goal()
            if dist is None:
                return

            self.get_logger().info(f"⏳ Distance to nurse area: {dist:.2f}")

            if dist < self.arrival_threshold:
                self.get_logger().info("✅ Arrived at nurse area → SEARCH")
                self.state = "SEARCH"

        # 1) nurse 탐색
        elif self.state == "SEARCH":
            twist = Twist()
            twist.angular.z = 0.4
            self.cmd_pub.publish(twist)

            if self.detected_label == "nurse":
                self.get_logger().info("👀 Nurse detected → ALIGN")
                self.state = "ALIGN"

        # 2) bbox center 정렬
        elif self.state == "ALIGN":
            if self.detected_label != "nurse" or self.center_x is None:
                self.state = "SEARCH"
                return

            img_center_x = 320
            error = self.center_x - img_center_x

            twist = Twist()
            if abs(error) > 20:
                twist.angular.z = -0.002 * error
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.state = "APPROACH"

        # 3) 접근
        elif self.state == "APPROACH":
            if self.detected_label != "nurse" or self.distance is None:
                self.state = "SEARCH"
                return

            twist = Twist()
            if self.distance > 1.0:
                twist.linear.x = 0.25
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.state = "ROTATE_AROUND"

        # 4) 주변 회전
        elif self.state == "ROTATE_AROUND":
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 0.4
            self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Mission6Nurse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
