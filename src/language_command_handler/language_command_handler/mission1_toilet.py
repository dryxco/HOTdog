#!/usr/bin/env python3
"""
ROS2 node: mission1_toilet
Goal: Move robot to pre-defined toilet location, align camera, and bark when in position.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import math

class Mission1Toilet(Node):
    def __init__(self):
        super().__init__('mission1_toilet')

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/detections/image', self.image_callback, 10)
        self.label_sub = self.create_subscription(String, '/detections/labels', self.label_callback, 10)
        self.distance_sub = self.create_subscription(Float32, '/detections/distance', self.distance_callback, 10)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_pub = self.create_publisher(String, '/robot_dog/speech', 10)

        # -----------------------------
        # Internal state
        # -----------------------------
        # Hardcoded toilet goal
        self.toilet_x = 10.0  # 예시
        self.toilet_y = 5.0   # 예시
        self.toilet_yaw = 1.57  # facing toilet

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.detected_label = None
        self.distance_to_toilet = None

        self.bridge = CvBridge()

        self.state = "MOVE_TO_GOAL"  # MOVE_TO_GOAL → ALIGN → APPROACH → BARK

        self.get_logger().info('NavigateToToilet node initialized.')

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    # -----------------------------
    # Callbacks
    # -----------------------------
    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def label_callback(self, msg: String):
        self.detected_label = msg.data

    def distance_callback(self, msg: Float32):
        self.distance_to_toilet = msg.data

    def image_callback(self, msg: Image):
        # Optional: can be used for visualization
        pass

    # -----------------------------
    # Helper functions
    # -----------------------------
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

    def distance_to_goal(self, x, y):
        return math.hypot(x - self.robot_x, y - self.robot_y)

    def angle_to_goal(self, x, y):
        return math.atan2(y - self.robot_y, x - self.robot_x)

    # -----------------------------
    # Main FSM
    # -----------------------------
    def control_loop(self):
        twist = Twist()

        if self.state == "MOVE_TO_GOAL":
            dist = self.distance_to_goal(self.toilet_x, self.toilet_y)
            goal_angle = self.angle_to_goal(self.toilet_x, self.toilet_y)
            angle_error = self.normalize_angle(goal_angle - self.robot_yaw)

            if dist > 0.3:
                twist.linear.x = min(0.3, 0.5 * dist)
                twist.angular.z = 0.5 * angle_error
                self.cmd_pub.publish(twist)
                self.get_logger().info(f"Moving to toilet: dist={dist:.2f}, angle_error={angle_error:.2f}")
            else:
                self.get_logger().info("Reached toilet goal → ALIGN stage")
                self.state = "ALIGN"

        elif self.state == "ALIGN":
            # Align robot yaw to face toilet
            yaw_error = self.normalize_angle(self.toilet_yaw - self.robot_yaw)
            if abs(yaw_error) > 0.05:
                twist.angular.z = 0.3 * yaw_error
                self.cmd_pub.publish(twist)
                self.get_logger().info(f"Aligning yaw: error={yaw_error:.2f}")
            else:
                self.get_logger().info("Aligned → APPROACH stage")
                self.state = "APPROACH"

        elif self.state == "APPROACH":
            # Use perception distance to adjust
            if self.detected_label == "toilet" and self.distance_to_toilet is not None:
                if self.distance_to_toilet > 1.0:
                    twist.linear.x = 0.2
                    self.cmd_pub.publish(twist)
                    self.get_logger().info(f"Approaching toilet: distance={self.distance_to_toilet:.2f}")
                else:
                    self.get_logger().info("Toilet reached → BARK stage")
                    self.state = "BARK"
            else:
                # Cannot detect toilet, stop
                self.cmd_pub.publish(Twist())
                self.get_logger().info("Toilet not detected, holding position...")

        elif self.state == "BARK":
            msg = String()
            msg.data = "bark"
            self.speech_pub.publish(msg)
            self.get_logger().info("BARK! Toilet position reached.")

            # Stop robot
            self.cmd_pub.publish(Twist())
            # Optionally, you could finish or loop
            # self.state = "DONE"

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToToilet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()