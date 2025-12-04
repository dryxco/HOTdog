#!/usr/bin/env python3
"""
This code is for ROS2 node 'rotate_around_nurse'
Mission 6: Find the nurse in the break room and rotate around her.

This node will:
1. Navigate to break room using odometry
2. Search for nurse using perception
3. Rotate around the detected nurse
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math


class RotateAroundNurseNode(Node):
    """
    A ROS2 node that navigates to break room, finds nurse, and rotates around her
    """

    def __init__(self):
        super().__init__('rotate_around_nurse')
        self.get_logger().info('Rotate around nurse node initialized')

        # TODO: 실제 breakroom의 x, y 좌표로 수정 필요함
        self.breakroom_x = 5.0  # Set actual break room X coordinate
        self.breakroom_y = 3.0  # Set actual break room Y coordinate

        # Parameters
        self.rotation_radius = 1.0      # Distance to maintain from nurse (meters)
        self.rotation_speed = 0.3       # Angular speed for circling (rad/s)
        self.linear_speed = 0.2         # Linear speed (m/s)
        self.approach_speed = 0.3       # Speed when navigating to break room
        self.position_tolerance = 0.3   # Tolerance for reaching goal (meters)
        self.angle_tolerance = 0.1      # Tolerance for angle alignment (radians)

        # Robot pose (from odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Nurse detection
        self.nurse_detected = False
        self.nurse_x = 0.0  # Nurse position in robot frame
        self.nurse_y = 0.0
        self.nurse_distance = 0.0
        self.nurse_angle = 0.0

        # State machine
        # NAVIGATING -> SEARCHING -> APPROACHING -> ROTATING -> COMPLETED
        self.state = 'NAVIGATING'

        # Publishers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # TODO: Subscribe to perception node's person detection topic
        # Modify the topic name and message type based on perception node after implementation
        self.person_subscriber = self.create_subscription(
            PoseStamped,  
            '/perception/detected_person', 
            self.person_callback,
            10
        )

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Target break room: ({self.breakroom_x}, {self.breakroom_y})')

    def odom_callback(self, msg):
        """
        Update robot pose from odometry
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def person_callback(self, msg):
        """
        Receive detected person position from perception node
        TODO: Modify based on perception node's output format after implementation
        """
        # Assuming perception outputs person position relative to robot
        self.nurse_x = msg.pose.position.x
        self.nurse_y = msg.pose.position.y

        # Calculate distance and angle to nurse
        self.nurse_distance = math.sqrt(self.nurse_x**2 + self.nurse_y**2)
        self.nurse_angle = math.atan2(self.nurse_y, self.nurse_x)

        self.nurse_detected = True
        self.get_logger().info(
            f'Nurse detected at distance: {self.nurse_distance:.2f}m, angle: {math.degrees(self.nurse_angle):.1f}°',
            throttle_duration_sec=2.0
        )

    def normalize_angle(self, angle):
        """
        Normalize angle to [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_distance_to_goal(self, goal_x, goal_y):
        """
        Calculate distance from robot to goal
        """
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        return math.sqrt(dx**2 + dy**2)

    def get_angle_to_goal(self, goal_x, goal_y):
        """
        Calculate angle from robot to goal
        """
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        return math.atan2(dy, dx)

    def control_loop(self):
        """
        Main control loop - state machine
        """
        twist = Twist()

        if self.state == 'NAVIGATING':
            twist = self.navigate_to_breakroom()

        elif self.state == 'SEARCHING':
            twist = self.search_for_nurse()

        elif self.state == 'APPROACHING':
            twist = self.approach_nurse()

        elif self.state == 'ROTATING':
            twist = self.rotate_around_nurse()

        elif self.state == 'COMPLETED':
            self.get_logger().info('Mission 6 completed!', throttle_duration_sec=5.0)
            twist = Twist()  # Stop

        # Publish velocity command
        self.publisher.publish(twist)

    def navigate_to_breakroom(self):
        """
        Navigate to break room using simple proportional control
        """
        twist = Twist()

        distance = self.get_distance_to_goal(self.breakroom_x, self.breakroom_y)
        angle_to_goal = self.get_angle_to_goal(self.breakroom_x, self.breakroom_y)
        angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)

        self.get_logger().info(
            f'Navigating to break room, distance: {distance:.2f}m',
            throttle_duration_sec=2.0
        )

        if distance < self.position_tolerance:
            # Reached break room
            self.get_logger().info('Reached break room! Starting to search for nurse...')
            self.state = 'SEARCHING'
            return twist

        # First rotate to face the goal
        if abs(angle_error) > self.angle_tolerance:
            twist.angular.z = 0.5 * angle_error
            twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
        else:
            # Move forward
            twist.linear.x = min(self.approach_speed, 0.5 * distance)
            twist.angular.z = 0.3 * angle_error  # Small correction while moving

        return twist

    def search_for_nurse(self):
        """
        Rotate in place to search for nurse
        """
        twist = Twist()

        if self.nurse_detected:
            self.get_logger().info('Nurse found! Approaching...')
            self.state = 'APPROACHING'
            return twist

        # Rotate slowly to search
        twist.angular.z = 0.3
        self.get_logger().info('Searching for nurse...', throttle_duration_sec=2.0)

        return twist

    def approach_nurse(self):
        """
        Approach nurse until at rotation radius
        """
        twist = Twist()

        if not self.nurse_detected:
            self.get_logger().warn('Lost nurse, searching again...')
            self.state = 'SEARCHING'
            return twist

        # Check if at rotation radius
        if self.nurse_distance <= self.rotation_radius + 0.1:
            self.get_logger().info('At rotation distance! Starting rotation...')
            self.state = 'ROTATING'
            return twist

        # Move towards nurse
        # First align to nurse
        if abs(self.nurse_angle) > self.angle_tolerance:
            twist.angular.z = 0.5 * self.nurse_angle
        else:
            twist.linear.x = min(self.linear_speed, 0.3 * (self.nurse_distance - self.rotation_radius))
            twist.angular.z = 0.3 * self.nurse_angle

        self.get_logger().info(
            f'Approaching nurse, distance: {self.nurse_distance:.2f}m',
            throttle_duration_sec=1.0
        )

        return twist

    def rotate_around_nurse(self):
        """
        Rotate around nurse in circular motion
        """
        twist = Twist()

        if not self.nurse_detected:
            self.get_logger().warn('Lost nurse during rotation, searching again...')
            self.state = 'SEARCHING'
            return twist

        # Circular motion control
        # Linear velocity (tangent to circle)
        twist.linear.x = self.linear_speed

        # Angular velocity for circular path
        # Base rotation + radius correction + keep nurse at side
        radius_error = self.nurse_distance - self.rotation_radius
        
        # We want nurse to be at -90 degrees (right side) while circling
        target_nurse_angle = -math.pi / 2
        angle_error = self.normalize_angle(self.nurse_angle - target_nurse_angle)

        twist.angular.z = self.rotation_speed
        twist.angular.z += 0.5 * radius_error   # Correct for radius
        twist.angular.z += 0.3 * angle_error    # Keep nurse at side

        # Limit angular velocity
        twist.angular.z = max(-0.6, min(0.6, twist.angular.z))

        self.get_logger().info(
            f'Rotating around nurse, distance: {self.nurse_distance:.2f}m, angle: {math.degrees(self.nurse_angle):.1f}°',
            throttle_duration_sec=1.0
        )

        return twist


def main(args=None):
    """
    Main function to initialize and run the ROS2 node
    """
    rclpy.init(args=args)
    rotate_around_nurse_node = RotateAroundNurseNode()

    try:
        rclpy.spin(rotate_around_nurse_node)
    finally:
        # Stop the robot before shutting down
        stop_twist = Twist()
        rotate_around_nurse_node.publisher.publish(stop_twist)
        rotate_around_nurse_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()