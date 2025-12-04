#!/usr/bin/env python3
"""
This code is for ROS2 node 'rotate_around_nurse'
Mission 6: Find the nurse in the break room and rotate around her.

Strategy:
1. Navigate to couch area (nurse is next to couch)
2. Search for nurse by rotating
3. If not found, move to next search position around couch
벽 벽 벽 벽 벽
벽 [COUCH] ←───── Position 1 (오른쪽)
벽    ↑    ↖
      │      Position 3 (대각선)
      │
   Position 2 (앞쪽)
4. When found, approach and rotate around her
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


class RotateAroundNurseNode(Node):
    """
    A ROS2 node that finds nurse near couch and rotates around her
    """

    def __init__(self):
        super().__init__('rotate_around_nurse')
        self.get_logger().info('Rotate around nurse node initialized')

        # TODO: Set actual couch coordinates from map
        self.couch_x = 5.0  # Couch X coordinate
        self.couch_y = 3.0  # Couch Y coordinate

        # Search positions around couch (will be calculated in setup)
        self.search_radius = 2.0  # Distance from couch to search positions
        self.search_positions = []
        self.current_search_index = 0
        self.setup_search_positions()

        # Parameters
        self.rotation_radius = 1.0      # Distance to maintain from nurse (meters)
        self.rotation_speed = 0.3       # Angular speed for circling (rad/s)
        self.linear_speed = 0.2         # Linear speed (m/s)
        self.approach_speed = 0.3       # Speed when navigating
        self.position_tolerance = 0.3   # Tolerance for reaching goal (meters)
        self.angle_tolerance = 0.1      # Tolerance for angle alignment (radians)
        self.search_rotation_count = 0  # Count rotation during search
        self.max_search_rotation = 2 * math.pi  # One full rotation

        # Robot pose (from odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.initial_search_yaw = 0.0

        # Nurse detection
        self.nurse_detected = False
        self.nurse_x = 0.0
        self.nurse_y = 0.0
        self.nurse_distance = 0.0
        self.nurse_angle = 0.0

        # State machine
        self.state = 'NAVIGATING'

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # TODO: Subscribe to perception node's person detection topic
        self.person_sub = self.create_subscription(
            PoseStamped,
            '/perception/detected_person',
            self.person_callback,
            10
        )

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Couch position: ({self.couch_x}, {self.couch_y})')
        self.get_logger().info(f'Search positions: {len(self.search_positions)} points around couch')

    def setup_search_positions(self):
        """
        Create search positions around the couch
        Couch is in corner (wall on left and back), so only accessible from:
        - Right side (+X direction)
        - Front side (-Y direction)
        
        벽 벽 벽 벽
        벽 [COUCH] ← Position 1 (오른쪽에서)
        벽    ↑
              Position 2 (앞에서)
        """
        # Position 1: 소파 오른쪽에서 (소파를 왼쪽으로 바라봄)
        self.search_positions.append((
            self.couch_x + self.search_radius,  # 소파 오른쪽
            self.couch_y,
            math.pi  # 왼쪽(소파 방향)을 바라봄
        ))

        # Position 2: 소파 앞에서 (소파를 위로 바라봄)
        self.search_positions.append((
            self.couch_x,
            self.couch_y - self.search_radius,  # 소파 앞쪽
            math.pi / 2  # 위쪽(소파 방향)을 바라봄
        ))

        # Position 3: 대각선 (오른쪽 앞)
        self.search_positions.append((
            self.couch_x + self.search_radius * 0.7,
            self.couch_y - self.search_radius * 0.7,
            math.pi * 3 / 4  # 소파 방향(왼쪽 위)을 바라봄
        ))

        self.get_logger().info(f'Search positions (corner couch): {self.search_positions}')

    def odom_callback(self, msg):
        """
        Update robot pose from odometry
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def person_callback(self, msg):
        """
        Receive detected person position from perception node
        """
        self.nurse_x = msg.pose.position.x
        self.nurse_y = msg.pose.position.y

        self.nurse_distance = math.sqrt(self.nurse_x**2 + self.nurse_y**2)
        self.nurse_angle = math.atan2(self.nurse_y, self.nurse_x)

        self.nurse_detected = True
        self.get_logger().info(
            f'Nurse detected! Distance: {self.nurse_distance:.2f}m, Angle: {math.degrees(self.nurse_angle):.1f}°',
            throttle_duration_sec=1.0
        )

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_distance_to_goal(self, goal_x, goal_y):
        """Calculate distance from robot to goal"""
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        return math.sqrt(dx**2 + dy**2)

    def get_angle_to_goal(self, goal_x, goal_y):
        """Calculate angle from robot to goal"""
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        return math.atan2(dy, dx)

    def control_loop(self):
        """Main control loop - state machine"""
        twist = Twist()

        if self.state == 'NAVIGATING':
            twist = self.navigate_to_search_position()

        elif self.state == 'SEARCHING':
            twist = self.search_for_nurse()

        elif self.state == 'APPROACHING':
            twist = self.approach_nurse()

        elif self.state == 'ROTATING':
            twist = self.rotate_around_nurse()

        elif self.state == 'COMPLETED':
            self.get_logger().info('Mission 6 completed!', throttle_duration_sec=5.0)
            twist = Twist()

        self.cmd_vel_pub.publish(twist)

    def navigate_to_search_position(self):
        """Navigate to current search position around couch"""
        twist = Twist()

        if self.current_search_index >= len(self.search_positions):
            self.get_logger().error('Searched all positions but nurse not found!')
            self.state = 'COMPLETED'
            return twist

        goal_x, goal_y, goal_yaw = self.search_positions[self.current_search_index]

        distance = self.get_distance_to_goal(goal_x, goal_y)
        angle_to_goal = self.get_angle_to_goal(goal_x, goal_y)
        angle_error = self.normalize_angle(angle_to_goal - self.robot_yaw)

        self.get_logger().info(
            f'Navigating to search position {self.current_search_index + 1}/{len(self.search_positions)}, distance: {distance:.2f}m',
            throttle_duration_sec=2.0
        )

        # Check if nurse detected while navigating
        if self.nurse_detected:
            self.get_logger().info('Nurse detected while navigating! Approaching...')
            self.state = 'APPROACHING'
            return twist

        if distance < self.position_tolerance:
            # Reached search position, now align to face couch
            yaw_error = self.normalize_angle(goal_yaw - self.robot_yaw)

            if abs(yaw_error) < self.angle_tolerance:
                self.get_logger().info(f'Reached search position {self.current_search_index + 1}, starting search...')
                self.state = 'SEARCHING'
                self.search_rotation_count = 0
                self.initial_search_yaw = self.robot_yaw
            else:
                twist.angular.z = 0.5 * yaw_error
        else:
            # Navigate to position
            if abs(angle_error) > self.angle_tolerance:
                twist.angular.z = 0.5 * angle_error
                twist.angular.z = max(-0.5, min(0.5, twist.angular.z))
            else:
                twist.linear.x = min(self.approach_speed, 0.5 * distance)
                twist.angular.z = 0.3 * angle_error

        return twist

    def search_for_nurse(self):
        """Rotate in place to search for nurse"""
        twist = Twist()

        # Check if nurse found
        if self.nurse_detected:
            self.get_logger().info('Nurse found! Approaching...')
            self.state = 'APPROACHING'
            return twist

        # Calculate how much we've rotated
        rotation_done = abs(self.normalize_angle(self.robot_yaw - self.initial_search_yaw))

        # Update rotation count (handle wrap-around)
        self.search_rotation_count += 0.1 * 0.3  # dt * angular_speed (approximate)

        if self.search_rotation_count >= self.max_search_rotation:
            # Completed one full rotation, nurse not found here
            self.get_logger().warn(f'Nurse not found at position {self.current_search_index + 1}, moving to next...')
            self.current_search_index += 1
            self.state = 'NAVIGATING'
            return twist

        # Rotate slowly to search
        twist.angular.z = 0.3
        self.get_logger().info(
            f'Searching... (rotation: {math.degrees(self.search_rotation_count):.0f}°)',
            throttle_duration_sec=1.0
        )

        return twist

    def approach_nurse(self):
        """Approach nurse until at rotation radius"""
        twist = Twist()

        if not self.nurse_detected:
            self.get_logger().warn('Lost nurse, continuing search...')
            self.state = 'SEARCHING'
            self.search_rotation_count = 0
            self.initial_search_yaw = self.robot_yaw
            return twist

        # Check if at rotation radius
        if self.nurse_distance <= self.rotation_radius + 0.1:
            self.get_logger().info('At rotation distance! Starting rotation around nurse...')
            self.state = 'ROTATING'
            return twist

        # Move towards nurse
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
        """Rotate around nurse in circular motion"""
        twist = Twist()

        if not self.nurse_detected:
            self.get_logger().warn('Lost nurse during rotation!')
            # Try to re-acquire by searching
            self.state = 'SEARCHING'
            self.search_rotation_count = 0
            self.initial_search_yaw = self.robot_yaw
            return twist

        # Circular motion
        twist.linear.x = self.linear_speed

        # Radius correction
        radius_error = self.nurse_distance - self.rotation_radius

        # Keep nurse at right side (-90 degrees)
        target_nurse_angle = -math.pi / 2
        angle_error = self.normalize_angle(self.nurse_angle - target_nurse_angle)

        twist.angular.z = self.rotation_speed
        twist.angular.z += 0.5 * radius_error
        twist.angular.z += 0.3 * angle_error

        twist.angular.z = max(-0.6, min(0.6, twist.angular.z))

        self.get_logger().info(
            f'Rotating around nurse, distance: {self.nurse_distance:.2f}m',
            throttle_duration_sec=1.0
        )

        return twist


def main(args=None):
    rclpy.init(args=args)
    node = RotateAroundNurseNode()

    try:
        rclpy.spin(node)
    finally:
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()