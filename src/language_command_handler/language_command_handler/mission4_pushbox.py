#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


class Mission4PushBox(Node):
    """
    Mission 4:
    - recognize box position (3 candidates)
    - go to pre-push pose
    - push straight to goal
    """

    def __init__(self):
        super().__init__('mission4_push_box')

        # ===============================
        # Publishers
        # ===============================
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # ===============================
        # Subscribers
        # ===============================
        self.cmd_sub = self.create_subscription(
            String,
            '/language_command',
            self.command_callback,
            10
        )

        self.box_sub = self.create_subscription(
            String,
            '/box_location',   # LEFT / CENTER / RIGHT
            self.box_callback,
            10
        )

        # ===============================
        # FSM state
        # ===============================
        self.state = 'IDLE'
        self.box_location = None

        # ===============================
        # Hard-coded positions (map frame)
        # ===============================
        # 박스 후보 위치 (중심)
        self.box_positions = {
            'LEFT':   (1.0, 2.0),
            'CENTER': (2.0, 2.0),
            'RIGHT':  (3.0, 2.0),
        }

        # goal 위치 (고정)
        self.goal_position = (2.0, 0.0)

        self.push_distance = 2.5
        self.push_speed = 0.2

        self.get_logger().info('Mission4PushBox node ready.')

    # ==================================================
    # Callbacks
    # ==================================================
    def command_callback(self, msg: String):
        if 'move the box' in msg.data.lower():
            self.get_logger().info('Mission 4 triggered.')
            self.state = 'WAIT_BOX'

    def box_callback(self, msg: String):
        self.box_location = msg.data
        self.get_logger().info(f'Box detected at {self.box_location}')

        if self.state == 'WAIT_BOX':
            self.execute_pre_push()

    # ==================================================
    # Core logic
    # ==================================================
    def execute_pre_push(self):
        bx, by = self.box_positions[self.box_location]
        gx, gy = self.goal_position

        # push direction
        dx = gx - bx
        dy = gy - by
        norm = math.sqrt(dx * dx + dy * dy)
        dx /= norm
        dy /= norm

        # pre-push pose
        px = bx - dx * self.push_distance
        py = by - dy * self.push_distance
        yaw = math.atan2(dy, dx)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = px
        goal.pose.position.y = py
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal)
        self.get_logger().info('Pre-push goal published.')

        self.state = 'PUSH'
        self.create_timer(5.0, self.push_forward)

    def push_forward(self):
        if self.state != 'PUSH':
            return

        self.get_logger().info('Pushing box forward...')
        twist = Twist()
        twist.linear.x = self.push_speed

        start = time.time()
        while time.time() - start < self.push_distance / self.push_speed:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        self.cmd_pub.publish(Twist())
        self.state = 'DONE'
        self.get_logger().info('Mission 4 complete.')

# ==================================================
def main(args=None):
    rclpy.init(args=args)
    node = Mission4PushBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
