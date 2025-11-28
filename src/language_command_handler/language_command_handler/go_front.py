#!/usr/bin/env python3
"""
This code is for ROS2 node 'go_front'
This node will publish velocity command geometry_msgs/msg/Twist to '/cmd_vel' topic
The command velocity is x=0.2 with all other velocity components are 0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GoFrontNode(Node):
    """
    A ROS2 node that publishes velocity command to '/cmd_vel' topic
    """
    def __init__(self):
        super().__init__('go_front')
        self.get_logger().info('Go front node initialized')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def publish_velocity(self):
        """
        Publish velocity command to '/cmd_vel' topic
        """
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    """
    Main function to initialize and run the ROS2 node
    """
    rclpy.init(args=args)
    go_front_node = GoFrontNode()
    try:
        rclpy.spin(go_front_node)
    finally:
        go_front_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()