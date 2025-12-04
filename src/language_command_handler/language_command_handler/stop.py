#!/usr/bin/env python3
"""
This code is for ROS2 node 'stop'
This node will publish velocity command geometry_msgs/msg/Twist to '/cmd_vel' topic
The command velocity is all zero.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StopNode(Node):
    """
    A ROS2 node that publishes velocity command to '/cmd_vel' topic
    """
    def __init__(self):
        super().__init__('stop')
        self.get_logger().info('Stop node initialized')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.index = 0

    def publish_velocity(self):
        """
        Publish velocity command to '/cmd_vel' topic
        """
        if self.index == 0:
            linear_x = 0.01
            self.index = 1
        else:
            linear_x = -0.01
            self.index = 0
        twist = Twist()
        twist.linear.x = linear_x # small value to avoid robot from dying
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
    stop_node = StopNode()
    try:
        rclpy.spin(stop_node)
    finally:
        stop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()