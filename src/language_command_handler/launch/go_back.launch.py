# usr/bin/env python3
"""
This code is for ROS2 launch file 'go_back.launch.py'
This launch file will start the go_back node
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    start_go_back_node_cmd = Node(
        package='language_command_handler',
        executable='go_back.py',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_go_back_node_cmd)
    return ld