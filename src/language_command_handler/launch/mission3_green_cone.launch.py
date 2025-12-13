#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='language_command_handler',
            executable='mission3_green_cone.py',
            output='screen',
        )
    ])
