#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='language_command_handler',
            executable='mission5_emptyroom.py',
            name='mission5_emptyroom',
            output='screen',
        )
    ])
