#!/usr/bin/env python3
"""
Launch file for mission 1: Navigate to toilet
Starts the 'mission1_toilet' node
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    start_mission1_node_cmd = Node(
        package='language_command_handler',  
        executable='mission1_toilet.py',  
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_mission1_node_cmd)

    return ld
