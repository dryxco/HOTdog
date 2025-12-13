#/usr/bin/env python3
"""
This code is for ROS2 launch file 'mission6_nurse.launch.py'
This launch file will start the mission6_nurse node (mission 6)
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    start_mission6_node_cmd = Node(
        package='language_command_handler',
        executable='mission6_nurse.py',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_mission6_node_cmd)

    return ld