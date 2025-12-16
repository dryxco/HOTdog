#! /usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory  # <-- use this

def generate_launch_description():
    package_name = 'go1_simulation'
    slam_toolbox_package = 'slam_toolbox'

    # Resolve package share directories as real paths (NOT via FindPackageShare)
    pkg_share = get_package_share_directory(package_name)
    slam_toolbox_share = get_package_share_directory(slam_toolbox_package)

    # Files
    slam_toolbox_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')
    default_map_file = os.path.join(pkg_share, 'maps', 'my_world.yaml')  # hospital.yaml must exist

    # Arg: choose new map vs load existing
    generate_new_map = LaunchConfiguration('generate_new_map')
    map_yaml = LaunchConfiguration('map_yaml')

    declare_generate_new_map_cmd = DeclareLaunchArgument(
        name='generate_new_map',
        default_value='false',
        description='If true, run SLAM to create a new map; if false, load maps/hospital.yaml'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        name='map_yaml',
        default_value=default_map_file,
        description='Full path to the map yaml to load when generate_new_map is false'
    )

    # (A) Create a new map with slam_toolbox (async online)
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_toolbox_config
        }.items(),
        condition=IfCondition(generate_new_map)
    )

    # (B) Load an existing map (map_server + lifecycle manager)
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': True
        }],
        condition=UnlessCondition(generate_new_map)
    )

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 10.0,
            'use_sim_time': True
        }],
        condition=UnlessCondition(generate_new_map)
    )

    ld = LaunchDescription()
    ld.add_action(declare_generate_new_map_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(slam_toolbox_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_cmd)
    return ld