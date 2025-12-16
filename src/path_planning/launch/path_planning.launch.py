import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    pkg_dir = get_package_share_directory('path_planning')

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    # Path Planner Node
    path_planner_node = Node(
        package='path_planning',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'params.yaml')
        ]
    )

    # # RViz Node
    # rviz_config_file = os.path.join(pkg_dir, 'rviz', 'path_planning.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('use_rviz')),
    #     arguments=['-d', rviz_config_file]
    # )

    return LaunchDescription([
        use_rviz_arg,
        path_planner_node,
        # rviz_node
    ])
