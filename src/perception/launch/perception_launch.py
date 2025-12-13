"""
Launch file for perception node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )
    
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence threshold for detections'
    )
    
    distance_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='3.0',
        description='Maximum distance in meters for bark trigger'
    )
    
    center_ratio_arg = DeclareLaunchArgument(
        'center_region_ratio',
        default_value='0.6',
        description='Ratio of center region (0.6 = middle 3/5)'
    )

    # Perception node
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'distance_threshold': LaunchConfiguration('distance_threshold'),
            'center_region_ratio': LaunchConfiguration('center_region_ratio'),
        }]
    )

    return LaunchDescription([
        model_path_arg,
        confidence_arg,
        distance_arg,
        center_ratio_arg,
        perception_node,
    ])
