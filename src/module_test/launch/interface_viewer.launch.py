from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="module_test",
                executable="interface_viewer",
                name="interface_viewer",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera_top/image",
                        "object_topic": "/detections/labels",
                        "distance_topic": "/detections/distance",
                        "speech_topic": "/robot_dog/speech",
                        "window_title": "Module Test Viewer",
                        "timer_period": 0.05,
                    }
                ],
            )
        ]
    )


