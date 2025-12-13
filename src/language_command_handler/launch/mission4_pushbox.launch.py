from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Mission 4 controller (push box mission)
        Node(
            package='language_command_handler',
            executable='mission4_pushbox',   # ← setup.py에 등록한 이름
            name='mission4_pushbox',
            output='screen',
            parameters=[
                # 1) 로봇이 처음 이동할 원점 좌표 (임의 기본값)
                {'origin_x': 0.0},
                {'origin_y': 1.0},
                {'origin_yaw': 0.0},

                # 2) red_area 고정 좌표 (임의 기본값)
                {'red_x': 2.0},
                {'red_y': 0.0},
                {'red_yaw': 0.0},

                # 3) 미션 하이퍼파라미터
                {'side_offset': 2.5},     # red_area 기준 좌/우 이동 거리
                {'forward_dist': 2.5},    # 최종 전진 거리
                {'done_tol': 0.25},       # box/red_area 거리 비교 허용 오차

                # 4) 사용하는 토픽들
                {'labels_topic': '/detections/labels'},
                {'distance_topic': '/detections/distance'},
                {'llm_trigger_topic': '/mission4/llm_trigger'},
                {'llm_side_topic': '/mission4/box_side'},

                # 5) path_tracker 실행 명령
                {'move_go1_cmd': 'ros2 run path_tracker move_go1.py'},
            ],
        ),

        # perception node는 이미 실행 중이라면 여기서 띄울 필요 없음
        # Node(
        #     package='object_recognition',
        #     executable='box_detector',
        #     output='screen'
        # ),
    ])
