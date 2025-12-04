"""
ROS2 launch file to start the language command handler node with config file
"""

import os
import getpass
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Generate launch description for language command handler node
    """

    # Get current username and paths dynamically
    username = getpass.getuser()
    home_path = os.path.expanduser("~")    
    print(f"Username: {username}")
    print(f"Home path: {home_path}")

    pkg_name = 'language_command_handler'
    pkg_share_dir = get_package_share_directory(pkg_name)
    config_file_path = os.path.join(pkg_share_dir, 'config', 'command_handler_config.yaml')
    print(f"Package share directory: {pkg_share_dir}")

    # Construct workspace install path
    workspace_install_path = os.path.join(
        pkg_share_dir,
        '..',
        '..',
        '..',
        'setup.bash'
    )
    workspace_install_path = os.path.abspath(workspace_install_path)  
    print(f"Workspace setup.bash path: {workspace_install_path}")

    # Anaconda environmnet name
    anaconda_env_name = 'language_command_handler'

    # Construct path to the Python executable and script
    python_executable = f"{home_path}/anaconda3/envs/{anaconda_env_name}/bin/python"
    if not os.path.exists(python_executable):
        print(f"Python executable not found: {python_executable}")
        return None
    print(f"Python executable: {python_executable}")

    python_script_path = os.path.join(
        pkg_share_dir,
        '..',
        '..',
        'lib',
        'language_command_handler',
        'language_command_handler.py'
    )
    python_script_path = os.path.abspath(python_script_path)  
    print(f"Python script path: {python_script_path}")  

    start_language_command_handler_cmd = ExecuteProcess(
        cmd = [
            "bash", "-c",
            [
                f"source {home_path}/anaconda3/etc/profile.d/conda.sh && "
                f"conda activate {anaconda_env_name} && "
                f"source /opt/ros/jazzy/setup.bash && "
                f"source {workspace_install_path} && "
                "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libgcc_s.so.1 && "
                f"{python_executable} {python_script_path} --ros-args -p config_path:={config_file_path}",
            ]
        ],
        output="screen",
    )
    
    ld = LaunchDescription()
    ld.add_action(start_language_command_handler_cmd)
    
    return ld