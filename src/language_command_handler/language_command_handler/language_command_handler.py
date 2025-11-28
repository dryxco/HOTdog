#!/usr/bin/env python3

"""
This code is for ROS2 node 'language_command_handler' which 
1. Listen user command via 'language_command' service
2. Call LLM to select the appropriate action
3. Current code selects ROS2 node among 'go_front', 'go_back' and 'stop'
"""

import os
import sys
import subprocess
import signal
import openai
import yaml
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_interfaces.srv import LanguageCommand


def call_LLM(prompt: str, client: openai.OpenAI) -> str:
    """
    Call LLM to select the appropriate action
    """
    # Call the OpenAI API
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": "You are a helpful robot assistant."},
            {"role": "user", "content": prompt}
        ],
        max_tokens=1000,
        temperature=0.0
    )
    
    return response.choices[0].message.content


def parse_LLM_response(response_text: str) -> str:
    """
    Parse the response text from LLM to select the appropriate action
    """
    response = response_text.strip()
    if response.startswith("```python"):
        response = response[len("```python"):].strip()
    if response.endswith("```"):
        response = response[:-len("```")].strip()

    return response


class LanguageCommandHandler(Node):
    """
    A ROS2 node that listens to user command and calls LLM to select the appropriate action
    """
    def __init__(self):
        super().__init__('language_command_handler')

        # Declare and get the 'config_path' parameter
        self.declare_parameter('config_path', 'default')
        self.config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if self.config_path == 'default':
            return

        # Load config file and update the class variables
        with open(self.config_path, 'r') as f:
            config_data = yaml.safe_load(f)        
        if config_data['OPENAI_API_KEY'] == 'bashrc':
            config_data['OPENAI_API_KEY'] = os.getenv('OPENAI_API_KEY')
        self.openai_client = openai.OpenAI(api_key=config_data['OPENAI_API_KEY'])
        self.prompt = config_data['prompt']
        self.node_action_candidates = config_data['action_candidates']['node']
        self.launch_action_candidates = config_data['action_candidates']['launch']
        self.current_action = None
        self.current_process = None  # Track the running subprocess
        self.pkg_name = 'language_command_handler'

        # Path to setup.bash file
        pkg_share_dir = get_package_share_directory(self.pkg_name)
        workspace_install_path = os.path.join(pkg_share_dir, '..', '..', '..', 'setup.bash')
        self.workspace_install_path = os.path.abspath(workspace_install_path)

        # Update prompt with action candidates
        self.action_string = ""
        for node_action in self.node_action_candidates:
            self.action_string += f"- {node_action}\n"
        for launch_action in self.launch_action_candidates:
            self.action_string += f"- {launch_action}\n"
        self.prompt = self.prompt.replace("ACTION_CANDIDATES", self.action_string)

        self.get_logger().info(f'Prompt: \n{self.prompt}\n')        

        # Create OpenAI client
        self.openai_client = openai.OpenAI(api_key=config_data['OPENAI_API_KEY'])
        if not self.openai_client:
            self.get_logger().error('Failed to create OpenAI client')
            return
       
        # Create a service to handle user command
        self.language_command_service = self.create_service(
            LanguageCommand,
            '/language_command',
            self.language_command_callback
        )

        self.get_logger().info('Language command handler node initialized')

    def stop_previous_action(self):
        """
        Stop the currently running action (node or launch file)
        """
        if self.current_process is not None:
            self.get_logger().info(f'Stopping previous action: {self.current_action}')
            try:
                # Send SIGINT to the process group to terminate all child processes
                os.killpg(os.getpgid(self.current_process.pid), signal.SIGINT)
                self.current_process.wait(timeout=5)
                self.get_logger().info('Previous action stopped successfully')
            except subprocess.TimeoutExpired:
                self.get_logger().warn('Process did not terminate gracefully, killing it')
                os.killpg(os.getpgid(self.current_process.pid), signal.SIGKILL)
                self.current_process.wait()
            except Exception as e:
                self.get_logger().error(f'Error stopping previous action: {str(e)}')
            finally:
                self.current_process = None
                self.current_action = None

    def start_action(self, action_name: str):
        """
        Start a new action (node or launch file)
        """
        # Stop previous action first
        self.stop_previous_action()
        time.sleep(1)
        
        # Determine if it's a node or launch action
        if action_name in self.node_action_candidates:
            command = f"source {self.workspace_install_path} && ros2 run {self.pkg_name} {action_name}"
            self.get_logger().info(f'Starting node action: {action_name}')
        elif action_name in self.launch_action_candidates:
            command = f"source {self.workspace_install_path} && ros2 launch {self.pkg_name} {action_name}"
            self.get_logger().info(f'Starting launch action: {action_name}')
        else:
            self.get_logger().error(f'Unknown action: {action_name}')
            return
        
        try:
            # Start the new process in a new process group
            self.current_process = subprocess.Popen(
                command,
                shell=True,
                executable='/bin/bash',
                preexec_fn=os.setsid,  # Create new process group
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.current_action = action_name
            self.get_logger().info(f'Action {action_name} started with PID: {self.current_process.pid}')
        except Exception as e:
            self.get_logger().error(f'Error starting action {action_name}: {str(e)}')

    def language_command_callback(self, request, response):
        """
        Callback function for user command service
        """       
        
        user_command = request.command
        self.get_logger().info(f'Received user command: {user_command}')

        # Update prompt with user command
        prompt = self.prompt.replace("USER_COMMAND", user_command)
        self.get_logger().info(f'Prompt: \n{prompt}\n')
        
        try:
            # Call LLM to get actionT
            llm_response = call_LLM(prompt, self.openai_client)
            self.get_logger().info(f'LLM response: \n{llm_response}\n')
            
            # Parse response
            selected_action = parse_LLM_response(llm_response)
            self.get_logger().info(f'Selected action: \n{selected_action}\n')
            
            # Determine action type and create response message
            if selected_action in self.node_action_candidates:
                response.response_message = f'Start ROS2 node: {selected_action}'
            elif selected_action in self.launch_action_candidates:
                response.response_message = f'Start ROS2 launch: {selected_action}'
            else:
                response.response_message = f'Unknown action: {selected_action}'
                self.get_logger().error(response.response_message)
                return response
            
            # Start the selected action (stops previous action first)
            self.start_action(selected_action)
            
        except Exception as e:
            error_msg = f'Error processing command: {str(e)}'
            self.get_logger().error(error_msg)
            response.response_message = error_msg
        
        return response

    def cleanup(self):
        """
        Cleanup method to stop any running processes before shutdown
        """
        self.get_logger().info('Cleaning up...')
        self.stop_previous_action()


def main(args=None):
    """
    Main function to initialize ROS2 node
    """
    
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node
    language_command_handler = LanguageCommandHandler()

    try:
        rclpy.spin(language_command_handler)
    except KeyboardInterrupt:
        language_command_handler.get_logger().info('Interrupted by user')
    finally:
        language_command_handler.cleanup()
        language_command_handler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    print('started python script')
    main()