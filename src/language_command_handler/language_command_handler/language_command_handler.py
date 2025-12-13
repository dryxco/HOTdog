#!/usr/bin/env python3

"""
ROS2 node: language_command_handler

1. Listen user command via 'language_command' service
2. Call LLM to select the appropriate mission (mission1~6)
3. Execute the selected mission as a ROS2 node
"""

import os
import subprocess
import signal
import time
import yaml
import openai

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import LanguageCommand


def call_LLM(prompt: str, client: openai.OpenAI) -> str:
    """Call LLM to select the appropriate mission"""
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": "You are a helpful robot assistant."},
            {"role": "user", "content": prompt}
        ],
        max_tokens=200,
        temperature=0.0
    )
    return response.choices[0].message.content


def parse_LLM_response(response_text: str) -> str:
    """Parse LLM response and extract a single action name"""
    response = response_text.strip()

    # Remove code fences if any
    if response.startswith("```"):
        lines = response.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].endswith("```"):
            lines = lines[:-1]
        response = "\n".join(lines).strip()

    # Take only the first line/token
    response = response.splitlines()[0].strip()

    # Strip quotes/backticks
    response = response.strip(' "\'`')

    return response


class LanguageCommandHandler(Node):
    """ROS2 node that routes language commands to mission executables"""

    def __init__(self):
        super().__init__('language_command_handler')

        # -----------------------------
        # Config loading
        # -----------------------------
        self.declare_parameter('config_path', 'default')
        self.config_path = self.get_parameter(
            'config_path'
        ).get_parameter_value().string_value

        if self.config_path == 'default':
            self.get_logger().error(
                "config_path parameter not set. Please provide a valid YAML config."
            )
            raise RuntimeError("Missing config_path")

        with open(self.config_path, 'r') as f:
            config_data = yaml.safe_load(f)

        # OpenAI API key
        api_key = config_data.get('OPENAI_API_KEY', None)
        if api_key == 'bashrc':
            api_key = os.getenv('OPENAI_API_KEY')

        if not api_key:
            raise RuntimeError("OPENAI_API_KEY not found")

        self.openai_client = openai.OpenAI(api_key=api_key)

        # -----------------------------
        # Mission definitions
        # -----------------------------
        self.node_action_candidates = [
            "mission1_toilet",
            "mission2_food",
            "mission3_red_cone",
            "mission3_green_cone",
            "mission3_blue_cone",
            "mission4_box",
            "mission5_stopsign",
            "mission6_nurse",
        ]

        # Build mission-aware prompt (README-based, language-robust)
        self.prompt = f"""
You are a routing module for a robot.

Your task:
- Read the USER COMMAND.
- Choose EXACTLY ONE action from ACTION_CANDIDATES.
- Output ONLY the action name.

IMPORTANT INTERPRETATION RULES:
- User commands may be indirect, emotional, metaphorical, or urgent.
- Do NOT rely only on literal keywords.
- Infer the user's INTENT from meaning and context.
- Complaints, desires, or physical states imply an action request.
- Slang, exaggeration, or casual expressions are common.

Do NOT include explanations, punctuation, quotes, or code blocks.
Mission descriptions and typical phrasing:

1) mission1_toilet
	INTENT: The user urgently needs to go to the toilet or restroom.
	This may be expressed indirectly or emotionally.
	
	Examples:
	- "I really need to go to the bathroom."
	- "I feel like I’m about to poop."
	- "I can’t hold it anymore."
	- "My stomach hurts, I need a toilet now."
	- "I’m desperate, where’s the restroom?"
	These expressions indicate urgency, discomfort, or bodily need.

2) mission2_food
INTENT: The user wants to find something edible because they are hungry.
This may be expressed as a feeling or complaint.

Examples:- "I’m hungry."
- "Is there anything I can eat?"
- "I want something to eat."
- "I haven’t eaten all day."
- "I feel starving."
Mentions of hunger, food desire, or edible objects imply this mission.

3.1) mission3_red_cone
INTENT: The user wants the robot to go to the RED cone.
Color cues include: "red", "reddish", "빨강", "빨간", "red cone"
Examples:
- "Go to the red cone."
- "Move to the 빨간 cone."
- "That red one."

3.2) mission3_green_cone
INTENT: The user wants the robot to go to the GREEN cone.
Color cues include: "green", "greenish", "초록", "초록색", "green cone"
Examples:
- "Go to the green cone."
- "Move to the 초록 cone."
- "That green one."

3.3) mission3_blue_cone
INTENT: The user wants the robot to go to the BLUE cone.
Color cues include: "blue", "bluish", "파랑", "파란", "blue cone"
Examples:
- "Go to the blue cone."
- "Move to the 파란 cone."
- "That blue one."

4) mission4_box
INTENT: The user wants the robot to push or move a box to a goal area.

Examples:
- "Move the box over there."
- "Push that box to the red zone."
- "Deliver the box."
Any request involving pushing, moving, or delivering a box implies this mission.

5) mission5_emptyroom
INTENT: The user wants to go to a room that is allowed or safe.

Examples:
- "Go to an empty room."
- "Find a room without a stop sign."
- "Move to the empty room without stop sign."
Avoiding stop signs or restricted areas implies this mission.

6) mission6_nurse
INTENT: The user wants the robot to approach a nurse and move around her.

Examples:
- "Find the nurse."
- "Go to that nurse."
- "Circle around her."
Mentions of nurse or rotating around a person imply this mission.

USER COMMAND:USER_COMMAND
ACTION CANDIDATES:ACTION_CANDIDATES""".strip()

        # Insert action list into prompt
        action_lines = "\n".join(f"- {a}" for a in self.node_action_candidates)
        self.prompt = self.prompt.replace("ACTION_CANDIDATES", action_lines)

        self.get_logger().info(f"Loaded prompt:\n{self.prompt}")

        # -----------------------------
        # Process management
        # -----------------------------
        self.current_action = None
        self.current_process = None
        self.pkg_name = 'language_command_handler'

        pkg_share_dir = get_package_share_directory(self.pkg_name)
        workspace_install_path = os.path.join(
            pkg_share_dir, '..', '..', '..', 'setup.bash'
        )
        self.workspace_install_path = os.path.abspath(workspace_install_path)

        # -----------------------------
        # ROS2 service
        # -----------------------------
        self.language_command_service = self.create_service(
            LanguageCommand,
            '/language_command',
            self.language_command_callback
        )

        self.get_logger().info("Language command handler node initialized")

    def stop_previous_action(self):
        """Stop currently running mission"""
        if self.current_process is not None:
            self.get_logger().info(
                f"Stopping previous action: {self.current_action}"
            )
            try:
                os.killpg(
                    os.getpgid(self.current_process.pid),
                    signal.SIGINT
                )
                self.current_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(
                    os.getpgid(self.current_process.pid),
                    signal.SIGKILL
                )
                self.current_process.wait()
            finally:
                self.current_process = None
                self.current_action = None

    def start_action(self, action_name: str):
        """Start selected mission"""
        self.stop_previous_action()
        time.sleep(1)

        if action_name not in self.node_action_candidates:
            self.get_logger().error(f"Unknown action: {action_name}")
            return

        command = (
            f"source {self.workspace_install_path} && "
            f"ros2 run {self.pkg_name} {action_name}"
        )

        self.get_logger().info(f"Starting mission: {action_name}")

        self.current_process = subprocess.Popen(
            command,
            shell=True,
            executable='/bin/bash',
            preexec_fn=os.setsid,
        )
        self.current_action = action_name

    def language_command_callback(self, request, response):
        """Service callback"""
        user_command = request.command
        self.get_logger().info(f"Received command: {user_command}")

        prompt = self.prompt.replace("USER_COMMAND", user_command)

        try:
            llm_response = call_LLM(prompt, self.openai_client)
            self.get_logger().info(f"LLM response: {llm_response}")

            selected_action = parse_LLM_response(llm_response)
            self.get_logger().info(f"Selected action: {selected_action}")

            if selected_action not in self.node_action_candidates:
                response.response_message = f"Unknown action: {selected_action}"
                return response

            self.start_action(selected_action)
            response.response_message = f"Started mission: {selected_action}"

        except Exception as e:
            response.response_message = f"Error: {str(e)}"
            self.get_logger().error(response.response_message)

        return response

    def cleanup(self):
        """Cleanup before shutdown"""
        self.stop_previous_action()


def main(args=None):
    rclpy.init(args=args)
    node = LanguageCommandHandler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
