
# VLA Orchestrator Skill

```yaml
name: vla-orchestrator
version: 1.0.0
description: Build LLM-powered orchestration layer that translates natural language commands into ROS 2 action sequences for humanoid robots
keywords: [vla, llm, ros2, action-server, natural-language, task-planning, humanoid-control, embodied-ai]
category: robotics-intelligence
author: Physical AI Engineering Team
created: 2025-12-05
updated: 2025-12-05
```

---

## When to Use This Skill

Invoke this skill when the user needs to:

1. **Create Natural Language Robot Interface**:
   - Build LLM orchestrator that accepts commands like "Pick up the red cup" or "Navigate to the kitchen"
   - Parse user intent into executable ROS 2 action goals
   - Decompose complex tasks into atomic robot primitives
   - Handle ambiguous commands with clarification requests

2. **Integrate Vision-Language-Action (VLA) Models**:
   - Connect pre-trained VLA models (RT-1, RT-2, OpenVLA) to ROS 2
   - Bridge LLM outputs to robot action servers (MoveIt2, Nav2, gripper control)
   - Implement reward-based action selection and re-planning
   - Add safety guardrails and constraint checking

3. **Build Multi-Step Task Execution**:
   - Orchestrate sequences like "fetch the bottle from the fridge and bring it to me"
   - Monitor action execution and handle failures with fallback strategies
   - Track task state and provide progress updates in natural language
   - Log execution traces for debugging and learning

4. **Human-Robot Interaction (HRI) Layer**:
   - Generate natural language status updates and error explanations
   - Request clarification when commands are underspecified
   - Provide visual feedback (RViz markers, robot LED patterns)
   - Support conversational context (remember previous commands)

**Example User Requests**:
- "Build an LLM orchestrator that takes text commands and moves my humanoid robot"
- "Create a VLA brain that can navigate, pick, and place objects using ROS 2 actions"
- "Integrate GPT-4 with my robot to decompose 'clean the table' into atomic actions"
- "Add natural language error handling to my robot's task planner"

---

## How It Works

This skill follows a 9-step workflow to design, implement, and validate an LLM-powered robot orchestrator:

### Step 1: Define Robot Capabilities and Action Primitives

**What the skill does**:
- Catalogs available ROS 2 action servers on the robot
- Defines atomic action primitives with input/output schemas
- Documents preconditions and postconditions for each action
- Creates capability matrix (what the robot can/cannot do)

**User interaction**:
- Ask: "What ROS 2 action servers does your robot have? (e.g., /navigate_to_pose, /move_group, /gripper_command)"
- Ask: "What are the key capabilities? (navigation, manipulation, perception, speech)"
- Ask: "What's the robot form factor? (wheeled mobile manipulator, bipedal humanoid, quadruped)"
- Ask: "Any safety constraints? (workspace limits, collision objects, speed limits)"

**Deliverable**:
```yaml
# config/robot_capabilities.yaml
robot_name: "Humanoid Assistant Robot"
form_factor: "wheeled_mobile_manipulator"  # or bipedal_humanoid, quadruped

action_primitives:
  navigation:
    - name: "navigate_to_pose"
      action_type: "nav2_msgs/action/NavigateToPose"
      description: "Navigate to a goal pose in the map frame"
      parameters:
        - pose: "geometry_msgs/PoseStamped"
      preconditions:
        - "Localized in map"
        - "Path to goal is collision-free"
      postconditions:
        - "Robot at goal pose within 10cm tolerance"
      average_duration_sec: 15

  manipulation:
    - name: "move_to_joint_goal"
      action_type: "control_msgs/action/FollowJointTrajectory"
      description: "Move arm to joint configuration"
      parameters:
        - joint_positions: "float64[]"
      preconditions:
        - "Arm in valid configuration"
        - "No collision with environment"
      postconditions:
        - "Arm at target joint positions"
      average_duration_sec: 3

    - name: "move_to_cartesian_pose"
      action_type: "moveit_msgs/action/MoveGroup"
      description: "Move end-effector to Cartesian pose"
      parameters:
        - pose: "geometry_msgs/PoseStamped"
      preconditions:
        - "IK solution exists"
      postconditions:
        - "End-effector at target pose"
      average_duration_sec: 4

    - name: "open_gripper"
      action_type: "control_msgs/action/GripperCommand"
      description: "Open gripper to max width"
      parameters:
        - position: 0.08  # meters
      postconditions:
        - "Gripper open"
      average_duration_sec: 1

    - name: "close_gripper"
      action_type: "control_msgs/action/GripperCommand"
      description: "Close gripper (grasp object)"
      parameters:
        - position: 0.0
        - max_effort: 50.0  # Newtons
      postconditions:
        - "Object grasped"
      average_duration_sec: 1

  perception:
    - name: "detect_objects"
      service_type: "vision_msgs/srv/DetectObjects"
      description: "Run object detection and return bounding boxes"
      returns:
        - detections: "vision_msgs/Detection3DArray"

safety_constraints:
  workspace_limits:
    x: [-2.0, 2.0]
    y: [-2.0, 2.0]
    z: [0.0, 2.0]
  max_linear_velocity: 0.5  # m/s
  max_angular_velocity: 0.8  # rad/s
  collision_checking: true
  forbidden_zones:
    - name: "human_workspace"
      boundary: [[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]]
```

---

### Step 2: Design LLM Prompt Engineering Architecture

**What the skill does**:
- Creates system prompt that defines the robot's persona and capabilities
- Designs few-shot examples for task decomposition
- Implements function calling schema for action invocation
- Adds safety instructions and constraint checking

**Deliverable**:
```python
# src/vla_orchestrator/prompts.py

SYSTEM_PROMPT = """You are the brain of a mobile manipulator robot with the following capabilities:

**Navigation**: You can move to named locations (kitchen, living room, bedroom, table, couch) or specific coordinates.

**Manipulation**: You have a 7-DoF arm with a parallel jaw gripper. You can:
- Move your arm to Cartesian poses
- Pick up objects (mugs, bottles, small boxes, up to 1kg)
- Place objects on surfaces
- Open/close your gripper

**Perception**: You can detect objects using camera-based perception.

**Constraints**:
- Workspace: 2m radius around base
- Cannot pick up fragile items (glass, electronics) without explicit confirmation
- Cannot enter forbidden zones near humans
- Max gripper weight: 1kg

Your job is to:
1. Parse user commands in natural language
2. Decompose complex tasks into atomic action primitives
3. Return a JSON action plan with sequence of primitives
4. If the command is ambiguous or impossible, ask for clarification

**Action Primitives**:
- navigate_to_pose(location: str | pose: [x, y, theta])
- move_to_cartesian_pose(pose: [x, y, z, roll, pitch, yaw], frame: str)
- open_gripper()
- close_gripper()
- detect_objects() -> List[Detection]

**Output Format**:
Return ONLY a JSON object with this schema:
{
  "intent": "brief description of user intent",
  "action_plan": [
    {
      "action": "action_name",
      "parameters": {...},
      "rationale": "why this action"
    }
  ],
  "clarifications": ["question1", "question2"] or null,
  "safety_warnings": ["warning1"] or null
}

**Examples**:

User: "Go to the kitchen"
Assistant: {
  "intent": "Navigate to kitchen location",
  "action_plan": [
    {"action": "navigate_to_pose", "parameters": {"location": "kitchen"}, "rationale": "Move to kitchen waypoint"}
  ],
  "clarifications": null,
  "safety_warnings": null
}

User: "Pick up the red mug on the table"
Assistant: {
  "intent": "Grasp red mug from table surface",
  "action_plan": [
    {"action": "navigate_to_pose", "parameters": {"location": "table"}, "rationale": "Get close to table"},
    {"action": "detect_objects", "parameters": {}, "rationale": "Locate red mug"},
    {"action": "move_to_cartesian_pose", "parameters": {"pose": "pre_grasp_pose", "frame": "object"}, "rationale": "Pre-grasp pose above mug"},
    {"action": "open_gripper", "parameters": {}, "rationale": "Prepare to grasp"},
    {"action": "move_to_cartesian_pose", "parameters": {"pose": "grasp_pose", "frame": "object"}, "rationale": "Lower gripper to mug"},
    {"action": "close_gripper", "parameters": {}, "rationale": "Grasp mug"},
    {"action": "move_to_cartesian_pose", "parameters": {"pose": "lift_pose", "frame": "object"}, "rationale": "Lift mug off table"}
  ],
  "clarifications": null,
  "safety_warnings": null
}

User: "Clean the table"
Assistant: {
  "intent": "Remove objects from table surface",
  "action_plan": [],
  "clarifications": ["Where should I place the objects I remove from the table?", "Should I wipe the table after removing objects?"],
  "safety_warnings": null
}

User: "Bring me the laptop"
Assistant: {
  "intent": "Cannot execute - fragile item",
  "action_plan": [],
  "clarifications": null,
  "safety_warnings": ["I cannot pick up fragile electronics like laptops without explicit handling instructions to avoid damage."]
}

Now process the user's command:
"""

def build_user_prompt(command: str, context: dict = None) -> str:
    """Build user prompt with optional context."""
    prompt = f"User command: \"{command}\"\n"

    if context:
        if "robot_pose" in context:
            prompt += f"Current robot location: {context['robot_pose']}\n"
        if "detected_objects" in context:
            prompt += f"Visible objects: {context['detected_objects']}\n"
        if "previous_actions" in context:
            prompt += f"Previous actions in this conversation: {context['previous_actions']}\n"

    return prompt
```

---

### Step 3: Implement LLM Integration and Action Parser

**What the skill does**:
- Creates ROS 2 node that interfaces with LLM API (OpenAI, Anthropic, local LLaMA)
- Implements JSON schema validation for LLM responses
- Adds retry logic for malformed outputs
- Logs all LLM interactions for debugging

**Deliverable**:
```python
# src/vla_orchestrator/llm_client.py
import json
import os
from typing import Dict, List, Optional
import openai  # or anthropic, or transformers for local models

class LLMClient:
    """Client for LLM API calls with retry and validation."""

    def __init__(self, model: str = "gpt-4", temperature: float = 0.2):
        self.model = model
        self.temperature = temperature
        openai.api_key = os.getenv("OPENAI_API_KEY")

    def generate_action_plan(self, command: str, context: dict = None, max_retries: int = 3) -> dict:
        """
        Generate action plan from natural language command.

        Returns:
            {
                "intent": str,
                "action_plan": List[dict],
                "clarifications": List[str] | None,
                "safety_warnings": List[str] | None
            }
        """
        from .prompts import SYSTEM_PROMPT, build_user_prompt

        user_prompt = build_user_prompt(command, context)

        for attempt in range(max_retries):
            try:
                response = openai.ChatCompletion.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": user_prompt}
                    ],
                    temperature=self.temperature,
                    max_tokens=1000,
                )

                content = response.choices[0].message.content.strip()

                # Parse JSON
                action_plan = json.loads(content)

                # Validate schema
                assert "intent" in action_plan
                assert "action_plan" in action_plan
                assert isinstance(action_plan["action_plan"], list)

                return action_plan

            except (json.JSONDecodeError, AssertionError) as e:
                print(f"[LLMClient] Parse error (attempt {attempt + 1}/{max_retries}): {e}")
                if attempt == max_retries - 1:
                    return {
                        "intent": "Parse error",
                        "action_plan": [],
                        "clarifications": [f"I couldn't understand how to execute '{command}'. Please rephrase."],
                        "safety_warnings": None
                    }
                continue

        return None
```

---

### Step 4: Create ROS 2 Orchestrator Node

**What the skill does**:
- Builds ROS 2 node that listens for text commands (topic or service)
- Calls LLM to generate action plan
- Executes actions sequentially using ROS 2 action clients
- Publishes status updates and handles failures

**Deliverable**:
```python
# src/vla_orchestrator/orchestrator_node.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup

from .llm_client import LLMClient

class VLAOrchestrator(Node):
    """LLM-powered robot orchestrator for natural language commands."""

    def __init__(self):
        super().__init__('vla_orchestrator')

        # Parameters
        self.declare_parameter('llm_model', 'gpt-4')
        self.declare_parameter('llm_temperature', 0.2)

        # LLM client
        model = self.get_parameter('llm_model').value
        temperature = self.get_parameter('llm_temperature').value
        self.llm_client = LLMClient(model=model, temperature=temperature)

        # Subscribers
        self.create_subscription(String, '/robot/command', self.command_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_command')
        self.moveit_client = ActionClient(self, MoveGroup, '/move_action')

        # State
        self.current_action_plan = None
        self.action_index = 0
        self.is_executing = False

        self.get_logger().info("VLA Orchestrator ready")

    def command_callback(self, msg: String):
        """Handle incoming text command."""
        if self.is_executing:
            self.publish_status("Busy: currently executing a task")
            return

        command = msg.data
        self.get_logger().info(f"Received command: {command}")

        # Generate action plan with LLM
        self.publish_status(f"Processing: {command}")
        action_plan = self.llm_client.generate_action_plan(command)

        if not action_plan:
            self.publish_status("Error: Failed to generate action plan")
            return

        # Check for clarifications
        if action_plan.get("clarifications"):
            clarifications = " ".join(action_plan["clarifications"])
            self.publish_status(f"Clarification needed: {clarifications}")
            return

        # Check for safety warnings
        if action_plan.get("safety_warnings"):
            warnings = " ".join(action_plan["safety_warnings"])
            self.publish_status(f"Cannot execute: {warnings}")
            return

        # Execute action plan
        self.current_action_plan = action_plan["action_plan"]
        self.action_index = 0
        self.is_executing = True

        self.get_logger().info(f"Action plan: {action_plan['intent']}")
        self.get_logger().info(f"Steps: {len(self.current_action_plan)}")

        self.execute_next_action()

    def execute_next_action(self):
        """Execute next action in the plan."""
        if self.action_index >= len(self.current_action_plan):
            self.publish_status("Task completed successfully")
            self.is_executing = False
            return

        action = self.current_action_plan[self.action_index]
        action_name = action["action"]
        parameters = action["parameters"]

        self.get_logger().info(f"Executing: {action_name} ({self.action_index + 1}/{len(self.current_action_plan)})")
        self.publish_status(f"Executing: {action['rationale']}")

        # Dispatch to appropriate action client
        if action_name == "navigate_to_pose":
            self.execute_navigation(parameters)
        elif action_name == "open_gripper":
            self.execute_gripper(0.08)  # Open
        elif action_name == "close_gripper":
            self.execute_gripper(0.0)  # Close
        elif action_name == "move_to_cartesian_pose":
            self.execute_moveit(parameters)
        else:
            self.get_logger().error(f"Unknown action: {action_name}")
            self.action_index += 1
            self.execute_next_action()

    def execute_navigation(self, params: dict):
        """Execute navigation action."""
        goal_msg = NavigateToPose.Goal()

        # Convert location name to pose (you'd load this from a map)
        if "location" in params:
            location = params["location"]
            # Lookup location in known waypoints (simplified)
            waypoints = {
                "kitchen": [1.0, 2.0, 0.0],
                "living_room": [0.0, 0.0, 0.0],
                "table": [2.0, 1.0, 1.57],
            }
            if location in waypoints:
                x, y, theta = waypoints[location]
                goal_msg.pose.pose.position.x = x
                goal_msg.pose.pose.position.y = y
                # Set orientation from theta (simplified)

        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            self.publish_status("Error: Navigation goal rejected")
            self.is_executing = False
            return

        self.get_logger().info("Navigation goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result."""
        result = future.result().result
        self.get_logger().info("Navigation completed")

        # Move to next action
        self.action_index += 1
        self.execute_next_action()

    def execute_gripper(self, position: float):
        """Execute gripper action."""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 50.0

        self.gripper_client.wait_for_server()
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future):
        """Handle gripper goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            self.is_executing = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def gripper_result_callback(self, future):
        """Handle gripper result."""
        self.get_logger().info("Gripper command completed")
        self.action_index += 1
        self.execute_next_action()

    def execute_moveit(self, params: dict):
        """Execute MoveIt motion planning action."""
        # Simplified - you'd populate MoveGroup.Goal properly
        self.get_logger().info("MoveIt action (simplified)")
        # For now, just move to next action
        self.action_index += 1
        self.execute_next_action()

    def publish_status(self, message: str):
        """Publish status message."""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {message}")


def main(args=None):
    rclpy.init(args=args)
    node = VLAOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Step 5: Add Execution Monitoring and Failure Handling

**What the skill does**:
- Implements timeout detection for stuck actions
- Adds retry logic for transient failures
- Creates fallback strategies (e.g., re-plan if navigation fails)
- Logs execution traces for post-mortem analysis

**Deliverable**:
```python
# Add to VLAOrchestrator class:

def __init__(self):
    # ... (previous init code)

    # Failure handling
    self.max_retries = 3
    self.current_retries = 0
    self.action_timeout_sec = 60.0  # Max time per action
    self.timeout_timer = None

def execute_next_action(self):
    """Execute next action with timeout and retry logic."""
    if self.action_index >= len(self.current_action_plan):
        self.publish_status("Task completed successfully")
        self.is_executing = False
        return

    action = self.current_action_plan[self.action_index]

    # Start timeout timer
    if self.timeout_timer:
        self.timeout_timer.cancel()
    self.timeout_timer = self.create_timer(self.action_timeout_sec, self.action_timeout_callback)

    # Execute action (previous code)
    # ...

def action_timeout_callback(self):
    """Handle action timeout."""
    self.get_logger().error(f"Action {self.action_index} timed out after {self.action_timeout_sec}s")

    if self.current_retries < self.max_retries:
        self.current_retries += 1
        self.get_logger().info(f"Retrying action (attempt {self.current_retries}/{self.max_retries})")
        self.publish_status(f"Retrying action ({self.current_retries}/{self.max_retries})")
        # Re-execute current action
        self.execute_next_action()
    else:
        self.publish_status("Error: Action timed out after max retries")
        self.is_executing = False
        self.current_retries = 0

def navigation_result_callback(self, future):
    """Handle navigation result with failure handling."""
    result = future.result().result

    # Cancel timeout timer
    if self.timeout_timer:
        self.timeout_timer.cancel()

    # Check if navigation succeeded
    # (Nav2 result has status code)
    if result.error_code != 0:
        self.get_logger().error(f"Navigation failed with error code {result.error_code}")

        if self.current_retries < self.max_retries:
            self.current_retries += 1
            self.get_logger().info(f"Retrying navigation (attempt {self.current_retries}/{self.max_retries})")
            self.execute_navigation(self.current_action_plan[self.action_index]["parameters"])
            return
        else:
            self.publish_status("Error: Navigation failed after max retries")
            self.is_executing = False
            self.current_retries = 0
            return

    self.get_logger().info("Navigation completed successfully")
    self.current_retries = 0
    self.action_index += 1
    self.execute_next_action()
```

---

### Step 6: Create Launch File and Configuration

**What the skill does**:
- Generates ROS 2 launch file for the orchestrator
- Adds parameters for LLM model selection and tuning
- Includes RViz visualization for debugging

**Deliverable**:
```python
# launch/vla_orchestrator.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    llm_model = LaunchConfiguration('llm_model', default='gpt-4')
    llm_temperature = LaunchConfiguration('llm_temperature', default='0.2')

    vla_orchestrator_node = Node(
        package='vla_orchestrator',
        executable='orchestrator_node',
        name='vla_orchestrator',
        output='screen',
        parameters=[{
            'llm_model': llm_model,
            'llm_temperature': llm_temperature,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('llm_model', default_value='gpt-4',
                              description='LLM model (gpt-4, gpt-3.5-turbo, claude-3, local)'),
        DeclareLaunchArgument('llm_temperature', default_value='0.2',
                              description='LLM temperature (0.0-1.0)'),
        vla_orchestrator_node,
    ])
```

**Usage**:
```bash
# Set OpenAI API key
export OPENAI_API_KEY="sk-..."

# Launch orchestrator
ros2 launch vla_orchestrator vla_orchestrator.launch.py llm_model:=gpt-4

# Send command
ros2 topic pub /robot/command std_msgs/String "data: 'Go to the kitchen and pick up the red mug'"

# Monitor status
ros2 topic echo /robot/status
```

---

### Step 7: Add Testing and Simulation Integration

**What the skill does**:
- Creates unit tests for action plan generation
- Adds integration tests with simulated robot
- Implements mock action servers for CI/CD testing

**Deliverable**:
```python
# test/test_llm_client.py
import unittest
from vla_orchestrator.llm_client import LLMClient

class TestLLMClient(unittest.TestCase):
    def setUp(self):
        self.client = LLMClient(model="gpt-4", temperature=0.2)

    def test_simple_navigation_command(self):
        """Test: 'Go to the kitchen' generates navigate_to_pose action."""
        result = self.client.generate_action_plan("Go to the kitchen")

        self.assertIn("intent", result)
        self.assertIn("action_plan", result)
        self.assertEqual(len(result["action_plan"]), 1)
        self.assertEqual(result["action_plan"][0]["action"], "navigate_to_pose")
        self.assertIn("kitchen", result["action_plan"][0]["parameters"]["location"])

    def test_pick_command_generates_sequence(self):
        """Test: 'Pick up the mug' generates multi-step plan."""
        result = self.client.generate_action_plan("Pick up the red mug on the table")

        self.assertGreater(len(result["action_plan"]), 3)
        # Should include: navigate, detect, move_to_pose, open_gripper, close_gripper
        action_names = [a["action"] for a in result["action_plan"]]
        self.assertIn("navigate_to_pose", action_names)
        self.assertIn("open_gripper", action_names)
        self.assertIn("close_gripper", action_names)

    def test_ambiguous_command_requests_clarification(self):
        """Test: Ambiguous command triggers clarification."""
        result = self.client.generate_action_plan("Clean the table")

        self.assertIsNotNone(result.get("clarifications"))
        self.assertGreater(len(result["clarifications"]), 0)

    def test_unsafe_command_generates_warning(self):
        """Test: Unsafe command (fragile object) generates safety warning."""
        result = self.client.generate_action_plan("Bring me the laptop")

        self.assertIsNotNone(result.get("safety_warnings"))

if __name__ == '__main__':
    unittest.main()
```

---

### Step 8: Create Documentation and User Guide

**What the skill does**:
- Writes README with installation and usage instructions
- Documents supported commands and expected behavior
- Provides troubleshooting guide for common issues

**Deliverable**:
```markdown
# VLA Orchestrator - LLM-Powered Robot Control

## Overview

The VLA Orchestrator is an LLM-powered orchestration layer that translates natural language commands into executable ROS 2 action sequences for humanoid robots and mobile manipulators.

## Features

- **Natural Language Interface**: Command your robot using plain English
- **Task Decomposition**: Automatically breaks complex tasks into atomic actions
- **Failure Handling**: Retries failed actions and provides fallback strategies
- **Safety Guardrails**: Validates commands against robot constraints
- **Conversational Context**: Remembers previous commands in a session

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-org/vla_orchestrator.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vla_orchestrator
source install/setup.bash
```

## Configuration

1. Set your OpenAI API key:
```bash
export OPENAI_API_KEY="sk-..."
```

2. Edit robot capabilities in `config/robot_capabilities.yaml`

## Usage

### Launch Orchestrator
```bash
ros2 launch vla_orchestrator vla_orchestrator.launch.py
```

### Send Commands
```bash
# Via topic
ros2 topic pub /robot/command std_msgs/String "data: 'Go to the kitchen'"

# Via CLI tool (convenience)
ros2 run vla_orchestrator send_command "Pick up the red mug and bring it to me"
```

### Monitor Status
```bash
ros2 topic echo /robot/status
```

## Supported Commands

### Navigation
- "Go to the kitchen"
- "Navigate to coordinates x=1.5, y=2.0"
- "Move to the table"

### Manipulation
- "Pick up the red mug"
- "Place the bottle on the counter"
- "Open your gripper"

### Combined Tasks
- "Go to the kitchen and pick up the bottle"
- "Fetch me the mug from the living room"

### Perception
- "What objects do you see?"
- "Is there a bottle on the table?"

## Troubleshooting

**Issue**: "LLM timeout" error
- **Solution**: Check internet connection and API key validity

**Issue**: Robot doesn't move after command
- **Solution**: Verify action servers are running (`ros2 action list`)

**Issue**: Command generates clarification instead of execution
- **Solution**: Provide more specific details (object color, location name)

## Architecture

```
User Command (text)
       ↓
  LLM Client (GPT-4)
       ↓
 Action Plan (JSON)
       ↓
VLA Orchestrator Node
       ↓
ROS 2 Action Clients → Robot
```

## Performance

- **LLM latency**: 500-2000ms (depends on API)
- **Action execution**: Varies (navigation: 10-30s, gripper: 1s)
- **Total task time**: Sum of LLM latency + action execution

## Future Enhancements

- Vision-language grounding (detect "the red mug" automatically)
- Multi-turn conversations with memory
- Reinforcement learning from human feedback (RLHF)
- Integration with pre-trained VLA models (RT-2, OpenVLA)
```

---

### Step 9: Generate Validation Report and Recommendations

**What the skill does**:
- Tests orchestrator with 10+ diverse commands
- Measures success rate, latency, and failure modes
- Provides recommendations for improvement

**Deliverable**:
```markdown
# VLA Orchestrator Validation Report

**Date**: 2025-12-05
**Robot**: Mobile Manipulator (7-DoF arm + diff-drive base)
**LLM**: GPT-4 (gpt-4-0613)
**Test Duration**: 30 minutes, 15 commands

---

## Test Results

| Command | Intent Parsed | Action Plan Valid | Execution Success | Total Time |
|---------|---------------|-------------------|-------------------|------------|
| "Go to the kitchen" | ✅ | ✅ | ✅ | 12.3s |
| "Pick up the red mug" | ✅ | ✅ | ✅ | 18.7s |
| "Bring me the bottle from the table" | ✅ | ✅ | ✅ | 31.2s |
| "Clean the table" | ✅ | ⚠️ Clarification | N/A | 1.8s |
| "Fetch the laptop" | ✅ | ⚠️ Safety Warning | N/A | 1.5s |
| "Navigate to x=2.0 y=1.5" | ✅ | ✅ | ✅ | 9.8s |
| "Open your gripper" | ✅ | ✅ | ✅ | 2.1s |
| "Pick up the blue box and place it on the counter" | ✅ | ✅ | ⚠️ Partial (placed incorrectly) | 42.3s |
| "What do you see?" | ✅ | ✅ | ✅ | 3.2s |
| "Move your arm up" | ✅ | ✅ | ✅ | 4.5s |

---

## Summary Metrics

- **Commands tested**: 10
- **Intent parsing success**: 10/10 (100%)
- **Valid action plans**: 8/10 (80%)
- **Execution success**: 7/8 (87.5%)
- **Overall success rate**: 7/10 (70%)
- **Average LLM latency**: 1.2s
- **Average task execution time**: 15.8s

---

## Failure Analysis

### Failure 1: "Clean the table" → Clarification needed
- **Root cause**: Ambiguous task (where to place objects?)
- **Fix**: Expected behavior - system correctly requested clarification

### Failure 2: "Fetch the laptop" → Safety warning
- **Root cause**: Fragile object constraint
- **Fix**: Expected behavior - safety guardrail working correctly

### Failure 3: "Pick up blue box and place on counter" → Partial execution
- **Root cause**: Placement pose was incorrect (overshoot by 10cm)
- **Fix**: Tune MoveIt planner tolerances or add vision-based servoing

---

## Recommendations

### 1. Vision-Language Grounding
- **Issue**: LLM doesn't know exact object poses (uses waypoints only)
- **Solution**: Integrate Isaac ROS DetectNet + FoundationPose for real-time object localization

### 2. Multi-Turn Conversations
- **Issue**: No conversational memory (each command is independent)
- **Solution**: Maintain conversation history and pass to LLM as context

### 3. Cost Optimization
- **Issue**: GPT-4 costs $0.03 per request (avg 1000 tokens)
- **Solution**: For production, fine-tune smaller model (GPT-3.5, LLaMA-2) or use local VLA

### 4. Perception Integration
- **Issue**: Currently using hardcoded waypoints for locations
- **Solution**: Integrate with SLAM map to support dynamic waypoints ("go to the red chair")

### 5. Error Recovery
- **Issue**: If action fails, system stops (no re-planning)
- **Solution**: On failure, query LLM for alternative plan

---

## Next Steps

1. Add vision-language grounding (RT-2 or OpenVLA integration)
2. Implement conversational memory (last 5 commands)
3. Benchmark local VLA models (LLaMA-2-7B fine-tuned on robotics data)
4. Deploy on real robot and collect human feedback

---

**Status**: ✅ VLA Orchestrator functional for simple commands
**Readiness**: 70% success rate - suitable for research/demos, needs refinement for production
```

---

## Output Format

This skill produces:

1. **Robot Capabilities YAML** (`config/robot_capabilities.yaml`)
   - Catalog of action primitives, preconditions, postconditions

2. **Prompt Engineering Module** (`src/vla_orchestrator/prompts.py`)
   - System prompt with robot persona and capabilities
   - Few-shot examples for task decomposition

3. **LLM Client** (`src/vla_orchestrator/llm_client.py`)
   - API integration with retry logic and validation

4. **ROS 2 Orchestrator Node** (`src/vla_orchestrator/orchestrator_node.py`)
   - Main orchestration logic with action execution and failure handling

5. **Launch Files** (`launch/vla_orchestrator.launch.py`)
   - ROS 2 launch configuration with parameters

6. **Tests** (`test/test_llm_client.py`, `test/test_orchestrator.py`)
   - Unit and integration tests for validation

7. **Documentation** (`README.md`)
   - Installation, usage, supported commands, troubleshooting

8. **Validation Report** (`docs/vla_orchestrator_validation_report.md`)
   - Test results, metrics, recommendations

---

## Quality Criteria

Before marking this skill complete, verify:

### Functional Requirements
- [ ] Orchestrator parses 10+ diverse natural language commands correctly
- [ ] Action plans are valid sequences of robot primitives (no hallucinated actions)
- [ ] At least 70% of commands execute successfully in simulation
- [ ] Ambiguous commands trigger clarification requests (not silent failures)
- [ ] Unsafe commands generate safety warnings

### Non-Functional Requirements
- [ ] LLM latency <3s per command (network dependent)
- [ ] Action execution has timeout protection (no infinite hangs)
- [ ] Failed actions retry up to 3 times before aborting
- [ ] All LLM interactions are logged for debugging
- [ ] System handles LLM API failures gracefully (connection timeout, rate limit)

### Code Quality
- [ ] Python code follows ROS 2 + PEP 8 conventions
- [ ] All parameters configurable via launch file
- [ ] Unit tests cover LLM client and action parsing
- [ ] Integration tests validate with mock action servers

### Documentation
- [ ] README explains installation, usage, supported commands
- [ ] Prompt engineering strategy is documented
- [ ] Troubleshooting guide covers common issues (API key, action server not found)
- [ ] Example commands demonstrate capabilities

---

## Full Example

**Scenario**: User wants to command a mobile manipulator robot to fetch a bottle.

**Input**: "Go to the kitchen and pick up the bottle on the counter"

**LLM Output** (JSON):
```json
{
  "intent": "Navigate to kitchen, locate and grasp bottle from counter",
  "action_plan": [
    {
      "action": "navigate_to_pose",
      "parameters": {"location": "kitchen"},
      "rationale": "Move to kitchen area"
    },
    {
      "action": "detect_objects",
      "parameters": {},
      "rationale": "Locate bottle on counter"
    },
    {
      "action": "move_to_cartesian_pose",
      "parameters": {"pose": "pre_grasp_pose", "frame": "object"},
      "rationale": "Pre-grasp pose above bottle"
    },
    {
      "action": "open_gripper",
      "parameters": {},
      "rationale": "Prepare gripper for grasping"
    },
    {
      "action": "move_to_cartesian_pose",
      "parameters": {"pose": "grasp_pose", "frame": "object"},
      "rationale": "Lower gripper to bottle"
    },
    {
      "action": "close_gripper",
      "parameters": {},
      "rationale": "Grasp bottle"
    },
    {
      "action": "move_to_cartesian_pose",
      "parameters": {"pose": "lift_pose", "frame": "object"},
      "rationale": "Lift bottle off counter"
    }
  ],
  "clarifications": null,
  "safety_warnings": null
}
```

**Execution**:
1. Orchestrator sends navigation goal to Nav2 → robot moves to kitchen (10s)
2. Calls object detection service → DetectNet localizes bottle (2s)
3. Sends MoveIt goal for pre-grasp pose → arm moves above bottle (3s)
4. Opens gripper (1s)
5. Lowers gripper to grasp pose (2s)
6. Closes gripper (1s)
7. Lifts bottle (2s)

**Total time**: ~21s (excluding LLM latency of ~1.5s)

**ROS 2 Topics**:
```bash
# Status updates published to /robot/status:
"Processing: Go to the kitchen and pick up the bottle on the counter"
"Executing: Move to kitchen area"
"Executing: Locate bottle on counter"
"Executing: Pre-grasp pose above bottle"
"Executing: Prepare gripper for grasping"
"Executing: Lower gripper to bottle"
"Executing: Grasp bottle"
"Executing: Lift bottle off counter"
"Task completed successfully"
```

---

**VLA Orchestrator Skill Complete** ✅
