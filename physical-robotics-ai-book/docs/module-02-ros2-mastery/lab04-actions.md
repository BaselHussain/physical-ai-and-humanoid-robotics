---
title: Lab 4 - Actions
sidebar_label: Lab 4 - Actions
sidebar_position: 7
description: Implement long-running goals with feedback using ROS 2 actions, create MoveArm.action with goal/feedback/result fields, implement action server and client nodes
---

# Lab 4: Actions

## Overview

In this lab, you'll learn to implement long-running goals with feedback using ROS 2 actions. You'll create a `MoveArm.action` with goal, feedback, and result fields, implement action server and client nodes, and understand when to use actions versus services or topics. Actions are essential for operations that take time to complete and need to provide feedback during execution.

**Duration**: 2 hours

**Learning Objectives**:
- ✅ Define custom action types using `.action` syntax
- ✅ Implement action servers that handle long-running goals with feedback
- ✅ Create action clients that send goals and monitor progress
- ✅ Validate action communication with `ros2 action` commands
- ✅ Implement MoveArm action with JointTrajectory goal, progress feedback, and execution result

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **Completed Lab 1** - Talker/Listener basics
✅ **Completed Lab 2** - Custom messages
✅ **Completed Lab 3** - Services
✅ **ROS 2 Humble installed** with all standard packages
✅ **Basic understanding** of ROS 2 nodes, topics, services, and custom messages
✅ **Python programming skills** for implementing action usage

---

## Step 1: Understanding ROS 2 Actions

### What are Actions?

**ROS 2 actions** provide a communication pattern for long-running tasks that require:
- **Goals**: Requests for the server to perform an operation
- **Feedback**: Periodic updates on the progress of the operation
- **Results**: Final outcome of the operation

### Action vs Services vs Topics

| Aspect | Actions | Services | Topics |
|--------|---------|----------|---------|
| **Duration** | Long-running tasks | Short synchronous calls | Continuous data |
| **Feedback** | Yes (progress updates) | No (single response) | No (data only) |
| **Cancellation** | Yes | No | No |
| **Use Case** | Navigation, manipulation, calibration | Configuration, queries | Sensor data, state |

### Action Definition Syntax

ROS 2 uses a three-part syntax for action definitions:

```
# Goal fields (top section)
float64 target_value
string description

---
# Result fields (middle section)
bool success
string message
float64 final_value

---
# Feedback fields (bottom section)
float64 current_value
float64 progress
string status
```

### MoveArm Action Requirements

Based on our spec, we need a `MoveArm.action` with:
- **Goal**: `joint_trajectory` (trajectory_msgs/JointTrajectory), `tolerance` (float64)
- **Result**: `success` (bool), `final_error` (float64), `execution_time` (builtin_interfaces/Time)
- **Feedback**: `current_joint_positions` (float64[]), `time_remaining` (builtin_interfaces/Duration), `progress` (float32)

---

## Step 2: Create Actions Package

First, let's create a package for our actions:

```bash
cd ~/ros2_ws/src
rm -rf lab04_actions  # Remove if exists
ros2 pkg create --build-type ament_cmake lab04_actions
```

---

## Step 3: Define the MoveArm Action

Create the actions directory:
```bash
mkdir -p ~/ros2_ws/src/lab04_actions/action
```

Create the MoveArm action definition:
```bash
touch ~/ros2_ws/src/lab04_actions/action/MoveArm.action
```

Add the following content to `~/ros2_ws/src/lab04_actions/action/MoveArm.action`:

```
# MoveArm.action: Move a robot arm along a trajectory
# Goal: Joint trajectory to follow with tolerance
# Result: Success flag, final error, execution time
# Feedback: Current positions, time remaining, progress percentage

# Goal fields
trajectory_msgs/JointTrajectory joint_trajectory
float64 tolerance

---
# Result fields
bool success
float64 final_error
builtin_interfaces/Time execution_time

---
# Feedback fields
float64[] current_joint_positions
builtin_interfaces/Duration time_remaining
float32 progress
```

---

## Step 4: Update Package Configuration for Actions

Edit the CMakeLists.txt:

Edit `~/ros2_ws/src/lab04_actions/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(lab04_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define action files
set(action_files
  "action/MoveArm.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES std_msgs trajectory_msgs builtin_interfaces
  ADD_LINTER_TESTS
)

ament_package()
```

Update the package.xml:

Edit `~/ros2_ws/src/lab04_actions/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lab04_actions</name>
  <version>0.0.0</version>
  <description>ROS 2 actions implementation with MoveArm action</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>trajectory_msgs</build_depend>
  <build_depend>builtin_interfaces</build_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>builtin_interfaces</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Step 5: Build the Actions Package

```bash
cd ~/ros2_ws
colcon build --packages-select lab04_actions
source ~/ros2_ws/install/setup.bash
```

---

## Step 6: Verify Action Generation

Check that the MoveArm action was generated correctly:

```bash
ros2 interface show lab04_actions/action/MoveArm
```

**Expected Output**:
```
# Goal:
trajectory_msgs/JointTrajectory joint_trajectory
float64 tolerance

# Result:
bool success
float64 final_error
builtin_interfaces/Time execution_time

# Feedback:
float64[] current_joint_positions
builtin_interfaces/Duration time_remaining
float32 progress
```

You can also list all available actions:
```bash
ros2 interface list | grep MoveArm
```

---

## Step 7: Create Action Server

First, let's create a Python package for our action server:

```bash
mkdir -p ~/ros2_ws/src/lab04_actions/lab04_actions
```

Create the action server file:
```bash
touch ~/ros2_ws/src/lab04_actions/lab04_actions/action_server.py
```

Add the following code to `~/ros2_ws/src/lab04_actions/lab04_actions/action_server.py`:

```python
#!/usr/bin/env python3
"""
Action server - implements MoveArm action to control robot arm movement
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math

# Import our custom action
from lab04_actions.action import MoveArm
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveArmActionServer(Node):

    def __init__(self):
        super().__init__('move_arm_action_server')

        # Create the action server
        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('MoveArm action server started')

        # Simulate current joint positions
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 DOF arm
        self._goal_handle = None

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')

        # Check if goal is valid
        if len(goal_request.joint_trajectory.points) == 0:
            self.get_logger().info('Rejecting goal: no trajectory points')
            return GoalResponse.REJECT

        # Check if we can accept the goal (e.g., not already executing)
        if self._goal_handle is not None and self._goal_handle.is_active:
            self.get_logger().info('Rejecting goal: another goal is active')
            return GoalResponse.REJECT

        self.get_logger().info('Accepting goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute the action goal - simulates arm movement along trajectory
        """
        self.get_logger().info('Executing goal...')
        self._goal_handle = goal_handle

        # Get the goal
        goal = goal_handle.request
        trajectory = goal.joint_trajectory
        tolerance = goal.tolerance

        # Initialize feedback
        feedback_msg = MoveArm.Feedback()
        feedback_msg.current_joint_positions = self.current_joint_positions[:]
        feedback_msg.time_remaining = Duration(sec=0, nanosec=0)
        feedback_msg.progress = 0.0

        # Start time for execution timing
        start_time = self.get_clock().now()

        # Execute the trajectory point by point
        total_points = len(trajectory.points)
        for i, point in enumerate(trajectory.points):
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = MoveArm.Result()
                result.success = False
                result.final_error = -1.0  # Canceled
                result.execution_time = (self.get_clock().now() - start_time).to_msg()
                return result

            # Update current positions to the target point
            if len(point.positions) <= len(self.current_joint_positions):
                for j, pos in enumerate(point.positions):
                    self.current_joint_positions[j] = pos
            else:
                # If more positions than we can handle, use as many as possible
                for j in range(len(self.current_joint_positions)):
                    self.current_joint_positions[j] = point.positions[j]

            # Calculate progress (0.0 to 1.0)
            progress = float(i + 1) / float(total_points)
            feedback_msg.progress = progress
            feedback_msg.current_joint_positions = self.current_joint_positions[:]

            # Calculate estimated time remaining
            elapsed_time = self.get_clock().now() - start_time
            estimated_total_time = elapsed_time.nanoseconds / progress if progress > 0 else 0
            remaining_nanoseconds = estimated_total_time - elapsed_time.nanoseconds
            feedback_msg.time_remaining.sec = int(remaining_nanoseconds // 1e9)
            feedback_msg.time_remaining.nanosec = int(remaining_nanoseconds % 1e9)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Published feedback: {progress:.2%} complete')

            # Simulate movement time based on time_from_start
            if i > 0:
                prev_point_time = trajectory.points[i-1].time_from_start
            else:
                prev_point_time = Duration(sec=0, nanosec=0)

            time_diff = point.time_from_start.sec - prev_point_time.sec + (point.time_from_start.nanosec - prev_point_time.nanosec) / 1e9
            time.sleep(max(0.1, time_diff))  # Minimum sleep to allow other callbacks to run

        # Check if final position is within tolerance
        # For this simulation, we'll consider success if we reached the last point
        final_error = 0.0  # In simulation, we assume perfect execution
        success = final_error <= tolerance

        # Create result
        result = MoveArm.Result()
        result.success = success
        result.final_error = final_error
        result.execution_time = (self.get_clock().now() - start_time).to_msg()

        # Log the result
        status = "SUCCEEDED" if success else "FAILED"
        self.get_logger().info(f'Goal {status} with final error: {final_error:.3f}, tolerance: {tolerance:.3f}')

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = MoveArmActionServer()

    # Use MultiThreadedExecutor to handle callbacks in separate threads
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        action_server.get_logger().info('Action server stopped')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 8: Create Action Client

Create the action client file:
```bash
touch ~/ros2_ws/src/lab04_actions/lab04_actions/action_client.py
```

Add the following code to `~/ros2_ws/src/lab04_actions/lab04_actions/action_client.py`:

```python
#!/usr/bin/env python3
"""
Action client - sends MoveArm goals and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

# Import our custom action
from lab04_actions.action import MoveArm
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class MoveArmActionClient(Node):

    def __init__(self):
        super().__init__('move_arm_action_client')

        # Create the action client
        self._action_client = ActionClient(self, MoveArm, 'move_arm')

        self.get_logger().info('MoveArm action client created')

    def send_goal(self, joint_trajectory, tolerance=0.01):
        """
        Send a goal to the MoveArm action server
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = MoveArm.Goal()
        goal_msg.joint_trajectory = joint_trajectory
        goal_msg.tolerance = tolerance

        self.get_logger().info(f'Sending goal with {len(joint_trajectory.points)} trajectory points')

        # Send the goal and get a future for the result
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Add a callback for when the result is ready
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._send_goal_future

    def goal_response_callback(self, future):
        """
        Callback when the goal response is received
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback when the result is received
        """
        result = future.result().result
        self.get_logger().info(f'Result received: success={result.success}, final_error={result.final_error:.3f}')
        self.get_logger().info(f'Execution time: {result.execution_time.sec}.{result.execution_time.nanosec:09d}s')

    def feedback_callback(self, feedback_msg):
        """
        Callback when feedback is received
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.progress:.1%} complete, '
            f'time remaining: {feedback.time_remaining.sec}.{feedback.time_remaining.nanosec:09d}s, '
            f'current positions: {[f"{pos:.2f}" for pos in feedback.current_joint_positions[:3]]}...'
        )


def main(args=None):
    rclpy.init(args=args)

    action_client = MoveArmActionClient()

    # Create a simple trajectory with 5 points
    from trajectory_msgs.msg import JointTrajectory
    trajectory = JointTrajectory()
    trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Create trajectory points
    for i in range(5):
        point = JointTrajectoryPoint()

        # Set joint positions (simple pattern)
        point.positions = [
            math.sin(i * 0.5),      # joint1
            math.cos(i * 0.3),      # joint2
            math.sin(i * 0.7),      # joint3
            math.cos(i * 0.4),      # joint4
            math.sin(i * 0.6),      # joint5
            math.cos(i * 0.8)       # joint6
        ]

        # Set velocities (optional)
        point.velocities = [0.0] * 6

        # Set time from start (increasing)
        point.time_from_start = Duration(sec=i+1, nanosec=0)

        trajectory.points.append(point)

    # Send the goal
    future = action_client.send_goal(trajectory, tolerance=0.01)

    # Spin until the result is received
    try:
        rclpy.spin_until_future_complete(action_client, future)
    except KeyboardInterrupt:
        action_client.get_logger().info('Action client interrupted')
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import math  # Import math for the main function
    main()


---

## Step 9: Update Package Configuration for Executables

We need to create a setup.py file to define the executable entry points. Since this is an ament_cmake package, we'll create a separate Python package for the nodes:

```bash
cd ~/ros2_ws/src
rm -rf move_arm_nodes  # Remove if exists
ros2 pkg create --build-type ament_python move_arm_nodes --dependencies rclpy lab04_actions
```

Now let's move our action server and client code to this new package:

```bash
mkdir -p ~/ros2_ws/src/move_arm_nodes/move_arm_nodes
mv ~/ros2_ws/src/lab04_actions/lab04_actions/action_server.py ~/ros2_ws/src/move_arm_nodes/move_arm_nodes/action_server.py
mv ~/ros2_ws/src/lab04_actions/lab04_actions/action_client.py ~/ros2_ws/src/move_arm_nodes/move_arm_nodes/action_client.py
```

Update the setup.py for the move_arm_nodes package:

Edit `~/ros2_ws/src/move_arm_nodes/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'move_arm_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Action server and client nodes for MoveArm action',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_arm_server = move_arm_nodes.action_server:main',
            'move_arm_client = move_arm_nodes.action_client:main',
        ],
    },
)
```

Update the package.xml for the move_arm_nodes package:

Edit `~/ros2_ws/src/move_arm_nodes/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>move_arm_nodes</name>
  <version>0.0.0</version>
  <description>Action server and client nodes for MoveArm action</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>lab04_actions</depend>
  <depend>trajectory_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## Step 10: Build Both Packages

```bash
cd ~/ros2_ws
colcon build --packages-select lab04_actions move_arm_nodes
source ~/ros2_ws/install/setup.bash
```

---

## Step 11: Test the Action

Run the action server in one terminal:

**Terminal 1 - Action Server**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run move_arm_nodes move_arm_server
```

In another terminal, run the action client:

**Terminal 2 - Action Client** (open a new terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run move_arm_nodes move_arm_client
```

You should see the client sending a goal and the server executing it with feedback updates.

---

## Step 12: Use Command Line Tools with Actions

ROS 2 command line tools work with actions too:

```bash
# List all available actions
ros2 action list
```

```bash
# Get info about a specific action
ros2 action info /move_arm
```

```bash
# Send a goal directly from command line (for testing)
ros2 action send_goal /move_arm lab04_actions/action/MoveArm "{joint_trajectory: {joint_names: ['joint1'], points: [{positions: [1.0], time_from_start: {sec: 1, nanosec: 0}}]}, tolerance: 0.01}"
```

```bash
# Check the action type
ros2 action type /move_arm
```

---

## Step 13: Advanced Action Patterns

### Action with Preemption

Actions can be preempted by sending a new goal while one is executing:

```python
def goal_callback(self, goal_request):
    # Accept new goals and preempt current one
    return GoalResponse.ACCEPT

def execute_callback(self, goal_handle):
    # Check if a new goal has preempted this one
    if goal_handle.is_cancel_requested:
        goal_handle.canceled()
        return MoveArm.Result()
```

### Action with Multiple Feedback Types

You can send different types of feedback during execution:

```python
def execute_callback(self, goal_handle):
    # Initial feedback
    feedback_msg = MoveArm.Feedback()
    feedback_msg.progress = 0.0
    feedback_msg.current_joint_positions = self.current_positions[:]
    feedback_msg.time_remaining = Duration(sec=10, nanosec=0)
    goal_handle.publish_feedback(feedback_msg)

    # During execution...
    for step in execution_steps:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return MoveArm.Result()

        # Update feedback based on current state
        feedback_msg.progress = calculate_progress()
        feedback_msg.current_joint_positions = get_current_positions()
        feedback_msg.time_remaining = estimate_remaining_time()
        goal_handle.publish_feedback(feedback_msg)

        time.sleep(0.1)  # Small delay to allow other callbacks

    # Final result
    result = MoveArm.Result()
    result.success = True
    return result
```

---

## Step 14: Actions vs Services vs Topics Comparison

| Aspect | Actions | Services | Topics |
|--------|---------|----------|---------|
| **Duration** | Long-running tasks | Short synchronous calls | Continuous data |
| **Feedback** | Yes (progress updates) | No (single response) | No (data only) |
| **Cancellation** | Yes | No | No |
| **Use Case** | Navigation, manipulation, calibration | Configuration, queries | Sensor data, state |
| **Blocking** | Client can continue while action runs | Client blocks until response | Non-blocking |

### When to Use Actions

Use actions when you need:
- **Long-running operations** that take time to complete
- **Progress feedback** during execution
- **Cancellation capability** for ongoing tasks
- **Complex state management** during execution

### When to Use Services

Use services when you need:
- **Synchronous communication** with guaranteed response
- **Simple request-response** patterns
- **Quick operations** that complete immediately

### When to Use Topics

Use topics when you need:
- **Continuous data streams** (sensors, state)
- **Real-time performance** (low latency)
- **Many-to-many communication** (broadcast)

---

## Step 15: Action Integration Test

Let's create a simple integration test to validate our action. Create a test file:

```bash
mkdir -p ~/ros2_ws/src/move_arm_nodes/test
touch ~/ros2_ws/src/move_arm_nodes/test/test_action_integration.py
```

Add the following code to `~/ros2_ws/src/move_arm_nodes/test/test_action_integration.py`:

```python
#!/usr/bin/env python3
"""
Integration test for MoveArm action
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time

# Import our custom action
from lab04_actions.action import MoveArm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ActionIntegrationTest(Node):

    def __init__(self):
        super().__init__('action_integration_test')

        # Create the action client
        self._action_client = ActionClient(self, MoveArm, 'move_arm')

        self.get_logger().info('Action integration test node created')

    def test_simple_trajectory(self):
        """Test a simple trajectory with the MoveArm action"""
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False

        # Create a simple trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        # Add one point to the trajectory
        point = JointTrajectoryPoint()
        point.positions = [0.5, 0.3, 0.1]
        point.velocities = [0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)
        trajectory.points.append(point)

        # Create the goal message
        goal_msg = MoveArm.Goal()
        goal_msg.joint_trajectory = trajectory
        goal_msg.tolerance = 0.01

        self.get_logger().info('Sending simple trajectory goal')

        # Send the goal and get a future for the result
        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Wait for the result
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return False

        # Get the result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        result = result_future.result().result
        success = result.success
        self.get_logger().info(f'Action test result: success={success}')
        return success

    def feedback_callback(self, feedback_msg):
        """Handle feedback during action execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.progress:.1%} complete')


def main(args=None):
    rclpy.init(args=args)

    test_node = ActionIntegrationTest()

    # Run the test
    success = test_node.test_simple_trajectory()

    result_msg = "PASSED" if success else "FAILED"
    test_node.get_logger().info(f'Action integration test: {result_msg}')

    test_node.destroy_node()
    rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    exit(main())
```

Add the test to the setup.py:

Edit `~/ros2_ws/src/move_arm_nodes/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'move_arm_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Action server and client nodes for MoveArm action',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_arm_server = move_arm_nodes.action_server:main',
            'move_arm_client = move_arm_nodes.action_client:main',
        ],
    },
)
```

---

## Step 16: Build and Test the Action Package

```bash
cd ~/ros2_ws
colcon build --packages-select lab04_actions move_arm_nodes
source ~/ros2_ws/install/setup.bash
```

---

## Step 17: Best Practices for Actions

### Action Design Guidelines

1. **Clear Goal Definition**: Define what the action should accomplish
2. **Meaningful Feedback**: Provide useful progress information
3. **Proper Result Handling**: Return clear success/failure indicators
4. **Cancellation Support**: Handle cancellation requests gracefully
5. **Error Handling**: Plan for various failure scenarios

### Performance Considerations

- **Feedback Frequency**: Don't publish feedback too frequently (e.g., every 100ms is usually sufficient)
- **Goal Validation**: Validate goals before starting execution
- **Resource Management**: Clean up resources when actions are canceled
- **Threading**: Use appropriate callback groups for concurrent access

### Common Action Patterns

**Navigation Action**:
```
# Goal: target pose
geometry_msgs/PoseStamped target_pose
---
# Result: success and final pose
bool success
geometry_msgs/PoseStamped final_pose
---
# Feedback: current pose and distance remaining
geometry_msgs/PoseStamped current_pose
float32 distance_remaining
```

**Pick and Place Action**:
```
# Goal: object location and grasp parameters
geometry_msgs/PoseStamped object_pose
float64 grasp_width
---
# Result: success and object status
bool success
bool object_grasped
---
# Feedback: current end-effector pose and grasp status
geometry_msgs/PoseStamped ee_pose
bool gripper_closed
float32 grasp_force
```

---

## Step 18: Troubleshooting Common Action Issues

### Issue 1: Action Server Not Found

**Symptoms**: `Waiting for action server...` indefinitely

**Solutions**:
```bash
# Check if action server is running
ros2 action list

# Check action info
ros2 action info /move_arm

# Make sure both server and client are using the same action name
```

### Issue 2: Callback Group Issues

**Symptoms**: Callbacks not executing concurrently

**Solutions**:
```python
# Use ReentrantCallbackGroup for concurrent access
from rclpy.callback_groups import ReentrantCallbackGroup

self._action_server = ActionServer(
    self,
    MoveArm,
    'move_arm',
    execute_callback=self.execute_callback,
    callback_group=ReentrantCallbackGroup(),  # Enable concurrent callbacks
)
```

### Issue 3: Import Errors

**Symptoms**: `ImportError` when trying to import action definition

**Solutions**:
- Make sure the action package is built: `colcon build --packages-select lab04_actions`
- Ensure the package is sourced: `source ~/ros2_ws/install/setup.bash`
- Check that the package.xml includes proper dependencies

---

## Lab Summary

In this lab, you've successfully:

✅ **Defined a custom MoveArm action** with goal, result, and feedback fields
✅ **Implemented an action server** that handles long-running goals with feedback
✅ **Created an action client** that sends goals and monitors progress
✅ **Validated action communication** with `ros2 action` commands
✅ **Understood action vs service vs topic** communication patterns

### Key Takeaways

- **Actions** provide asynchronous communication with feedback and cancellation
- **Action definitions** use `.action` files with goal/result/feedback separation
- **Action servers** implement callbacks for goals, cancellations, and execution
- **Action clients** send goals and can monitor progress with feedback
- **Actions vs Services vs Topics** have different use cases based on communication needs

---

## Next Steps

Now that you understand actions, you're ready to explore:

- **Lab 5**: URDF fundamentals for robot modeling
- **Lab 6**: TF2 coordinate frames and transforms
- **Lab 7**: Launch files for system startup

**Continue to [Lab 5: URDF Fundamentals](./urdf-fundamentals.md)**

---
**Previous**: [Lab 3: Services](./lab03-services.md) | **Next**: [Lab 5: URDF Fundamentals](./urdf-fundamentals.md)