---
title: Lifecycle and Real-Time Concepts
sidebar_label: Lifecycle and Real-Time Concepts
sidebar_position: 12
description: Master ROS 2 lifecycle nodes for state management and real-time programming concepts for deterministic robot control systems
---

# Lifecycle and Real-Time Concepts

## Overview

In this section, you'll master ROS 2 lifecycle nodes for robust state management and real-time programming concepts for deterministic robot control systems. You'll learn to implement nodes with explicit state transitions (unconfigured → inactive → active → finalized), configure real-time capabilities for time-critical applications, and understand the importance of deterministic execution in robotics systems.

**Duration**: 2 hours

**Learning Objectives**:
- ✅ Implement lifecycle nodes with proper state transition management
- ✅ Configure real-time capabilities for time-critical ROS 2 nodes
- ✅ Understand deterministic execution requirements for robot control
- ✅ Apply lifecycle management patterns for system reliability
- ✅ Integrate lifecycle nodes with launch files and system management

---

## Prerequisites

Before starting this section, ensure you have:

✅ **Completed Lab 1** - Talker/Listener basics
✅ **Completed Lab 2** - Custom messages
✅ **Completed Lab 3** - Services
✅ **Completed Lab 4** - Actions
✅ **Completed Lab 5** - Humanoid URDF
✅ **Completed Lab 6** - TF2 Broadcasting
✅ **ROS 2 Humble installed** with all standard packages
✅ **Understanding of** ROS 2 nodes, topics, services, and actions
✅ **Basic knowledge** of system state management concepts

---

## Step 1: Understanding Lifecycle Nodes

### What are Lifecycle Nodes?

**Lifecycle nodes** provide explicit state management for ROS 2 nodes, allowing for predictable and robust system behavior. Rather than having nodes that simply start and run, lifecycle nodes have well-defined states and transitions between states.

### Lifecycle Node States

| State | Description | Transitions To |
|-------|-------------|----------------|
| **Unconfigured** | Node created but not configured | Inactive (via configure) |
| **Inactive** | Configured but not running | Active (via activate), Unconfigured (via cleanup) |
| **Active** | Running and operational | Inactive (via deactivate), ShuttingDown (via shutdown) |
| **Activating** | Transitioning to active | Active or Inactive (on error) |
| **Deactivating** | Transitioning to inactive | Active or Inactive |
| **CleaningUp** | Cleaning up resources | Unconfigured or Inactive |
| **ShuttingDown** | Shutting down | Finalized |
| **ErrorProcessing** | Error state | Any state depending on error recovery |
| **Finalized** | Completely shut down | None |

### Benefits of Lifecycle Nodes

- **Predictable behavior**: Well-defined state transitions
- **Resource management**: Proper allocation/deallocation during transitions
- **System reliability**: Graceful handling of startup/shutdown
- **Configuration flexibility**: Runtime reconfiguration capabilities
- **Monitoring**: Ability to monitor node state programmatically

---

## Step 2: Lifecycle Node Implementation

Let's create a lifecycle node example. First, create a new package:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python lifecycle_examples --dependencies rclpy lifecycle_msgs std_msgs geometry_msgs
```

Create the lifecycle node implementation:

```bash
mkdir -p ~/ros2_ws/src/lifecycle_examples/lifecycle_examples
touch ~/ros2_ws/src/lifecycle_examples/lifecycle_examples/lifecycle_node_example.py
```

Add the following content to `~/ros2_ws/src/lifecycle_examples/lifecycle_examples/lifecycle_node_example.py`:

```python
#!/usr/bin/env python3
"""
Lifecycle Node Example - demonstrates ROS 2 lifecycle node implementation
"""

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import State as LifecycleStateType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class LifecycleControlNode(LifecycleNode):

    def __init__(self):
        super().__init__('lifecycle_control_node')

        # Initialize variables that will be set during lifecycle transitions
        self.publisher = None
        self.cmd_vel_publisher = None
        self.timer = None
        self.counter = 0

        self.get_logger().info('Lifecycle node created in unconfigured state')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Callback when node enters configuring state.
        Initialize resources and prepare for activation.
        """
        self.get_logger().info('Configuring lifecycle node...')

        # Create publishers
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(String, 'lifecycle_status', qos_profile)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)

        # Initialize variables
        self.counter = 0

        # Example: Initialize hardware interfaces, open files, etc.
        self.hardware_initialized = True
        self.get_logger().info('Hardware interfaces initialized')

        # Example: Load configuration parameters
        self.declare_parameter('control_frequency', 10)
        self.control_frequency = self.get_parameter('control_frequency').value
        self.get_logger().info(f'Control frequency set to: {self.control_frequency}Hz')

        # Return SUCCESS to indicate successful configuration
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Callback when node enters activating state.
        Activate the node and start normal operation.
        """
        self.get_logger().info('Activating lifecycle node...')

        # Activate publishers/subscribers
        self.publisher.on_activate()
        self.cmd_vel_publisher.on_activate()

        # Create timer for periodic operation
        self.timer = self.create_timer(
            1.0 / self.control_frequency,
            self.timer_callback
        )

        # Example: Start hardware control loop
        self.control_loop_active = True
        self.get_logger().info(f'Control loop started at {self.control_frequency}Hz')

        self.get_logger().info('Lifecycle node activated successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Callback when node enters deactivating state.
        Stop normal operation and prepare for inactivity.
        """
        self.get_logger().info('Deactivating lifecycle node...')

        # Deactivate publishers/subscribers
        self.publisher.on_deactivate()
        self.cmd_vel_publisher.on_deactivate()

        # Destroy timer
        if self.timer:
            self.destroy_timer(self.timer)
            self.timer = None

        # Example: Pause hardware control
        self.control_loop_active = False
        self.get_logger().info('Control loop paused')

        self.get_logger().info('Lifecycle node deactivated successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Callback when node enters cleaning up state.
        Clean up resources allocated during configuration.
        """
        self.get_logger().info('Cleaning up lifecycle node...')

        # Example: Close files, disconnect hardware, etc.
        if hasattr(self, 'hardware_initialized') and self.hardware_initialized:
            # Simulate hardware cleanup
            self.get_logger().info('Hardware interfaces cleaned up')
            self.hardware_initialized = False

        # Reset publishers
        if self.publisher:
            self.destroy_publisher(self.publisher)
            self.publisher = None

        if self.cmd_vel_publisher:
            self.destroy_publisher(self.cmd_vel_publisher)
            self.cmd_vel_publisher = None

        self.get_logger().info('Lifecycle node cleaned up successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Callback when node enters shutting down state.
        Perform final cleanup before node destruction.
        """
        self.get_logger().info('Shutting down lifecycle node...')

        # Perform final cleanup
        if self.timer:
            self.destroy_timer(self.timer)

        if self.publisher:
            self.destroy_publisher(self.publisher)

        if self.cmd_vel_publisher:
            self.destroy_publisher(self.cmd_vel_publisher)

        self.get_logger().info('Lifecycle node shutdown complete')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Callback when node enters error processing state.
        Handle errors and attempt recovery.
        """
        self.get_logger().error(f'Lifecycle node entered error state from: {state.label}')

        # Example: Attempt recovery
        try:
            # Try to reinitialize critical components
            if not self.hardware_initialized:
                self.get_logger().info('Attempting to recover hardware interfaces...')
                self.hardware_initialized = True

            # Return SUCCESS if recovery successful, FAILURE if not
            self.get_logger().info('Error recovery attempted successfully')
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Error recovery failed: {e}')
            return TransitionCallbackReturn.FAILURE

    def timer_callback(self):
        """
        Timer callback - runs when node is active
        """
        if self.counter % 10 == 0:
            # Publish status message every 10 iterations
            msg = String()
            msg.data = f'Lifecycle node active - iteration {self.counter}'
            self.publisher.publish(msg)

            # Publish a simple velocity command
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_msg.angular.z = 0.1 * (self.counter % 10)  # Gentle turn
            self.cmd_vel_publisher.publish(cmd_msg)

            self.get_logger().info(f'Published status: {msg.data}')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    # Create lifecycle node
    node = LifecycleControlNode()

    # Initially the node is in "unconfigured" state
    assert node.get_current_state().label == 'unconfigured'

    # Manually trigger transitions for demonstration
    # In practice, these would be triggered by lifecycle manager

    # Configure the node
    node.trigger_configure()
    assert node.get_current_state().label == 'inactive'

    # Activate the node
    node.trigger_activate()
    assert node.get_current_state().label == 'active'

    try:
        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')

        # Properly shut down the lifecycle node
        node.trigger_deactivate()
        node.trigger_cleanup()
        node.trigger_shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 3: Creating a Lifecycle Manager Client

Let's create a client to manage lifecycle nodes:

```bash
touch ~/ros2_ws/src/lifecycle_examples/lifecycle_examples/lifecycle_manager_client.py
```

Add the following content to `~/ros2_ws/src/lifecycle_examples/lifecycle_examples/lifecycle_manager_client.py`:

```python
#!/usr/bin/env python3
"""
Lifecycle Manager Client - controls lifecycle nodes through service calls
"""

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState, GetAvailableStates, GetAvailableTransitions
from lifecycle_msgs.msg import Transition, State
import sys


class LifecycleManagerClient(Node):

    def __init__(self):
        super().__init__('lifecycle_manager_client')

        # Store the node name to manage
        self.target_node_name = 'lifecycle_control_node'

        # Create clients for lifecycle services
        self.change_state_cli = self.create_client(
            ChangeState,
            f'{self.target_node_name}/change_state'
        )

        self.get_state_cli = self.create_client(
            GetState,
            f'{self.target_node_name}/get_state'
        )

        self.get_available_states_cli = self.create_client(
            GetAvailableStates,
            f'{self.target_node_name}/get_available_states'
        )

        self.get_available_transitions_cli = self.create_client(
            GetAvailableTransitions,
            f'{self.target_node_name}/get_available_transitions'
        )

        # Wait for services to be available
        self.get_logger().info('Waiting for lifecycle services...')
        self.change_state_cli.wait_for_service()
        self.get_state_cli.wait_for_service()
        self.get_available_states_cli.wait_for_service()
        self.get_available_transitions_cli.wait_for_service()

        self.get_logger().info('Lifecycle manager client ready')

    def get_current_state(self):
        """
        Get the current state of the target lifecycle node
        """
        request = GetState.Request()
        future = self.get_state_cli.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            return response.current_state
        else:
            self.get_logger().error('Failed to get current state')
            return None

    def change_state(self, transition_id):
        """
        Change the state of the target lifecycle node
        """
        request = ChangeState.Request()
        request.transition.id = transition_id

        future = self.change_state_cli.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            success_str = "SUCCESS" if response.success else "FAILED"
            self.get_logger().info(f'State change {success_str} for transition {transition_id}')
            return response.success
        else:
            self.get_logger().error(f'Failed to change state with transition {transition_id}')
            return False

    def configure_node(self):
        """
        Configure the target node (transition from unconfigured to inactive)
        """
        self.get_logger().info(f'Configuring {self.target_node_name}...')
        success = self.change_state(Transition.TRANSITION_CONFIGURE)

        if success:
            current_state = self.get_current_state()
            if current_state:
                self.get_logger().info(f'Node is now in state: {current_state.label}')
        else:
            self.get_logger().error('Failed to configure node')

        return success

    def activate_node(self):
        """
        Activate the target node (transition from inactive to active)
        """
        self.get_logger().info(f'Activating {self.target_node_name}...')
        success = self.change_state(Transition.TRANSITION_ACTIVATE)

        if success:
            current_state = self.get_current_state()
            if current_state:
                self.get_logger().info(f'Node is now in state: {current_state.label}')
        else:
            self.get_logger().error('Failed to activate node')

        return success

    def deactivate_node(self):
        """
        Deactivate the target node (transition from active to inactive)
        """
        self.get_logger().info(f'Deactivating {self.target_node_name}...')
        success = self.change_state(Transition.TRANSITION_DEACTIVATE)

        if success:
            current_state = self.get_current_state()
            if current_state:
                self.get_logger().info(f'Node is now in state: {current_state.label}')
        else:
            self.get_logger().error('Failed to deactivate node')

        return success

    def cleanup_node(self):
        """
        Clean up the target node (transition from inactive to unconfigured)
        """
        self.get_logger().info(f'Cleaning up {self.target_node_name}...')
        success = self.change_state(Transition.TRANSITION_CLEANUP)

        if success:
            current_state = self.get_current_state()
            if current_state:
                self.get_logger().info(f'Node is now in state: {current_state.label}')
        else:
            self.get_logger().error('Failed to clean up node')

        return success

    def shutdown_node(self):
        """
        Shutdown the target node (transition to finalized)
        """
        self.get_logger().info(f'Shutting down {self.target_node_name}...')
        success = self.change_state(Transition.TRANSITION_SHUTDOWN)

        if success:
            current_state = self.get_current_state()
            if current_state:
                self.get_logger().info(f'Node is now in state: {current_state.label}')
        else:
            self.get_logger().error('Failed to shutdown node')

        return success

    def print_available_states(self):
        """
        Print all available states for the target node
        """
        request = GetAvailableStates.Request()
        future = self.get_available_states_cli.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info('Available states:')
            for state in response.available_states:
                self.get_logger().info(f'  - {state.id}: {state.label}')
        else:
            self.get_logger().error('Failed to get available states')

    def print_available_transitions(self):
        """
        Print all available transitions for the target node
        """
        request = GetAvailableTransitions.Request()
        future = self.get_available_transitions_cli.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response:
            self.get_logger().info('Available transitions:')
            for transition in response.available_transitions:
                self.get_logger().info(
                    f'  - {transition.transition.id}: {transition.transition.label} '
                    f'(from {transition.start_state.label} to {transition.goal_state.label})'
                )
        else:
            self.get_logger().error('Failed to get available transitions')

    def print_current_state(self):
        """
        Print the current state of the target node
        """
        current_state = self.get_current_state()
        if current_state:
            self.get_logger().info(f'Current state of {self.target_node_name}: {current_state.label}')
        else:
            self.get_logger().error('Failed to get current state')


def main(args=None):
    rclpy.init(args=args)

    manager = LifecycleManagerClient()

    # Parse command line arguments
    if len(sys.argv) < 2:
        print("Usage: python lifecycle_manager_client.py <command>")
        print("Commands: get_state, configure, activate, deactivate, cleanup, shutdown, list_states, list_transitions")
        sys.exit(1)

    command = sys.argv[1]

    try:
        if command == 'get_state':
            manager.print_current_state()
        elif command == 'configure':
            manager.configure_node()
        elif command == 'activate':
            manager.activate_node()
        elif command == 'deactivate':
            manager.deactivate_node()
        elif command == 'cleanup':
            manager.cleanup_node()
        elif command == 'shutdown':
            manager.shutdown_node()
        elif command == 'list_states':
            manager.print_available_states()
        elif command == 'list_transitions':
            manager.print_available_transitions()
        else:
            print(f"Unknown command: {command}")
            print("Available commands: get_state, configure, activate, deactivate, cleanup, shutdown, list_states, list_transitions")
            sys.exit(1)

    except KeyboardInterrupt:
        manager.get_logger().info('Lifecycle manager client stopped')
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 4: Real-Time Programming Concepts

Real-time programming is crucial for time-critical robot applications. Let's explore real-time concepts in ROS 2:

```bash
touch ~/ros2_ws/src/lifecycle_examples/lifecycle_examples/realtime_control_node.py
```

Add the following content to `~/ros2_ws/src/lifecycle_examples/lifecycle_examples/realtime_control_node.py`:

```python
#!/usr/bin/env python3
"""
Real-Time Control Node - demonstrates real-time programming concepts in ROS 2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration
import time
import threading
from collections import deque
import numpy as np


class RealTimeControlNode(Node):

    def __init__(self):
        super().__init__('realtime_control_node')

        # Real-time control parameters
        self.control_frequency = 100  # Hz (100 Hz for real-time control)
        self.dt = 1.0 / self.control_frequency  # Control period

        # Publishers for control commands
        self.joint_cmd_publisher = self.create_publisher(Float64MultiArray, 'joint_commands', 1)
        self.velocity_cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        # Subscribers for sensor feedback
        self.joint_state_subscriber = self.create_subscription(
            Float64MultiArray, 'joint_states', self.joint_state_callback, 1)

        # Real-time control variables
        self.joint_positions = [0.0] * 6  # 6 DOF robot
        self.joint_velocities = [0.0] * 6
        self.desired_positions = [0.0] * 6
        self.control_enabled = False

        # Timing and performance monitoring
        self.loop_times = deque(maxlen=100)  # Keep last 100 loop times
        self.period_violations = 0  # Count of deadline misses
        self.iteration_count = 0

        # Real-time thread for control loop
        self.rt_thread = None
        self.rt_thread_running = False

        # PID controller gains
        self.kp = [50.0] * 6  # Proportional gains
        self.ki = [10.0] * 6  # Integral gains
        self.kd = [5.0] * 6   # Derivative gains

        self.integral_errors = [0.0] * 6
        self.previous_errors = [0.0] * 6

        self.get_logger().info(
            f'Real-time control node initialized at {self.control_frequency}Hz '
            f'with control period {self.dt*1000:.1f}ms'
        )

    def joint_state_callback(self, msg):
        """
        Callback for joint state updates
        """
        if len(msg.data) >= 6:
            self.joint_positions = list(msg.data[:6])
        else:
            self.get_logger().warn(f'Insufficient joint data: {len(msg.data)} values, expected 6')

    def start_realtime_control(self):
        """
        Start the real-time control thread
        """
        if self.rt_thread is not None:
            self.get_logger().warn('Real-time control thread already running')
            return

        self.rt_thread_running = True
        self.rt_thread = threading.Thread(target=self.realtime_control_loop)
        self.rt_thread.daemon = True  # Dies when main thread dies
        self.rt_thread.start()

        self.control_enabled = True
        self.get_logger().info('Real-time control thread started')

    def stop_realtime_control(self):
        """
        Stop the real-time control thread
        """
        self.control_enabled = False
        self.rt_thread_running = False

        if self.rt_thread:
            self.rt_thread.join(timeout=2.0)  # Wait up to 2 seconds for thread to finish
            self.rt_thread = None

        self.get_logger().info('Real-time control thread stopped')

    def realtime_control_loop(self):
        """
        Real-time control loop - runs in separate thread for timing precision
        """
        self.get_logger().info('Real-time control loop started')

        # Initialize timing
        start_time = time.time()
        next_iteration_time = start_time + self.dt

        while self.rt_thread_running and self.control_enabled:
            iteration_start = time.time()

            try:
                # Execute control algorithm
                self.execute_control_iteration()

                # Calculate loop time
                iteration_end = time.time()
                loop_time = iteration_end - iteration_start
                self.loop_times.append(loop_time * 1000)  # Store in ms

                # Check for period violation
                if loop_time > self.dt:
                    self.period_violations += 1
                    self.get_logger().warn(
                        f'Period violation! Loop took {loop_time*1000:.2f}ms, '
                        f'exceeded deadline of {self.dt*1000:.2f}ms'
                    )

                # Calculate next iteration time
                next_iteration_time += self.dt

                # Sleep until next iteration (with deadline checking)
                sleep_time = next_iteration_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # We missed our deadline, resynchronize
                    next_iteration_time = time.time() + self.dt
                    self.get_logger().warn('Resynchronizing control loop after deadline miss')

                # Log performance periodically
                self.iteration_count += 1
                if self.iteration_count % (self.control_frequency * 5) == 0:  # Every 5 seconds
                    avg_loop_time = sum(self.loop_times) / len(self.loop_times) if self.loop_times else 0
                    max_loop_time = max(self.loop_times) if self.loop_times else 0

                    self.get_logger().info(
                        f'Control performance: avg={avg_loop_time:.2f}ms, '
                        f'max={max_loop_time:.2f}ms, '
                        f'violations={self.period_violations}, '
                        f'frequency={self.control_frequency}Hz'
                    )

            except Exception as e:
                self.get_logger().error(f'Error in real-time control loop: {e}')
                time.sleep(0.1)  # Brief pause before continuing

        self.get_logger().info('Real-time control loop exited')

    def execute_control_iteration(self):
        """
        Execute one iteration of the control algorithm
        """
        # Calculate control commands using PID control
        control_commands = []

        for i in range(min(len(self.joint_positions), len(self.desired_positions))):
            # Calculate error
            error = self.desired_positions[i] - self.joint_positions[i]

            # Update integral (with anti-windup)
            self.integral_errors[i] = self.integral_errors[i] + error * self.dt
            # Clamp integral to prevent windup
            self.integral_errors[i] = max(min(self.integral_errors[i], 10.0), -10.0)

            # Calculate derivative
            derivative = (error - self.previous_errors[i]) / self.dt if self.dt > 0 else 0.0
            self.previous_errors[i] = error

            # Calculate PID output
            pid_output = (
                self.kp[i] * error +
                self.ki[i] * self.integral_errors[i] +
                self.kd[i] * derivative
            )

            # Apply output limits
            pid_output = max(min(pid_output, 100.0), -100.0)  # Clamp to ±100
            control_commands.append(pid_output)

        # Pad with zeros if needed
        while len(control_commands) < 6:
            control_commands.append(0.0)

        # Publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = control_commands
        self.joint_cmd_publisher.publish(cmd_msg)

    def set_desired_positions(self, positions):
        """
        Set desired joint positions for the controller
        """
        if len(positions) == len(self.desired_positions):
            self.desired_positions = list(positions)
            self.get_logger().info(f'Set desired positions: {positions}')
        else:
            self.get_logger().error(
                f'Incorrect number of positions: {len(positions)}, expected {len(self.desired_positions)}'
            )

    def get_performance_stats(self):
        """
        Get real-time performance statistics
        """
        if not self.loop_times:
            return {
                'avg_loop_time_ms': 0.0,
                'max_loop_time_ms': 0.0,
                'min_loop_time_ms': 0.0,
                'period_violations': self.period_violations,
                'total_iterations': self.iteration_count
            }

        return {
            'avg_loop_time_ms': sum(self.loop_times) / len(self.loop_times),
            'max_loop_time_ms': max(self.loop_times),
            'min_loop_time_ms': min(self.loop_times),
            'period_violations': self.period_violations,
            'total_iterations': self.iteration_count
        }


def main(args=None):
    rclpy.init(args=args)

    node = RealTimeControlNode()

    try:
        # Start real-time control
        node.start_realtime_control()

        # Set some example desired positions
        node.set_desired_positions([0.5, 0.3, 0.0, -0.3, -0.5, 0.0])

        # Let it run for a while
        time.sleep(10.0)

        # Get performance stats
        stats = node.get_performance_stats()
        node.get_logger().info(f'Performance stats: {stats}')

    except KeyboardInterrupt:
        node.get_logger().info('Real-time control node stopped by user')
    finally:
        node.stop_realtime_control()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 5: Update Package Configuration

Update the setup.py file:

Edit `~/ros2_ws/src/lifecycle_examples/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'lifecycle_examples'

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
    description='Examples for lifecycle nodes and real-time programming in ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lifecycle_node_example = lifecycle_examples.lifecycle_node_example:main',
            'lifecycle_manager_client = lifecycle_examples.lifecycle_manager_client:main',
            'realtime_control_node = lifecycle_examples.realtime_control_node:main',
        ],
    },
)
```

Update the package.xml:

Edit `~/ros2_ws/src/lifecycle_examples/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lifecycle_examples</name>
  <version>0.0.0</version>
  <description>Examples for lifecycle nodes and real-time programming in ROS 2</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>lifecycle_msgs</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## Step 6: Building the Lifecycle Package

```bash
cd ~/ros2_ws
colcon build --packages-select lifecycle_examples
source ~/ros2_ws/install/setup.bash
```

---

## Step 7: Testing Lifecycle Nodes

### Running Lifecycle Node Example

**Terminal 1 - Lifecycle Node**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run lifecycle_examples lifecycle_node_example
```

**Terminal 2 - Lifecycle Manager Client** (open a new terminal):
```bash
source ~/ros2_ws/install/setup.bash

# Check current state
ros2 run lifecycle_examples lifecycle_manager_client get_state

# Configure the node
ros2 run lifecycle_examples lifecycle_manager_client configure

# Check state after configuration
ros2 run lifecycle_examples lifecycle_manager_client get_state

# Activate the node
ros2 run lifecycle_examples lifecycle_manager_client activate

# Check state after activation
ros2 run lifecycle_examples lifecycle_manager_client get_state

# List available states and transitions
ros2 run lifecycle_examples lifecycle_manager_client list_states
ros2 run lifecycle_examples lifecycle_manager_client list_transitions

# Deactivate and cleanup
ros2 run lifecycle_examples lifecycle_manager_client deactivate
ros2 run lifecycle_examples lifecycle_manager_client cleanup
```

---

## Step 8: Real-Time Programming Concepts

### Real-Time Requirements for Robotics

Robots require real-time behavior for:

1. **Safety**: Emergency stops and collision avoidance
2. **Control**: Motor control loops (typically 100Hz-1kHz)
3. **Sensing**: Sensor data processing and fusion
4. **Actuation**: Precise timing for coordinated movements

### Real-Time vs Non-Real-Time Systems

| Aspect | Non-Real-Time | Real-Time |
|--------|---------------|-----------|
| **Timing Guarantees** | Best-effort | Deterministic deadlines |
| **Scheduling** | General-purpose | Priority-based, deadline-aware |
| **Interrupt Handling** | Standard | Minimal, predictable latency |
| **Memory Management** | Dynamic allocation | Pre-allocated, no garbage collection |
| **System Calls** | Any POSIX call | Only real-time safe calls |

### Real-Time Programming in ROS 2

While Python isn't ideal for hard real-time systems, ROS 2 supports real-time concepts:

1. **Soft Real-Time**: Meeting timing requirements with high probability
2. **Real-Time Middleware**: DDS with real-time QoS profiles
3. **Real-Time Kernel**: Use with PREEMPT_RT patched Linux kernels
4. **Deterministic Execution**: Predictable execution paths

### Real-Time Safe Programming Practices

```python
# AVOID: Dynamic memory allocation in critical paths
def bad_rt_function():
    large_list = []  # Could cause unpredictable GC
    for i in range(10000):
        large_list.append(i)  # More allocations
    return large_list

# PREFER: Pre-allocate and reuse
class RTController:
    def __init__(self):
        self.buffer = [0.0] * 1000  # Pre-allocated
        self.index = 0

    def update(self, value):
        self.buffer[self.index] = value
        self.index = (self.index + 1) % len(self.buffer)  # Wrap around
```

### Real-Time Scheduling in Linux

For true real-time performance on Linux:

1. **Install RT kernel**: Use PREEMPT_RT patched kernel
2. **Configure scheduler**: Use SCHED_FIFO or SCHED_RR for critical threads
3. **Lock memory**: Prevent page faults with mlock()
4. **Minimize system calls**: Reduce context switches

---

## Step 9: Lifecycle Node Best Practices

### State Management Best Practices

1. **Always return appropriate TransitionCallbackReturn values**
2. **Handle errors gracefully in each callback**
3. **Clean up resources properly in cleanup and shutdown callbacks**
4. **Use mutexes when sharing data between callbacks and normal operation**
5. **Validate inputs and configurations before accepting transitions**

### Real-Time Best Practices

1. **Minimize non-deterministic operations** (disk I/O, network calls, dynamic allocation)
2. **Use appropriate control frequencies** (100Hz for joint control, 10Hz for high-level planning)
3. **Monitor timing performance** and deadline violations
4. **Implement watchdog mechanisms** for detecting missed deadlines
5. **Separate real-time and non-real-time components**

### Example: Proper Resource Management

```python
class ProperLifecycleNode(LifecycleNode):

    def __init__(self):
        super().__init__('proper_lifecycle_node')
        self.hardware_resources = None
        self.is_operational = False

    def on_configure(self, state):
        try:
            # Initialize resources
            self.hardware_resources = self.initialize_hardware()
            if not self.hardware_resources:
                self.get_logger().error('Failed to initialize hardware')
                return TransitionCallbackReturn.FAILURE

            # Create publishers/subscribers
            self.publisher = self.create_publisher(String, 'status', 10)

            self.is_operational = True
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Configuration failed: {e}')
            return TransitionCallbackReturn.ERROR

    def on_cleanup(self, state):
        try:
            # Properly clean up resources
            if hasattr(self, 'publisher') and self.publisher:
                self.destroy_publisher(self.publisher)
                self.publisher = None

            if self.hardware_resources:
                self.cleanup_hardware(self.hardware_resources)
                self.hardware_resources = None

            self.is_operational = False
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f'Cleanup failed: {e}')
            return TransitionCallbackReturn.ERROR

    def initialize_hardware(self):
        """
        Initialize hardware resources safely
        """
        # Implementation here
        pass

    def cleanup_hardware(self, resources):
        """
        Clean up hardware resources safely
        """
        # Implementation here
        pass
```

---

## Step 10: Advanced Lifecycle Patterns

### Component-Based Architecture

Lifecycle nodes work well with composition:

```python
class RobotSystemManager(LifecycleNode):

    def __init__(self):
        super().__init__('robot_system_manager')

        # Create lifecycle managed components
        self.drivers = []
        self.controllers = []
        self.sensors = []

    def on_configure(self, state):
        success = True

        # Configure all components
        for driver in self.drivers:
            result = driver.trigger_configure()
            success &= result.success

        for controller in self.controllers:
            result = controller.trigger_configure()
            success &= result.success

        return TransitionCallbackReturn.SUCCESS if success else TransitionCallbackReturn.FAILURE
```

### Hierarchical State Management

Complex robots often have hierarchical state management:

```python
class HierarchicalLifecycleManager:

    def __init__(self):
        self.base_controller = None  # Lifecycle node
        self.arm_controller = None   # Lifecycle node
        self.sensor_system = None    # Lifecycle node

    def configure_robot(self):
        """
        Configure all subsystems in dependency order
        """
        # Sensors first
        self.sensor_system.trigger_configure()

        # Base after sensors are ready
        self.base_controller.trigger_configure()

        # Arm last (depends on base)
        self.arm_controller.trigger_configure()

    def activate_robot(self):
        """
        Activate all subsystems
        """
        # Activate in parallel for efficiency
        self.sensor_system.trigger_activate()
        self.base_controller.trigger_activate()
        self.arm_controller.trigger_activate()
```

---

## Step 11: Lifecycle Node Tools and Utilities

### Using lifecycle tools

ROS 2 provides command-line tools for lifecycle management:

```bash
# List lifecycle nodes
ros2 lifecycle list

# Get state of a specific node
ros2 lifecycle get /lifecycle_control_node

# Change state of a node
ros2 lifecycle configure /lifecycle_control_node
ros2 lifecycle activate /lifecycle_control_node
ros2 lifecycle deactivate /lifecycle_control_node
ros2 lifecycle cleanup /lifecycle_control_node
ros2 lifecycle shutdown /lifecycle_control_node

# Transition directly to active (configure + activate)
ros2 lifecycle set /lifecycle_control_node active
```

### Launch files with lifecycle nodes

You can manage lifecycle nodes in launch files:

```python
# In a launch file
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.events import Shutdown

def generate_launch_description():
    # Lifecycle node that starts unconfigured
    lifecycle_node = LifecycleNode(
        package='lifecycle_examples',
        executable='lifecycle_node_example',
        name='lifecycle_control_node',
        # Don't automatically configure/activate
        parameters=[{'auto_configure': False, 'auto_activate': False}]
    )

    return LaunchDescription([
        lifecycle_node,
    ])
```

---

## Step 12: Real-World Applications

### Industrial Robot Control

In industrial robotics, lifecycle nodes ensure:

- **Safe startup**: All safety checks before activation
- **Coordinated operation**: Synchronized activation of multiple nodes
- **Graceful shutdown**: Safe stopping of all robot motion
- **Error recovery**: Automatic or manual recovery from faults

### Mobile Robot Navigation

For navigation systems, lifecycle nodes manage:

- **Localization**: AMCL in configured state until map loaded
- **Costmaps**: Static map loaded before inflation activated
- **Controllers**: Trajectory planners activated after localizer ready
- **Sensors**: LiDAR and cameras initialized before costmap activation

### Multi-Robot Systems

In multi-robot systems, lifecycle management coordinates:

- **Formation control**: All robots reach configured state before formation begins
- **Task allocation**: Assignment happens after all robots are ready
- **Recovery**: Individual robot recovery without affecting others

---

## Summary

In this section, you've learned:

✅ **Lifecycle node implementation** with proper state transitions
✅ **Real-time programming concepts** for deterministic robot control
✅ **Resource management** in lifecycle callbacks
✅ **Real-time performance monitoring** and deadline violation detection
✅ **Advanced lifecycle patterns** for complex robotic systems

### Key Takeaways

- **Lifecycle nodes** provide predictable state management for complex systems
- **Real-time programming** requires careful attention to timing and resource usage
- **Proper resource management** prevents memory leaks and system instability
- **State transition callbacks** must handle errors gracefully
- **Performance monitoring** helps detect timing issues in real-time systems

---

## Next Steps

Now that you understand lifecycle nodes and real-time concepts, you're ready to explore:

- **Lab 7**: Launch files for system startup and management
- **Lab 8**: RViz integration for visualization
- **Lab 9**: Advanced navigation and path planning

**Continue to [Lab 7: Launch Files](./lab07-launch-files.md)**

---
**Previous**: [Lab 6: TF2 Broadcasting](./lab06-tf2-broadcasting.md) | **Next**: [Lab 7: Launch Files](./lab07-launch-files.md)