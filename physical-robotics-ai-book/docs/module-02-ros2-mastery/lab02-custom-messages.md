---
title: Lab 2 - Custom Messages
sidebar_label: Lab 2 - Custom Messages
sidebar_position: 5
description: Define and use custom message types in ROS 2, create RobotState.msg with header, joint positions, velocities, battery level and status
---

# Lab 2: Custom Messages

## Overview

In this lab, you'll learn to define and use custom message types in ROS 2. You'll create a `RobotState.msg` with fields for joint positions, velocities, battery level, and status. Custom messages are essential for passing domain-specific data between nodes in your robotic applications.

**Duration**: 1.5 hours

**Learning Objectives**:
- ✅ Define custom message types using `.msg` syntax
- ✅ Configure `rosidl_generate_interfaces` for message generation
- ✅ Use custom messages in publisher and subscriber nodes
- ✅ Validate message generation with `ros2 interface` commands
- ✅ Implement RobotState message with header, joint arrays, and status fields

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **Completed Lab 1** - Talker/Listener basics
✅ **ROS 2 Humble installed** with all standard packages
✅ **Basic understanding** of ROS 2 nodes, topics, and standard messages
✅ **Python programming skills** for implementing message usage

---

## Step 1: Understanding ROS 2 Message Types

### What are Custom Messages?

**ROS 2 messages** define the structure of data passed between nodes. While ROS 2 provides standard messages (std_msgs, geometry_msgs, sensor_msgs), custom messages allow you to define domain-specific data structures.

### Message Definition Syntax

ROS 2 uses a simple syntax for message definitions:

```
# Comments start with #
string name                    # String field
int32 age                      # Integer field
float64[] values               # Array of floats
geometry_msgs/Point position   # Nested message type
```

### RobotState Message Requirements

Based on our spec, we need a `RobotState.msg` with:
- `header`: Standard ROS header (timestamp, frame_id)
- `joint_positions[]`: Array of joint angles in radians
- `joint_velocities[]`: Array of joint velocities in rad/s
- `battery_level`: Battery percentage (0.0 to 100.0)
- `status`: Robot status (IDLE=0, MOVING=1, ERROR=2)

---

## Step 2: Create Custom Messages Package

First, let's create a package specifically for our custom messages:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake lab02_custom_messages
```

This creates a CMake-based package since message packages typically use CMake for the `rosidl_generate_interfaces` macro.

---

## Step 3: Define the RobotState Message

Create the messages directory:
```bash
mkdir -p ~/ros2_ws/src/lab02_custom_messages/msg
```

Create the RobotState message definition:
```bash
touch ~/ros2_ws/src/lab02_custom_messages/msg/RobotState.msg
```

Add the following content to `~/ros2_ws/src/lab02_custom_messages/msg/RobotState.msg`:

```
# RobotState.msg: Represents the state of a robot
# Includes joint information, battery level, and operational status

# Standard header for timestamp and frame information
std_msgs/Header header

# Joint state information
float64[] joint_positions     # Joint angles in radians
float64[] joint_velocities    # Joint velocities in rad/s

# System status
float32 battery_level         # Battery level percentage (0.0 - 100.0)
uint8 status                  # Robot status: IDLE=0, MOVING=1, ERROR=2

# Constants for status field
uint8 IDLE = 0
uint8 MOVING = 1
uint8 ERROR = 2
```

---

## Step 4: Configure CMake for Message Generation

Edit the `CMakeLists.txt` file to configure message generation:

Edit `~/ros2_ws/src/lab02_custom_messages/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(lab02_custom_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define message files
set(msg_files
  "msg/RobotState.msg"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_package()
```

---

## Step 5: Update Package.xml

Edit the `package.xml` file to include the necessary dependencies:

Edit `~/ros2_ws/src/lab02_custom_messages/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lab02_custom_messages</name>
  <version>0.0.0</version>
  <description>Custom messages for robot state</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Step 6: Build the Custom Messages Package

```bash
cd ~/ros2_ws
colcon build --packages-select lab02_custom_messages
source ~/ros2_ws/install/setup.bash
```

---

## Step 7: Verify Message Generation

Check that the RobotState message was generated correctly:

```bash
ros2 interface show lab02_custom_messages/msg/RobotState
```

**Expected Output**:
```
std_msgs/Header header
float64[] joint_positions
float64[] joint_velocities
float32 battery_level
uint8 status
uint8 IDLE=0
uint8 MOVING=1
uint8 ERROR=2
```

You can also list all available custom messages:
```bash
ros2 interface list | grep RobotState
```

---

## Step 8: Create Publisher with Custom Message

Now let's create a publisher node that uses our custom RobotState message. First, create a new package for the publisher:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_state_publisher --dependencies rclpy lab02_custom_messages
```

Create the publisher file:
```bash
touch ~/ros2_ws/src/robot_state_publisher/robot_state_publisher/state_publisher.py
```

Add the following code to `~/ros2_ws/src/robot_state_publisher/robot_state_publisher/state_publisher.py`:

```python
#!/usr/bin/env python3
"""
Robot state publisher - publishes custom RobotState messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

# Import our custom message
from lab02_custom_messages.msg import RobotState
import math
import time


class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create publisher for our custom RobotState message
        self.publisher = self.create_publisher(RobotState, 'robot_state', 10)

        # Timer to publish at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Robot state publisher started')

    def timer_callback(self):
        msg = RobotState()

        # Set header with timestamp and frame
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Simulate joint positions (e.g., 6 DOF robot)
        # Using sine waves to simulate moving joints
        msg.joint_positions = [
            math.sin(self.counter * 0.1),           # Joint 1
            math.sin(self.counter * 0.1 + math.pi/3), # Joint 2
            math.sin(self.counter * 0.1 + 2*math.pi/3), # Joint 3
            math.sin(self.counter * 0.1 + math.pi),     # Joint 4
            math.sin(self.counter * 0.1 + 4*math.pi/3), # Joint 5
            math.sin(self.counter * 0.1 + 5*math.pi/3)  # Joint 6
        ]

        # Calculate velocities (derivative of positions)
        msg.joint_velocities = [
            0.1 * math.cos(self.counter * 0.1),           # d/dt of Joint 1
            0.1 * math.cos(self.counter * 0.1 + math.pi/3), # d/dt of Joint 2
            0.1 * math.cos(self.counter * 0.1 + 2*math.pi/3), # d/dt of Joint 3
            0.1 * math.cos(self.counter * 0.1 + math.pi),     # d/dt of Joint 4
            0.1 * math.cos(self.counter * 0.1 + 4*math.pi/3), # d/dt of Joint 5
            0.1 * math.cos(self.counter * 0.1 + 5*math.pi/3)  # d/dt of Joint 6
        ]

        # Simulate battery level decreasing over time
        msg.battery_level = max(100.0 - (self.counter * 0.01), 20.0)

        # Set status based on counter (cycling through states)
        if self.counter % 30 < 10:
            msg.status = RobotState.IDLE
        elif self.counter % 30 < 20:
            msg.status = RobotState.MOVING
        else:
            msg.status = RobotState.ERROR  # Simulate occasional errors

        self.publisher.publish(msg)

        # Log some key information
        self.get_logger().info(
            f'Published robot state - Joints: {len(msg.joint_positions)}, '
            f'Battery: {msg.battery_level:.1f}%, Status: {msg.status}'
        )

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    state_publisher = RobotStatePublisher()

    try:
        rclpy.spin(state_publisher)
    except KeyboardInterrupt:
        state_publisher.get_logger().info('Robot state publisher stopped')
    finally:
        state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 9: Create Subscriber with Custom Message

Create a subscriber node that receives our custom RobotState message:

Create the subscriber file:
```bash
touch ~/ros2_ws/src/robot_state_publisher/robot_state_publisher/state_subscriber.py
```

Add the following code to `~/ros2_ws/src/robot_state_publisher/robot_state_publisher/state_subscriber.py`:

```python
#!/usr/bin/env python3
"""
Robot state subscriber - subscribes to custom RobotState messages
"""

import rclpy
from rclpy.node import Node

# Import our custom message
from lab02_custom_messages.msg import RobotState


class RobotStateSubscriber(Node):

    def __init__(self):
        super().__init__('robot_state_subscriber')

        # Create subscription to our custom RobotState message
        self.subscription = self.create_subscription(
            RobotState,
            'robot_state',
            self.state_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Robot state subscriber started')

    def state_callback(self, msg):
        # Log the received robot state
        self.get_logger().info(f'Received robot state:')
        self.get_logger().info(f'  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        self.get_logger().info(f'  Frame: {msg.header.frame_id}')
        self.get_logger().info(f'  Joints: {len(msg.joint_positions)} positions, {len(msg.joint_velocities)} velocities')
        self.get_logger().info(f'  Battery: {msg.battery_level}%')

        # Decode status
        status_names = {RobotState.IDLE: 'IDLE', RobotState.MOVING: 'MOVING', RobotState.ERROR: 'ERROR'}
        status_name = status_names.get(msg.status, f'UNKNOWN({msg.status})')
        self.get_logger().info(f'  Status: {status_name}')

        # Log first few joint positions for brevity
        if len(msg.joint_positions) > 0:
            positions_str = ', '.join([f'{pos:.3f}' for pos in msg.joint_positions[:3]])
            self.get_logger().info(f'  Joint positions (first 3): [{positions_str}...]')

        self.get_logger().info('---')  # Separator for readability


def main(args=None):
    rclpy.init(args=args)

    state_subscriber = RobotStateSubscriber()

    try:
        rclpy.spin(state_subscriber)
    except KeyboardInterrupt:
        state_subscriber.get_logger().info('Robot state subscriber stopped')
    finally:
        state_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 10: Update Publisher Package Configuration

Edit the `setup.py` file for the publisher package:

Edit `~/ros2_ws/src/robot_state_publisher/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'robot_state_publisher'

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
    description='Robot state publisher and subscriber using custom messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = robot_state_publisher.state_publisher:main',
            'state_subscriber = robot_state_publisher.state_subscriber:main',
        ],
    },
)
```

---

## Step 11: Build Both Packages

```bash
cd ~/ros2_ws
colcon build --packages-select lab02_custom_messages robot_state_publisher
source ~/ros2_ws/install/setup.bash
```

---

## Step 12: Test Custom Messages

Run the publisher and subscriber to test the custom messages:

**Terminal 1 - State Publisher**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher state_publisher
```

**Terminal 2 - State Subscriber** (open a new terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher state_subscriber
```

You should see the publisher sending RobotState messages and the subscriber receiving and displaying them.

---

## Step 13: Use Command Line Tools with Custom Messages

ROS 2 command line tools work with custom messages too:

```bash
# Echo the custom message topic
ros2 topic echo /robot_state lab02_custom_messages/msg/RobotState
```

```bash
# Get info about the topic
ros2 topic info /robot_state
```

```bash
# Check the message definition
ros2 interface show lab02_custom_messages/msg/RobotState
```

---

## Step 14: Advanced Message Types

ROS 2 supports various field types in messages:

### Basic Types
```
bool
byte
char
float32, float64
int8, uint8, int16, uint16, int32, uint32, int64, uint64
string
wstring
```

### Array Types
```
type[] variable_name    # Unbounded array
type[10] variable_name # Fixed-size array
```

### Nested Messages
```
std_msgs/Header header
geometry_msgs/Point position
sensor_msgs/JointState joint_state
```

### Constants
```
uint8 CONSTANT_NAME = value
```

---

## Step 15: Message Validation Test

Let's create a simple test to validate that our message works correctly. Create a test file:

```bash
mkdir -p ~/ros2_ws/src/lab02_custom_messages/test
touch ~/ros2_ws/src/lab02_custom_messages/test/test_msg_build.cpp
```

Add the following code to `~/ros2_ws/src/lab02_custom_messages/test/test_msg_build.cpp`:

```cpp
// Simple test to validate that the custom message can be included and used
#include <gtest/gtest.h>
#include "lab02_custom_messages/msg/robot_state.hpp"

TEST(CustomMessageTest, MessageCreation) {
    // Create a RobotState message
    lab02_custom_messages::msg::RobotState msg;

    // Set some values
    msg.battery_level = 85.5f;
    msg.status = lab02_custom_messages::msg::RobotState::MOVING;

    // Verify values are set correctly
    EXPECT_FLOAT_EQ(msg.battery_level, 85.5f);
    EXPECT_EQ(msg.status, lab02_custom_messages::msg::RobotState::MOVING);

    // Test constants
    EXPECT_EQ(lab02_custom_messages::msg::RobotState::IDLE, 0);
    EXPECT_EQ(lab02_custom_messages::msg::RobotState::MOVING, 1);
    EXPECT_EQ(lab02_custom_messages::msg::RobotState::ERROR, 2);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

Update the CMakeLists.txt to include the test:

Add these lines to `~/ros2_ws/src/lab02_custom_messages/CMakeLists.txt` before `ament_package()`:

```cmake
find_package(ament_cmake_gtest REQUIRED)

# Add test
ament_add_gtest(test_msg_build test/test_msg_build.cpp)
if(TARGET test_msg_build)
  target_link_libraries(test_msg_build)
endif()
```

---

## Step 16: Build and Run Tests

```bash
cd ~/ros2_ws
colcon build --packages-select lab02_custom_messages
source ~/ros2_ws/install/setup.bash

# Run the tests
colcon test --packages-select lab02_custom_messages
```

---

## Step 17: Best Practices for Custom Messages

### Message Design Guidelines

1. **Use Standard Headers**: Always include `std_msgs/Header header` for timestamp and frame info
2. **Descriptive Names**: Use clear, descriptive field names
3. **Appropriate Types**: Choose the right data type (float32 vs float64, bounded vs unbounded arrays)
4. **Validation**: Define constants for enum-like fields
5. **Documentation**: Comment your message definitions

### Performance Considerations

- **Array Sizes**: Use bounded arrays `[N]` when size is known and fixed
- **Data Types**: Use float32 instead of float64 when precision allows
- **Message Size**: Keep messages small for better network performance
- **Frequency**: Consider publish frequency vs. message size trade-off

### Common Message Patterns

**Sensor Data**:
```
std_msgs/Header header
float64[] data
string sensor_name
float32 confidence
```

**Robot State**:
```
std_msgs/Header header
float64[] joint_positions
float64[] joint_velocities
float64[] joint_efforts
uint8[] joint_statuses
```

**Command**:
```
std_msgs/Header header
float64[] target_positions
float64[] target_velocities
float64 max_velocity
float64 tolerance
```

---

## Step 18: Troubleshooting Common Issues

### Issue 1: Message Not Found After Build

**Symptoms**: `ModuleNotFoundError` or `ros2 interface show` fails

**Solutions**:
```bash
# Make sure to source the workspace after building
source ~/ros2_ws/install/setup.bash

# Check if message was generated
ls ~/ros2_ws/install/lab02_custom_messages/share/lab02_custom_messages/msg/
```

### Issue 2: CMake Build Errors

**Symptoms**: Errors related to `rosidl_generate_interfaces`

**Solutions**:
```bash
# Clean build
rm -rf ~/ros2_ws/build/lab02_custom_messages/
rm -rf ~/ros2_ws/install/lab02_custom_messages/

# Rebuild
cd ~/ros2_ws
colcon build --packages-select lab02_custom_messages
```

### Issue 3: Import Errors in Python

**Symptoms**: `ImportError` when trying to import custom message

**Solutions**:
- Make sure the message package is built: `colcon build --packages-select lab02_custom_messages`
- Ensure the package is in your PYTHONPATH: `source ~/ros2_ws/install/setup.bash`
- Check that the package.xml includes proper dependencies

---

## Lab Summary

In this lab, you've successfully:

✅ **Defined a custom RobotState message** with header, joint arrays, battery level, and status
✅ **Configured rosidl_generate_interfaces** to generate Python and C++ code for the message
✅ **Created publisher and subscriber nodes** that use the custom message
✅ **Validated message generation** with `ros2 interface` commands
✅ **Implemented proper message design patterns** with headers and constants

### Key Takeaways

- **Custom messages** allow you to define domain-specific data structures
- **Message definition syntax** is simple but powerful for complex data
- **rosidl_generate_interfaces** automatically creates language bindings
- **Header fields** provide essential metadata (timestamp, frame_id)
- **Constants** make status fields more readable and maintainable

---

## Next Steps

Now that you understand custom messages, you're ready to explore:

- **Lab 3**: Services for request-response communication patterns
- **Lab 4**: Actions for long-running tasks with feedback
- **Lab 5**: URDF fundamentals for robot modeling

**Continue to [Lab 3: Services](./lab03-services.md)**

---
**Previous**: [Lab 1: Talker/Listener](./lab01-talker-listener.md) | **Next**: [Lab 3: Services](./lab03-services.md)