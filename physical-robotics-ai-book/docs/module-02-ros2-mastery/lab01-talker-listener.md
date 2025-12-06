---
title: Lab 1 - Hello ROS 2 World
sidebar_label: Lab 1 - Hello ROS 2 World
sidebar_position: 4
description: Create your first publisher and subscriber nodes in Python, understand topics and message passing, visualize the node graph
---

# Lab 1: Hello ROS 2 World

## Overview

In this lab, you'll create your first ROS 2 publisher and subscriber nodes using Python. You'll understand the fundamental concepts of topics and message passing, and visualize the node graph with `rqt_graph`. This is the foundation of all ROS 2 communication.

**Duration**: 1 hour

**Learning Objectives**:
- ✅ Create a simple publisher node that sends messages
- ✅ Create a subscriber node that receives messages
- ✅ Understand the publisher-subscriber pattern
- ✅ Use ROS 2 command-line tools to examine nodes
- ✅ Visualize the node graph with `rqt_graph`

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **ROS 2 Humble installed** and sourced in your terminal
```bash
source /opt/ros/humble/setup.bash
```

✅ **A working ROS 2 workspace** (created in Module 01)
```bash
# Verify workspace exists
ls -la ~/ros2_ws/src/
```

✅ **Basic Python knowledge** (functions, classes, imports)

---

## Step 1: Create a New Package

First, let's create a package for our lab exercises:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python hello_ros2_lab --dependencies rclpy std_msgs
```

This creates a Python-based package called `hello_ros2_lab` with dependencies on `rclpy` (ROS 2 Python client library) and `std_msgs` (standard message types).

**Expected Output**:
```
going to create a new package
package name: hello_ros2_lab
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['user <user@todo.todo>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: ['rclpy', 'std_msgs']
creating folder ./hello_ros2_lab
creating folder ./hello_ros2_lab/hello_ros2_lab
creating folder ./hello_ros2_lab/test
creating file ./hello_ros2_lab/package.xml
creating file ./hello_ros2_lab/setup.py
creating file ./hello_ros2_lab/setup.cfg
creating file ./hello_ros2_lab/README.md
creating file ./hello_ros2_lab/hello_ros2_lab/__init__.py
```

---

## Step 2: Create the Publisher Node

Now let's create our first publisher node that will send "Hello World" messages.

Create the publisher file:
```bash
touch ~/ros2_ws/src/hello_ros2_lab/hello_ros2_lab/talker.py
```

Add the following code to `hello_ros2_lab/hello_ros2_lab/talker.py`:

```python
#!/usr/bin/env python3
"""
Simple talker demo that published std_msgs/String messages
to the 'chatter' topic
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    talker = TalkerNode()

    rclpy.spin(talker)

    # Destroy the node explicitly
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Code Breakdown**:

1. **Node Inheritance**: `TalkerNode` inherits from `rclpy.node.Node`
2. **Publisher Creation**: `create_publisher()` creates a publisher for the `chatter` topic
3. **Timer**: Creates a timer that calls `timer_callback` every 0.5 seconds
4. **Message Publishing**: In the callback, creates a `String` message and publishes it
5. **Logging**: Uses `get_logger().info()` to log published messages

---

## Step 3: Create the Subscriber Node

Now let's create a subscriber that will listen to the messages from our publisher.

Create the subscriber file:
```bash
touch ~/ros2_ws/src/hello_ros2_lab/hello_ros2_lab/listener.py
```

Add the following code to `hello_ros2_lab/hello_ros2_lab/listener.py`:

```python
#!/usr/bin/env python3
"""
Simple talker demo that published std_msgs/String messages
to the 'chatter' topic
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    listener = ListenerNode()

    rclpy.spin(listener)

    # Destroy the node explicitly
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Code Breakdown**:

1. **Subscription Creation**: `create_subscription()` creates a subscription to the `chatter` topic
2. **Callback Function**: `listener_callback` is called whenever a message arrives
3. **Message Handling**: The callback receives the message and logs its content

---

## Step 4: Make Python Files Executable

Set the execute permissions for both Python files:

```bash
chmod +x ~/ros2_ws/src/hello_ros2_lab/hello_ros2_lab/talker.py
chmod +x ~/ros2_ws/src/hello_ros2_lab/hello_ros2_lab/listener.py
```

---

## Step 5: Update Package Configuration

We need to make our Python scripts available as ROS 2 executables. Edit the `setup.py` file:

Edit `~/ros2_ws/src/hello_ros2_lab/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'hello_ros2_lab'

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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = hello_ros2_lab.talker:main',
            'listener = hello_ros2_lab.listener:main',
        ],
    },
)
```

---

## Step 6: Build the Package

Build your workspace to make the new executables available:

```bash
cd ~/ros2_ws
colcon build --packages-select hello_ros2_lab
```

**Expected Output**:
```
Starting >>> hello_ros2_lab
Finished <<< hello_ros2_lab [2.35s]

Summary: 1 package finished [2.65s]
```

Source the workspace to make the executables available:
```bash
source ~/ros2_ws/install/setup.bash
```

---

## Step 7: Run the Publisher and Subscriber

Now let's run our nodes in separate terminals:

**Terminal 1 - Publisher**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run hello_ros2_lab talker
```

**Terminal 2 - Subscriber** (open a new terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run hello_ros2_lab listener
```

**Expected Output**:

Publisher terminal:
```
[INFO] [1678882650.123456789] [talker]: Publishing: "Hello World: 0"
[INFO] [1678882650.623456789] [talker]: Publishing: "Hello World: 1"
[INFO] [1678882651.123456789] [talker]: Publishing: "Hello World: 2"
...
```

Subscriber terminal:
```
[INFO] [1678882650.156789012] [listener]: I heard: "Hello World: 0"
[INFO] [1678882650.656789012] [listener]: I heard: "Hello World: 1"
[INFO] [1678882651.156789012] [listener]: I heard: "Hello World: 2"
...
```

**Congratulations!** You've successfully created your first ROS 2 publisher-subscriber pair.

---

## Step 8: Examine the Node Graph

Now let's use ROS 2 command-line tools to examine our running nodes:

**In a third terminal**:
```bash
# List all running nodes
ros2 node list
```

**Expected Output**:
```
/listener
/talker
```

```bash
# List topics
ros2 topic list
```

**Expected Output**:
```
/chatter
/parameter_events
/rosout
```

```bash
# Get info about the chatter topic
ros2 topic info /chatter
```

**Expected Output**:
```
Type: std_msgs/msg/String

Publisher count: 1
Subscription count: 1
```

---

## Step 9: Visualize with rqt_graph

ROS 2 comes with visualization tools to help you understand the node graph. Install and run `rqt`:

```bash
# Install rqt if not already installed
sudo apt update
sudo apt install ros-humble-rqt ros-humble-rqt-graph
```

Run rqt_graph:
```bash
rqt_graph
```

This opens a GUI showing the node graph with:
- `/talker` node publishing to `/chatter` topic
- `/listener` node subscribing to `/chatter` topic

**Node Graph Visualization**:
```mermaid
graph LR
    A[Talker Node] -->|/chatter| B[Listener Node]

    style A fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style B fill:#50C878,stroke:#2E7D4E,color:#fff
```

---

## Step 10: Advanced Topic Commands

Let's explore more ROS 2 topic commands:

```bash
# Echo messages on the chatter topic (like our subscriber)
ros2 topic echo /chatter
```

```bash
# Check the frequency of messages on chatter
ros2 topic hz /chatter
```

```bash
# Check the type of message on chatter
ros2 topic type /chatter
```

**Expected Output for type**:
```
std_msgs/msg/String
```

---

## Understanding the Publisher-Subscriber Pattern

### Key Concepts

1. **Decoupling**: Publisher and subscriber don't need to know about each other
2. **Asynchronous**: Publisher sends messages regardless of subscriber status
3. **Many-to-Many**: Multiple publishers can send to one topic, multiple subscribers can listen to one topic
4. **Topic Names**: Follows the format `/namespace/topic_name`

### Message Flow

```
[Talker Node] --(String message)--> [/chatter topic] --(String message)--> [Listener Node]
     |                                       |                                    |
   Publisher                           Message Buffer                      Subscriber
```

---

## Common Issues and Troubleshooting

### Issue 1: Permission Denied Error

**Symptoms**: `PermissionError: [Errno 13] Permission denied`

**Solution**: Make sure your Python files are executable:
```bash
chmod +x ~/ros2_ws/src/hello_ros2_lab/hello_ros2_lab/*.py
```

### Issue 2: Module Not Found

**Symptoms**: `ModuleNotFoundError: No module named 'hello_ros2_lab'`

**Solution**: Make sure you sourced the workspace after building:
```bash
source ~/ros2_ws/install/setup.bash
```

### Issue 3: Nodes Can't Communicate

**Symptoms**: Publisher runs but subscriber doesn't receive messages

**Solutions**:
1. Check that both terminals have sourced the workspace
2. Verify nodes are running: `ros2 node list`
3. Check topic connection: `ros2 topic info /chatter`

---

## Lab Summary

In this lab, you've successfully:

✅ **Created a ROS 2 package** with Python dependencies
✅ **Built a publisher node** that sends String messages to `/chatter`
✅ **Built a subscriber node** that receives messages from `/chatter`
✅ **Ran both nodes** and observed successful communication
✅ **Used ROS 2 command-line tools** to examine nodes and topics
✅ **Visualized the node graph** with rqt_graph

### Key Takeaways

- **Topics** enable asynchronous, decoupled communication between nodes
- **Publishers** send messages without knowing who receives them
- **Subscribers** receive messages without knowing who sent them
- **Message types** define the structure of data passed between nodes
- **ROS 2 tools** help you debug and understand your system's architecture

---

## Next Steps

Now that you understand the publisher-subscriber pattern, you're ready to explore:

- **Lab 2**: Understanding DDS middleware and how ROS 2 uses it
- **Lab 3**: Quality of Service (QoS) profiles for different communication needs
- **Lab 4**: Services for request-response communication patterns

**Continue to [Lab 2: Custom Messages](./lab02-custom-messages.md)**

---
**Previous**: [Workspace Setup](./workspace-setup.md) | **Next**: [Lab 2: Custom Messages](./lab02-custom-messages.md)