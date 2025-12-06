---
title: Lab 6 - TF2 Broadcasting
sidebar_label: Lab 6 - TF2 Broadcasting
sidebar_position: 11
description: Implement TF2 transform broadcasting for robot coordinate frames, create static and dynamic transform broadcasters, and validate coordinate frame relationships
---

# Lab 6: TF2 Broadcasting

## Overview

In this lab, you'll implement TF2 transform broadcasting for robot coordinate frames. You'll create both static and dynamic transform broadcasters, understand the difference between them, and validate coordinate frame relationships. TF2 broadcasting is essential for establishing spatial relationships between robot components, sensors, and the environment, enabling proper sensor fusion and navigation.

**Duration**: 1.5 hours

**Learning Objectives**:
- ✅ Create static transform broadcasters for fixed frame relationships
- ✅ Implement dynamic transform broadcasters for changing frame relationships
- ✅ Use TransformBroadcaster and StaticTransformBroadcaster APIs correctly
- ✅ Validate TF2 tree structure with `view_frames` command
- ✅ Implement coordinate frame relationships for 5-DOF humanoid robot

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **Completed Lab 1-5** - All previous labs covering ROS 2 basics through URDF
✅ **ROS 2 Humble installed** with all standard packages
✅ **Basic understanding** of coordinate frames and transformations
✅ **Python programming skills** for implementing TF2 broadcasters
✅ **Completed TF2 fundamentals** - Understanding of frame relationships

---

## Step 1: Understanding TF2 Broadcasting Concepts

### What is TF2 Broadcasting?

**TF2 broadcasting** is the process of publishing coordinate frame transformations to establish spatial relationships between different parts of a robot or between the robot and its environment. TF2 allows you to:

- **Track positions** of robot parts relative to each other over time
- **Transform data** between different coordinate frames seamlessly
- **Maintain temporal relationships** for time-consistent transformations
- **Fuse sensor data** from multiple coordinate systems

### Static vs Dynamic Transforms

| Transform Type | Purpose | Example | Frequency |
|----------------|---------|---------|-----------|
| **Static** | Fixed relationship | Sensor mount on robot | Published once |
| **Dynamic** | Changing relationship | Robot movement, joint positions | Published continuously |

### TF2 Data Structure

A transform contains:
- **Translation**: 3D position (x, y, z) from parent to child frame
- **Rotation**: 4D quaternion (x, y, z, w) representing orientation
- **Timestamp**: When the transform was valid
- **Parent/Child**: Frame relationship (parent → child)

---

## Step 2: Creating a TF2 Broadcasting Package

First, let's create a package specifically for our TF2 examples:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python tf2_broadcasting_examples --dependencies rclpy tf2_ros geometry_msgs std_msgs
```

Create the necessary directory:
```bash
mkdir -p ~/ros2_ws/src/tf2_broadcasting_examples/tf2_broadcasting_examples
```

---

## Step 3: Creating Static Transform Broadcaster

Static transforms represent fixed relationships between frames (e.g., sensor mounts). Let's create a static transform broadcaster:

```bash
touch ~/ros2_ws/src/tf2_broadcasting_examples/tf2_broadcasting_examples/static_broadcaster.py
```

Add the following content to `~/ros2_ws/src/tf2_broadcasting_examples/tf2_broadcasting_examples/static_broadcaster.py`:

```python
#!/usr/bin/env python3
"""
Static Transform Broadcaster - broadcasts fixed frame relationships for a humanoid robot
"""

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time


class StaticBroadcaster(Node):

    def __init__(self):
        super().__init__('static_broadcaster')

        # Create static transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms for a humanoid robot
        self.publish_static_transforms()

        self.get_logger().info('Static transform broadcaster initialized')

    def publish_static_transforms(self):
        """
        Publish static transforms between robot frames
        """
        transforms = []

        # Base link to camera frame (camera mounted on robot head)
        camera_transform = TransformStamped()
        camera_transform.header.stamp = self.get_clock().now().to_msg()
        camera_transform.header.frame_id = 'head_link'
        camera_transform.child_frame_id = 'camera_frame'

        # Mount camera 5cm forward, 2cm up, 0cm left/right from head center
        camera_transform.transform.translation.x = 0.05
        camera_transform.transform.translation.y = 0.0
        camera_transform.transform.translation.z = 0.02

        # Camera looks forward (identity rotation - Z forward, X right, Y down in optical frame)
        camera_transform.transform.rotation.w = 1.0
        camera_transform.transform.rotation.x = 0.0
        camera_transform.transform.rotation.y = 0.0
        camera_transform.transform.rotation.z = 0.0

        transforms.append(camera_transform)

        # Base link to IMU frame (IMU mounted in robot torso)
        imu_transform = TransformStamped()
        imu_transform.header.stamp = self.get_clock().now().to_msg()
        imu_transform.header.frame_id = 'torso_link'
        imu_transform.child_frame_id = 'imu_frame'

        # Mount IMU at robot center, 10cm above base
        imu_transform.transform.translation.x = 0.0
        imu_transform.transform.translation.y = 0.0
        imu_transform.transform.translation.z = 0.1

        # IMU aligned with robot frame
        imu_transform.transform.rotation.w = 1.0
        imu_transform.transform.rotation.x = 0.0
        imu_transform.transform.rotation.y = 0.0
        imu_transform.transform.rotation.z = 0.0

        transforms.append(imu_transform)

        # Left arm base (shoulder) - relative to torso
        left_shoulder_transform = TransformStamped()
        left_shoulder_transform.header.stamp = self.get_clock().now().to_msg()
        left_shoulder_transform.header.frame_id = 'torso_link'
        left_shoulder_transform.child_frame_id = 'left_shoulder_link'

        # Position left shoulder: 15cm to the left, 30cm up from torso center
        left_shoulder_transform.transform.translation.x = 0.0
        left_shoulder_transform.transform.translation.y = 0.15
        left_shoulder_transform.transform.translation.z = 0.3

        # Shoulder aligned with torso
        left_shoulder_transform.transform.rotation.w = 1.0
        left_shoulder_transform.transform.rotation.x = 0.0
        left_shoulder_transform.transform.rotation.y = 0.0
        left_shoulder_transform.transform.rotation.z = 0.0

        transforms.append(left_shoulder_transform)

        # Right arm base (shoulder) - relative to torso
        right_shoulder_transform = TransformStamped()
        right_shoulder_transform.header.stamp = self.get_clock().now().to_msg()
        right_shoulder_transform.header.frame_id = 'torso_link'
        right_shoulder_transform.child_frame_id = 'right_shoulder_link'

        # Position right shoulder: 15cm to the right, 30cm up from torso center
        right_shoulder_transform.transform.translation.x = 0.0
        right_shoulder_transform.transform.translation.y = -0.15
        right_shoulder_transform.transform.translation.z = 0.3

        # Shoulder aligned with torso
        right_shoulder_transform.transform.rotation.w = 1.0
        right_shoulder_transform.transform.rotation.x = 0.0
        right_shoulder_transform.transform.rotation.y = 0.0
        right_shoulder_transform.transform.rotation.z = 0.0

        transforms.append(right_shoulder_transform)

        # Left leg base (hip) - relative to base
        left_hip_transform = TransformStamped()
        left_hip_transform.header.stamp = self.get_clock().now().to_msg()
        left_hip_transform.header.frame_id = 'base_link'
        left_hip_transform.child_frame_id = 'left_hip_link'

        # Position left hip: 8cm to the left from center
        left_hip_transform.transform.translation.x = 0.0
        left_hip_transform.transform.translation.y = 0.08
        left_hip_transform.transform.translation.z = 0.0

        # Hip aligned with base
        left_hip_transform.transform.rotation.w = 1.0
        left_hip_transform.transform.rotation.x = 0.0
        left_hip_transform.transform.rotation.y = 0.0
        left_hip_transform.transform.rotation.z = 0.0

        transforms.append(left_hip_transform)

        # Right leg base (hip) - relative to base
        right_hip_transform = TransformStamped()
        right_hip_transform.header.stamp = self.get_clock().now().to_msg()
        right_hip_transform.header.frame_id = 'base_link'
        right_hip_transform.child_frame_id = 'right_hip_link'

        # Position right hip: 8cm to the right from center
        right_hip_transform.transform.translation.x = 0.0
        right_hip_transform.transform.translation.y = -0.08
        right_hip_transform.transform.translation.z = 0.0

        # Hip aligned with base
        right_hip_transform.transform.rotation.w = 1.0
        right_hip_transform.transform.rotation.x = 0.0
        right_hip_transform.transform.rotation.y = 0.0
        right_hip_transform.transform.rotation.z = 0.0

        transforms.append(right_hip_transform)

        # Publish all static transforms
        self.tf_static_broadcaster.sendTransform(transforms)

        self.get_logger().info(f'Published {len(transforms)} static transforms')


def main(args=None):
    rclpy.init(args=args)

    static_broadcaster = StaticBroadcaster()

    try:
        # Keep the node alive to maintain static transforms
        rclpy.spin(static_broadcaster)
    except KeyboardInterrupt:
        static_broadcaster.get_logger().info('Static broadcaster stopped')
    finally:
        static_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 4: Creating Dynamic Transform Broadcaster

Dynamic transforms change over time (e.g., robot motion, joint angles). Let's create a dynamic transform broadcaster:

```bash
touch ~/ros2_ws/src/tf2_broadcasting_examples/tf2_broadcasting_examples/dynamic_broadcaster.py
```

Add the following content to `~/ros2_ws/src/tf2_broadcasting_examples/tf2_broadcasting_examples/dynamic_broadcaster.py`:

```python
#!/usr/bin/env python3
"""
Dynamic Transform Broadcaster - broadcasts changing frame relationships for a humanoid robot
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import math


class DynamicBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_broadcaster')

        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to joint states to get current joint positions
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Timer to broadcast transforms at 50 Hz
        self.timer = self.create_timer(0.02, self.broadcast_dynamic_transforms)

        # Store joint positions
        self.joint_positions = {}
        self.time_counter = 0.0

        # Initialize joint positions for simulation
        self.initialize_joints()

        self.get_logger().info('Dynamic transform broadcaster initialized')

    def initialize_joints(self):
        """
        Initialize joint positions for simulation
        """
        # Initialize all humanoid joints with zero positions
        joint_names = [
            'left_shoulder_pitch', 'left_elbow', 'left_wrist_roll',
            'right_shoulder_pitch', 'right_elbow', 'right_wrist_roll',
            'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_pitch', 'right_knee', 'right_ankle'
        ]

        for joint_name in joint_names:
            self.joint_positions[joint_name] = 0.0

    def joint_state_callback(self, msg):
        """
        Callback for joint state messages
        """
        # Update joint positions from message
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

        # Log joint updates occasionally
        if int(self.time_counter) % 10 == 0:  # Every 10 seconds
            self.get_logger().info(f'Updated {len(msg.name)} joint positions')

    def broadcast_dynamic_transforms(self):
        """
        Broadcast dynamic transforms between frames
        """
        # Update time counter
        self.time_counter += 0.02  # Increment by timer period

        # Get current time for transform timestamp
        current_time = self.get_clock().now()

        # Simulate some joint movements if not receiving real joint states
        self.simulate_joint_movements()

        # Transform: base_link -> torso_link (for this example, assume base and torso are the same)
        base_to_torso = TransformStamped()
        base_to_torso.header.stamp = current_time.to_msg()
        base_to_torso.header.frame_id = 'base_link'
        base_to_torso.child_frame_id = 'torso_link'

        # For now, keep torso at same position as base (identity transform)
        base_to_torso.transform.translation.x = 0.0
        base_to_torso.transform.translation.y = 0.0
        base_to_torso.transform.translation.z = 0.6  # Torso is 60cm above ground
        base_to_torso.transform.rotation.w = 1.0
        base_to_torso.transform.rotation.x = 0.0
        base_to_torso.transform.rotation.y = 0.0
        base_to_torso.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(base_to_torso)

        # Transform: torso_link -> head_link (with neck movement)
        torso_to_head = TransformStamped()
        torso_to_head.header.stamp = current_time.to_msg()
        torso_to_head.header.frame_id = 'torso_link'
        torso_to_head.child_frame_id = 'head_link'

        # Position head: 30cm above torso
        torso_to_head.transform.translation.x = 0.0
        torso_to_head.transform.translation.y = 0.0
        torso_to_head.transform.translation.z = 0.3

        # Simulate head turning (slow yaw motion)
        head_yaw = math.sin(self.time_counter * 0.3) * 0.3  # ±17 degrees
        cy = math.cos(head_yaw / 2.0)
        sy = math.sin(head_yaw / 2.0)
        torso_to_head.transform.rotation.w = cy
        torso_to_head.transform.rotation.x = 0.0
        torso_to_head.transform.rotation.y = 0.0
        torso_to_head.transform.rotation.z = sy

        self.tf_broadcaster.sendTransform(torso_to_head)

        # Left arm transforms
        # Shoulder to upper arm (with joint movement)
        shoulder_to_upper_arm = TransformStamped()
        shoulder_to_upper_arm.header.stamp = current_time.to_msg()
        shoulder_to_upper_arm.header.frame_id = 'left_shoulder_link'
        shoulder_to_upper_arm.child_frame_id = 'left_upper_arm_link'

        # Fixed position (upper arm connected to shoulder)
        shoulder_to_upper_arm.transform.translation.x = 0.0
        shoulder_to_upper_arm.transform.translation.y = 0.0
        shoulder_to_upper_arm.transform.translation.z = 0.15  # 15cm long upper arm

        # Get shoulder joint position or simulate
        shoulder_angle = self.joint_positions.get('left_shoulder_pitch',
                                                math.sin(self.time_counter * 0.5) * 0.5)  # Simulated movement

        # Apply shoulder rotation (pitch)
        cy = math.cos(shoulder_angle / 2.0)
        sx = math.sin(shoulder_angle / 2.0)
        shoulder_to_upper_arm.transform.rotation.w = cy
        shoulder_to_upper_arm.transform.rotation.x = sx
        shoulder_to_upper_arm.transform.rotation.y = 0.0
        shoulder_to_upper_arm.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(shoulder_to_upper_arm)

        # Upper arm to forearm (elbow joint)
        upper_arm_to_forearm = TransformStamped()
        upper_arm_to_forearm.header.stamp = current_time.to_msg()
        upper_arm_to_forearm.header.frame_id = 'left_upper_arm_link'
        upper_arm_to_forearm.child_frame_id = 'left_forearm_link'

        # Fixed position (forearm connected to upper arm)
        upper_arm_to_forearm.transform.translation.x = 0.0
        upper_arm_to_forearm.transform.translation.y = 0.0
        upper_arm_to_forearm.transform.translation.z = 0.15  # 15cm long forearm

        # Get elbow joint position or simulate
        elbow_angle = self.joint_positions.get('left_elbow',
                                             math.sin(self.time_counter * 0.7) * 0.8)  # Simulated movement

        # Apply elbow rotation (pitch)
        cy = math.cos(elbow_angle / 2.0)
        sx = math.sin(elbow_angle / 2.0)
        upper_arm_to_forearm.transform.rotation.w = cy
        upper_arm_to_forearm.transform.rotation.x = sx
        upper_arm_to_forearm.transform.rotation.y = 0.0
        upper_arm_to_forearm.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(upper_arm_to_forearm)

        # Forearm to hand (wrist joint)
        forearm_to_hand = TransformStamped()
        forearm_to_hand.header.stamp = current_time.to_msg()
        forearm_to_hand.header.frame_id = 'left_forearm_link'
        forearm_to_hand.child_frame_id = 'left_hand_link'

        # Fixed position (hand connected to forearm)
        forearm_to_hand.transform.translation.x = 0.0
        forearm_to_hand.transform.translation.y = 0.0
        forearm_to_hand.transform.translation.z = 0.1  # 10cm to hand

        # Get wrist joint position or simulate
        wrist_angle = self.joint_positions.get('left_wrist_roll',
                                             math.sin(self.time_counter * 1.0) * 0.5)  # Simulated movement

        # Apply wrist rotation (roll)
        cy = math.cos(wrist_angle / 2.0)
        sz = math.sin(wrist_angle / 2.0)
        forearm_to_hand.transform.rotation.w = cy
        forearm_to_hand.transform.rotation.x = 0.0
        forearm_to_hand.transform.rotation.y = 0.0
        forearm_to_hand.transform.rotation.z = sz

        self.tf_broadcaster.sendTransform(forearm_to_hand)

        # Right arm transforms (similar to left but mirrored)
        # Right shoulder to upper arm
        r_shoulder_to_upper_arm = TransformStamped()
        r_shoulder_to_upper_arm.header.stamp = current_time.to_msg()
        r_shoulder_to_upper_arm.header.frame_id = 'right_shoulder_link'
        r_shoulder_to_upper_arm.child_frame_id = 'right_upper_arm_link'

        r_shoulder_to_upper_arm.transform.translation.x = 0.0
        r_shoulder_to_upper_arm.transform.translation.y = 0.0
        r_shoulder_to_upper_arm.transform.translation.z = 0.15

        r_shoulder_angle = self.joint_positions.get('right_shoulder_pitch',
                                                   math.sin(self.time_counter * 0.5 + math.pi) * 0.5)  # Opposite phase
        cy = math.cos(r_shoulder_angle / 2.0)
        sx = math.sin(r_shoulder_angle / 2.0)
        r_shoulder_to_upper_arm.transform.rotation.w = cy
        r_shoulder_to_upper_arm.transform.rotation.x = sx
        r_shoulder_to_upper_arm.transform.rotation.y = 0.0
        r_shoulder_to_upper_arm.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(r_shoulder_to_upper_arm)

        # Right upper arm to forearm
        r_upper_arm_to_forearm = TransformStamped()
        r_upper_arm_to_forearm.header.stamp = current_time.to_msg()
        r_upper_arm_to_forearm.header.frame_id = 'right_upper_arm_link'
        r_upper_arm_to_forearm.child_frame_id = 'right_forearm_link'

        r_upper_arm_to_forearm.transform.translation.x = 0.0
        r_upper_arm_to_forearm.transform.translation.y = 0.0
        r_upper_arm_to_forearm.transform.translation.z = 0.15

        r_elbow_angle = self.joint_positions.get('right_elbow',
                                                math.sin(self.time_counter * 0.7 + math.pi) * 0.8)
        cy = math.cos(r_elbow_angle / 2.0)
        sx = math.sin(r_elbow_angle / 2.0)
        r_upper_arm_to_forearm.transform.rotation.w = cy
        r_upper_arm_to_forearm.transform.rotation.x = sx
        r_upper_arm_to_forearm.transform.rotation.y = 0.0
        r_upper_arm_to_forearm.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(r_upper_arm_to_forearm)

        # Right forearm to hand
        r_forearm_to_hand = TransformStamped()
        r_forearm_to_hand.header.stamp = current_time.to_msg()
        r_forearm_to_hand.header.frame_id = 'right_forearm_link'
        r_forearm_to_hand.child_frame_id = 'right_hand_link'

        r_forearm_to_hand.transform.translation.x = 0.0
        r_forearm_to_hand.transform.translation.y = 0.0
        r_forearm_to_hand.transform.translation.z = 0.1

        r_wrist_angle = self.joint_positions.get('right_wrist_roll',
                                                math.sin(self.time_counter * 1.0 + math.pi) * 0.5)
        cy = math.cos(r_wrist_angle / 2.0)
        sz = math.sin(r_wrist_angle / 2.0)
        r_forearm_to_hand.transform.rotation.w = cy
        r_forearm_to_hand.transform.rotation.x = 0.0
        r_forearm_to_hand.transform.rotation.y = 0.0
        r_forearm_to_hand.transform.rotation.z = sz

        self.tf_broadcaster.sendTransform(r_forearm_to_hand)

        # Left leg transforms
        # Hip to thigh
        hip_to_thigh = TransformStamped()
        hip_to_thigh.header.stamp = current_time.to_msg()
        hip_to_thigh.header.frame_id = 'left_hip_link'
        hip_to_thigh.child_frame_id = 'left_thigh_link'

        hip_to_thigh.transform.translation.x = 0.0
        hip_to_thigh.transform.translation.y = 0.0
        hip_to_thigh.transform.translation.z = -0.2  # 20cm down to thigh

        hip_angle = self.joint_positions.get('left_hip_pitch',
                                           math.sin(self.time_counter * 0.2) * 0.2)
        cy = math.cos(hip_angle / 2.0)
        sx = math.sin(hip_angle / 2.0)
        hip_to_thigh.transform.rotation.w = cy
        hip_to_thigh.transform.rotation.x = sx
        hip_to_thigh.transform.rotation.y = 0.0
        hip_to_thigh.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(hip_to_thigh)

        # Thigh to shin
        thigh_to_shin = TransformStamped()
        thigh_to_shin.header.stamp = current_time.to_msg()
        thigh_to_shin.header.frame_id = 'left_thigh_link'
        thigh_to_shin.child_frame_id = 'left_shin_link'

        thigh_to_shin.transform.translation.x = 0.0
        thigh_to_shin.transform.translation.y = 0.0
        thigh_to_shin.transform.translation.z = -0.25  # 25cm down to shin

        knee_angle = self.joint_positions.get('left_knee',
                                            math.sin(self.time_counter * 0.3) * 0.5)
        cy = math.cos(knee_angle / 2.0)
        sx = math.sin(knee_angle / 2.0)
        thigh_to_shin.transform.rotation.w = cy
        thigh_to_shin.transform.rotation.x = sx
        thigh_to_shin.transform.rotation.y = 0.0
        thigh_to_shin.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(thigh_to_shin)

        # Shin to foot
        shin_to_foot = TransformStamped()
        shin_to_foot.header.stamp = current_time.to_msg()
        shin_to_foot.header.frame_id = 'left_shin_link'
        shin_to_foot.child_frame_id = 'left_foot_link'

        shin_to_foot.transform.translation.x = 0.0
        shin_to_foot.transform.translation.y = 0.0
        shin_to_foot.transform.translation.z = -0.15  # 15cm down to foot

        ankle_angle = self.joint_positions.get('left_ankle',
                                             math.sin(self.time_counter * 0.4) * 0.3)
        cy = math.cos(ankle_angle / 2.0)
        sx = math.sin(ankle_angle / 2.0)
        shin_to_foot.transform.rotation.w = cy
        shin_to_foot.transform.rotation.x = sx
        shin_to_foot.transform.rotation.y = 0.0
        shin_to_foot.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(shin_to_foot)

        # Right leg transforms (similar to left)
        # Right hip to thigh
        r_hip_to_thigh = TransformStamped()
        r_hip_to_thigh.header.stamp = current_time.to_msg()
        r_hip_to_thigh.header.frame_id = 'right_hip_link'
        r_hip_to_thigh.child_frame_id = 'right_thigh_link'

        r_hip_to_thigh.transform.translation.x = 0.0
        r_hip_to_thigh.transform.translation.y = 0.0
        r_hip_to_thigh.transform.translation.z = -0.2

        r_hip_angle = self.joint_positions.get('right_hip_pitch',
                                              math.sin(self.time_counter * 0.2 + math.pi) * 0.2)
        cy = math.cos(r_hip_angle / 2.0)
        sx = math.sin(r_hip_angle / 2.0)
        r_hip_to_thigh.transform.rotation.w = cy
        r_hip_to_thigh.transform.rotation.x = sx
        r_hip_to_thigh.transform.rotation.y = 0.0
        r_hip_to_thigh.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(r_hip_to_thigh)

        # Right thigh to shin
        r_thigh_to_shin = TransformStamped()
        r_thigh_to_shin.header.stamp = current_time.to_msg()
        r_thigh_to_shin.header.frame_id = 'right_thigh_link'
        r_thigh_to_shin.child_frame_id = 'right_shin_link'

        r_thigh_to_shin.transform.translation.x = 0.0
        r_thigh_to_shin.transform.translation.y = 0.0
        r_thigh_to_shin.transform.translation.z = -0.25

        r_knee_angle = self.joint_positions.get('right_knee',
                                               math.sin(self.time_counter * 0.3 + math.pi) * 0.5)
        cy = math.cos(r_knee_angle / 2.0)
        sx = math.sin(r_knee_angle / 2.0)
        r_thigh_to_shin.transform.rotation.w = cy
        r_thigh_to_shin.transform.rotation.x = sx
        r_thigh_to_shin.transform.rotation.y = 0.0
        r_thigh_to_shin.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(r_thigh_to_shin)

        # Right shin to foot
        r_shin_to_foot = TransformStamped()
        r_shin_to_foot.header.stamp = current_time.to_msg()
        r_shin_to_foot.header.frame_id = 'right_shin_link'
        r_shin_to_foot.child_frame_id = 'right_foot_link'

        r_shin_to_foot.transform.translation.x = 0.0
        r_shin_to_foot.transform.translation.y = 0.0
        r_shin_to_foot.transform.translation.z = -0.15

        r_ankle_angle = self.joint_positions.get('right_ankle',
                                                math.sin(self.time_counter * 0.4 + math.pi) * 0.3)
        cy = math.cos(r_ankle_angle / 2.0)
        sx = math.sin(r_ankle_angle / 2.0)
        r_shin_to_foot.transform.rotation.w = cy
        r_shin_to_foot.transform.rotation.x = sx
        r_shin_to_foot.transform.rotation.y = 0.0
        r_shin_to_foot.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(r_shin_to_foot)

        # Log transform publishing occasionally
        if int(self.time_counter * 10) % 50 == 0:  # Every 5 seconds at 10Hz
            self.get_logger().info('Published dynamic transforms for humanoid robot')

    def simulate_joint_movements(self):
        """
        Simulate joint movements when no real joint states are received
        """
        # Simulate breathing motion (torso slight movement)
        breathing_motion = math.sin(self.time_counter * 0.1) * 0.01  # Very subtle

        # Update torso position slightly to simulate breathing
        # In a real system, this would come from odometry or joint states


def main(args=None):
    rclpy.init(args=args)

    dynamic_broadcaster = DynamicBroadcaster()

    try:
        rclpy.spin(dynamic_broadcaster)
    except KeyboardInterrupt:
        dynamic_broadcaster.get_logger().info('Dynamic broadcaster stopped')
    finally:
        dynamic_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 5: Creating a Combined Broadcasting Node

Let's create a node that combines both static and dynamic broadcasting:

```bash
touch ~/ros2_ws/src/tf2_examples/tf2_examples/combined_broadcaster.py
```

Add the following content to `~/ros2_ws/src/tf2_examples/tf2_examples/combined_broadcaster.py`:

```python
#!/usr/bin/env python3
"""
Combined broadcaster - demonstrates both static and dynamic transform broadcasting
"""

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform
from builtin_interfaces.msg import Time
import math


class CombinedBroadcaster(Node):

    def __init__(self):
        super().__init__('combined_broadcaster')

        # Create both static and dynamic transform broadcasters
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish static transforms once
        self.publish_static_transforms()

        # Timer for dynamic transforms (50 Hz)
        self.timer = self.create_timer(0.02, self.broadcast_dynamic_transforms)
        self.time_counter = 0.0

        self.get_logger().info('Combined static/dynamic transform broadcaster started')

    def publish_static_transforms(self):
        """
        Publish static transforms that define fixed relationships
        """
        transforms = []

        # Robot base to various sensor frames (fixed mounts)
        # Camera frame (mounted on head)
        camera_transform = TransformStamped()
        camera_transform.header.stamp = self.get_clock().now().to_msg()
        camera_transform.header.frame_id = 'head_link'
        camera_transform.child_frame_id = 'camera_frame'

        # Mount camera 10cm forward, 5cm up from head center
        camera_transform.transform.translation.x = 0.1
        camera_transform.transform.translation.y = 0.0
        camera_transform.transform.translation.z = 0.05

        # Camera looking forward (optical frame: Z forward, X right, Y down)
        # For a camera mounted on head looking forward, we want identity rotation
        camera_transform.transform.rotation.w = 1.0
        camera_transform.transform.rotation.x = 0.0
        camera_transform.transform.rotation.y = 0.0
        camera_transform.transform.rotation.z = 0.0

        transforms.append(camera_transform)

        # IMU frame (mounted in torso)
        imu_transform = TransformStamped()
        imu_transform.header.stamp = self.get_clock().now().to_msg()
        imu_transform.header.frame_id = 'torso_link'
        imu_transform.child_frame_id = 'imu_frame'

        # Mount IMU at torso center, 5cm up
        imu_transform.transform.translation.x = 0.0
        imu_transform.transform.translation.y = 0.0
        imu_transform.transform.translation.z = 0.05

        # IMU aligned with robot frame
        imu_transform.transform.rotation.w = 1.0
        imu_transform.transform.rotation.x = 0.0
        imu_transform.transform.rotation.y = 0.0
        imu_transform.transform.rotation.z = 0.0

        transforms.append(imu_transform)

        # LiDAR frame (mounted on head)
        lidar_transform = TransformStamped()
        lidar_transform.header.stamp = self.get_clock().now().to_msg()
        lidar_transform.header.frame_id = 'head_link'
        lidar_transform.child_frame_id = 'lidar_frame'

        # Mount LiDAR at head center, looking forward
        lidar_transform.transform.translation.x = 0.05  # 5cm forward from head center
        lidar_transform.transform.translation.y = 0.0
        lidar_transform.transform.translation.z = 0.0

        # LiDAR aligned with forward direction
        lidar_transform.transform.rotation.w = 1.0
        lidar_transform.transform.rotation.x = 0.0
        lidar_transform.transform.rotation.y = 0.0
        lidar_transform.transform.rotation.z = 0.0

        transforms.append(lidar_transform)

        # Publish all static transforms
        self.tf_static_broadcaster.sendTransform(transforms)
        self.get_logger().info(f'Published {len(transforms)} static transforms')

    def broadcast_dynamic_transforms(self):
        """
        Broadcast dynamic transforms that change over time
        """
        # Update time counter
        self.time_counter += 0.02  # Increment by timer period (20ms)

        # Get current time for transform timestamps
        current_time = self.get_clock().now()

        # Dynamic: odom -> base_link (robot moving in a circular path)
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = current_time.to_msg()
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'

        # Circular motion: radius 2m, period 10 seconds
        radius = 2.0
        angular_velocity = 2 * math.pi / 10.0  # Complete circle every 10 seconds
        angle = self.time_counter * angular_velocity

        # Position in circular path
        odom_to_base.transform.translation.x = radius * math.cos(angle)
        odom_to_base.transform.translation.y = radius * math.sin(angle)
        odom_to_base.transform.translation.z = 0.0  # Ground level

        # Robot orientation (facing direction of movement)
        # Derivative of position gives direction of movement
        orientation_angle = angle + math.pi/2  # Perpendicular to radial vector (tangential)
        cy = math.cos(orientation_angle / 2.0)
        sy = math.sin(orientation_angle / 2.0)
        odom_to_base.transform.rotation.w = cy
        odom_to_base.transform.rotation.x = 0.0
        odom_to_base.transform.rotation.y = 0.0
        odom_to_base.transform.rotation.z = sy

        self.tf_broadcaster.sendTransform(odom_to_base)

        # Dynamic: base_link to torso (with slight body sway)
        base_to_torso = TransformStamped()
        base_to_torso.header.stamp = current_time.to_msg()
        base_to_torso.header.frame_id = 'base_link'
        base_to_torso.child_frame_id = 'torso_link'

        # Fixed height but with slight swaying motion
        base_to_torso.transform.translation.x = 0.0
        base_to_torso.transform.translation.y = 0.0
        base_to_torso.transform.translation.z = 0.6  # Torso height

        # Slight body sway (simulating walking or balance)
        sway_angle = math.sin(self.time_counter * 2.0) * 0.05  # Small sway
        cy = math.cos(sway_angle / 2.0)
        sx = math.sin(sway_angle / 2.0)
        base_to_torso.transform.rotation.w = cy
        base_to_torso.transform.rotation.x = sx
        base_to_torso.transform.rotation.y = 0.0
        base_to_torso.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(base_to_torso)

        # Dynamic: torso to head (with head movement)
        torso_to_head = TransformStamped()
        torso_to_head.header.stamp = current_time.to_msg()
        torso_to_head.header.frame_id = 'torso_link'
        torso_to_head.child_frame_id = 'head_link'

        # Fixed offset: head is 30cm above torso
        torso_to_head.transform.translation.x = 0.0
        torso_to_head.transform.translation.y = 0.0
        torso_to_head.transform.translation.z = 0.3

        # Head turning motion (looking around)
        head_yaw = math.sin(self.time_counter * 0.8) * 0.3  # Look left/right
        head_pitch = math.sin(self.time_counter * 0.5) * 0.2  # Look up/down

        # Convert Euler to quaternion (yaw-pitch-roll)
        # For small movements, we can use simplified conversion
        cy_yaw = math.cos(head_yaw / 2.0)
        sy_yaw = math.sin(head_yaw / 2.0)
        cp_pitch = math.cos(head_pitch / 2.0)
        sp_pitch = math.sin(head_pitch / 2.0)

        # Combined rotation (yaw then pitch)
        torso_to_head.transform.rotation.w = cy_yaw * cp_pitch
        torso_to_head.transform.rotation.x = -sy_yaw * sp_pitch
        torso_to_head.transform.rotation.y = cy_yaw * sp_pitch
        torso_to_head.transform.rotation.z = sy_yaw * cp_pitch

        self.tf_broadcaster.sendTransform(torso_to_head)

        # Dynamic: Left arm joints (shoulder-elbow-wrist)
        # Shoulder to upper arm
        shoulder_to_upper_arm = TransformStamped()
        shoulder_to_upper_arm.header.stamp = current_time.to_msg()
        shoulder_to_upper_arm.header.frame_id = 'left_shoulder_link'
        shoulder_to_upper_arm.child_frame_id = 'left_upper_arm_link'

        shoulder_to_upper_arm.transform.translation.x = 0.0
        shoulder_to_upper_arm.transform.translation.y = 0.0
        shoulder_to_upper_arm.transform.translation.z = 0.15  # 15cm upper arm

        # Shoulder movement (simulated)
        shoulder_pitch = math.sin(self.time_counter * 0.7) * 0.5
        cy = math.cos(shoulder_pitch / 2.0)
        sx = math.sin(shoulder_pitch / 2.0)
        shoulder_to_upper_arm.transform.rotation.w = cy
        shoulder_to_upper_arm.transform.rotation.x = sx
        shoulder_to_upper_arm.transform.rotation.y = 0.0
        shoulder_to_upper_arm.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(shoulder_to_upper_arm)

        # Upper arm to forearm (elbow)
        upper_arm_to_forearm = TransformStamped()
        upper_arm_to_forearm.header.stamp = current_time.to_msg()
        upper_arm_to_forearm.header.frame_id = 'left_upper_arm_link'
        upper_arm_to_forearm.child_frame_id = 'left_forearm_link'

        upper_arm_to_forearm.transform.translation.x = 0.0
        upper_arm_to_forearm.transform.translation.y = 0.0
        upper_arm_to_forearm.transform.translation.z = 0.15  # 15cm forearm

        # Elbow movement
        elbow_angle = math.sin(self.time_counter * 0.9) * 0.8
        cy = math.cos(elbow_angle / 2.0)
        sx = math.sin(elbow_angle / 2.0)
        upper_arm_to_forearm.transform.rotation.w = cy
        upper_arm_to_forearm.transform.rotation.x = sx
        upper_arm_to_forearm.transform.rotation.y = 0.0
        upper_arm_to_forearm.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(upper_arm_to_forearm)

        # Forearm to hand (wrist)
        forearm_to_hand = TransformStamped()
        forearm_to_hand.header.stamp = current_time.to_msg()
        forearm_to_hand.header.frame_id = 'left_forearm_link'
        forearm_to_hand.child_frame_id = 'left_hand_link'

        forearm_to_hand.transform.translation.x = 0.0
        forearm_to_hand.transform.translation.y = 0.0
        forearm_to_hand.transform.translation.z = 0.1  # 10cm to hand

        # Wrist movement
        wrist_roll = math.sin(self.time_counter * 1.2) * 0.5
        cy = math.cos(wrist_roll / 2.0)
        sz = math.sin(wrist_roll / 2.0)
        forearm_to_hand.transform.rotation.w = cy
        forearm_to_hand.transform.rotation.x = 0.0
        forearm_to_hand.transform.rotation.y = 0.0
        forearm_to_hand.transform.rotation.z = sz

        self.tf_broadcaster.sendTransform(forearm_to_hand)

        # Right arm (similar to left but with different phase)
        r_shoulder_to_upper_arm = TransformStamped()
        r_shoulder_to_upper_arm.header.stamp = current_time.to_msg()
        r_shoulder_to_upper_arm.header.frame_id = 'right_shoulder_link'
        r_shoulder_to_upper_arm.child_frame_id = 'right_upper_arm_link'

        r_shoulder_to_upper_arm.transform.translation.x = 0.0
        r_shoulder_to_upper_arm.transform.translation.y = 0.0
        r_shoulder_to_upper_arm.transform.translation.z = 0.15

        # Right shoulder with opposite phase to create coordinated movement
        r_shoulder_pitch = math.sin(self.time_counter * 0.7 + math.pi) * 0.5
        cy = math.cos(r_shoulder_pitch / 2.0)
        sx = math.sin(r_shoulder_pitch / 2.0)
        r_shoulder_to_upper_arm.transform.rotation.w = cy
        r_shoulder_to_upper_arm.transform.rotation.x = sx
        r_shoulder_to_upper_arm.transform.rotation.y = 0.0
        r_shoulder_to_upper_arm.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(r_shoulder_to_upper_arm)

        # Additional right arm segments would follow similar pattern...

        # Log periodically
        if int(self.time_counter * 10) % 10 == 0:  # Every second
            self.get_logger().info(
                f'Published dynamic transforms - Time: {self.time_counter:.1f}s, '
                f'Robot at ({odom_to_base.transform.translation.x:.2f}, {odom_to_base.transform.translation.y:.2f})'
            )


def main(args=None):
    rclpy.init(args=args)

    combined_broadcaster = CombinedBroadcaster()

    try:
        rclpy.spin(combined_broadcaster)
    except KeyboardInterrupt:
        combined_broadcaster.get_logger().info('Combined broadcaster stopped')
    finally:
        combined_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 6: Creating a TF2 Listener Node

Now let's create a listener node that can query transforms:

```bash
touch ~/ros2_ws/src/tf2_examples/tf2_examples/transform_listener.py
```

Add the following content to `~/ros2_ws/src/tf2_examples/tf2_examples/transform_listener.py`:

```python
#!/usr/bin/env python3
"""
Transform listener - subscribes to TF2 transforms and queries relationships between frames
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
import math


class TransformListenerNode(Node):

    def __init__(self):
        super().__init__('transform_listener')

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to query transforms at 1 Hz
        self.timer = self.create_timer(1.0, self.lookup_transform)
        self.query_count = 0

        self.get_logger().info('Transform listener started')

    def lookup_transform(self):
        """
        Query transform between two frames
        """
        try:
            # Look up transform from base_link to camera_frame
            transform = self.tf_buffer.lookup_transform(
                'base_link',  # Target frame
                'camera_frame',  # Source frame
                rclpy.time.Time()  # Latest available time
            )

            # Log the transform
            self.get_logger().info(
                f'Transform base_link <- camera_frame: '
                f'position({transform.transform.translation.x:.3f}, '
                f'{transform.transform.translation.y:.3f}, '
                f'{transform.transform.translation.z:.3f}), '
                f'rotation({transform.transform.rotation.x:.3f}, '
                f'{transform.transform.rotation.y:.3f}, '
                f'{transform.transform.rotation.z:.3f}, '
                f'{transform.transform.rotation.w:.3f})'
            )

            # Also query odom to base_link transform
            try:
                odom_to_base = self.tf_buffer.lookup_transform(
                    'odom',
                    'base_link',
                    rclpy.time.Time()
                )

                self.get_logger().info(
                    f'Robot pose in odom: '
                    f'({odom_to_base.transform.translation.x:.2f}, '
                    f'{odom_to_base.transform.translation.y:.2f}), '
                    f'Yaw: {math.atan2(2*(odom_to_base.transform.rotation.w*odom_to_base.transform.rotation.z + odom_to_base.transform.rotation.x*odom_to_base.transform.rotation.y), 1 - 2*(odom_to_base.transform.rotation.y**2 + odom_to_base.transform.rotation.z**2)):.2f} rad'
                )
            except TransformException as e:
                self.get_logger().warn(f'Could not lookup odom->base_link: {e}')

        except TransformException as e:
            self.get_logger().warn(f'Could not transform base_link to camera_frame: {e}')

        # Demonstrate transforming a point
        self.transform_point_example()

        self.query_count += 1

    def transform_point_example(self):
        """
        Example of transforming a point from one frame to another
        """
        # Create a point in camera_frame (e.g., detected object 1m in front of camera)
        point_camera_frame = PointStamped()
        point_camera_frame.header.frame_id = 'camera_frame'
        point_camera_frame.header.stamp = self.get_clock().now().to_msg()
        point_camera_frame.point.x = 1.0  # 1m in front of camera
        point_camera_frame.point.y = 0.0  # centered left-right
        point_camera_frame.point.z = 0.0  # same height

        try:
            # Transform the point to base_link frame
            point_base_frame = self.tf_buffer.transform(
                point_camera_frame,
                'base_link',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.get_logger().info(
                f'Point transformed from camera_frame to base_link: '
                f'({point_base_frame.point.x:.2f}, {point_base_frame.point.y:.2f}, {point_base_frame.point.z:.2f})'
            )

        except TransformException as e:
            self.get_logger().warn(f'Could not transform point: {e}')


def main(args=None):
    rclpy.init(args=args)

    listener = TransformListenerNode()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        listener.get_logger().info('Transform listener stopped')
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 7: Package Configuration and Setup

Update the package.xml for the TF2 examples:

Edit `~/ros2_ws/src/tf2_examples/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tf2_examples</name>
  <version>0.0.0</version>
  <description>TF2 examples for coordinate frame transformations</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>tf2_ros</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create the setup.py file:

Edit `~/ros2_ws/src/tf2_examples/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'tf2_examples'

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
    description='TF2 examples for coordinate frame transformations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_broadcaster = tf2_examples.static_transform_broadcaster:main',
            'dynamic_broadcaster = tf2_examples.dynamic_transform_broadcaster:main',
            'combined_broadcaster = tf2_examples.combined_broadcaster:main',
            'transform_listener = tf2_examples.transform_listener:main',
        ],
    },
)
```

---

## Step 8: Building and Testing the TF2 Examples

```bash
cd ~/ros2_ws
colcon build --packages-select tf2_examples
source ~/ros2_ws/install/setup.bash
```

### Running the TF2 Examples

**Terminal 1 - Combined Broadcaster**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_examples combined_broadcaster
```

**Terminal 2 - Transform Listener** (open a new terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_examples transform_listener
```

**Terminal 3 - View TF Tree** (open a third terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_tools view_frames
```

This will create a `frames.pdf` file showing your TF tree structure.

---

## Step 9: TF2 Command Line Tools

ROS 2 provides several useful command line tools for working with TF2:

```bash
# View the current TF tree
ros2 run tf2_tools view_frames

# Echo transforms between two frames
ros2 run tf2_ros tf2_echo base_link camera_frame

# Get detailed information about a specific transform
ros2 run tf2_ros tf2_monitor

# Check for transform issues
ros2 run tf2_ros tf2_monitor --verbose
```

---

## Step 10: Advanced TF2 Concepts

### Time-Consistent Transformations

TF2 excels at time-consistent transformations:

```python
def get_transform_at_specific_time(self, target_frame, source_frame, timestamp):
    """
    Get transform at a specific time for time-consistent processing
    """
    try:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            timestamp,  # Specific time in the past
            timeout=rclpy.duration.Duration(seconds=0.1)
        )
        return transform
    except TransformException as e:
        self.get_logger().error(f'Transform lookup failed: {e}')
        return None
```

### Transform Interpolation

TF2 automatically interpolates between stored transforms for smooth motion:

```python
def lookup_smooth_transform(self, target_frame, source_frame, timestamp):
    """
    Lookup transform with automatic interpolation between stored values
    """
    try:
        # TF2 automatically interpolates between nearby transforms
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            timestamp
        )
        return transform
    except TransformException as e:
        self.get_logger().warn(f'No interpolated transform available: {e}')
        return None
```

### Buffer Management

Manage the TF2 buffer to control memory usage:

```python
def setup_tf_buffer_with_cache_time(self):
    """
    Create TF buffer with specific cache duration
    """
    # Create buffer with 30-second cache window
    self.tf_buffer = Buffer(
        cache_time=rclpy.duration.Duration(seconds=30),
        node=self
    )
```

---

## Step 11: TF2 Best Practices

### Frame Naming Conventions

- **Use descriptive names**: `base_link`, `camera_frame`, `laser_frame`
- **Follow ROS conventions**: Use standard frame names when possible
- **Be consistent**: Use the same naming scheme throughout your robot

### Transform Broadcasting

- **Static vs Dynamic**: Use StaticTransformBroadcaster for fixed relationships
- **Frequency**: Broadcast dynamic transforms at appropriate rates (10-100 Hz)
- **Accuracy**: Use precise timestamps for time-consistent transformations
- **Coordination**: Ensure all related transforms are published together

### Transform Listening

- **Error Handling**: Always handle transform lookup failures gracefully
- **Timeouts**: Use appropriate timeouts to avoid blocking indefinitely
- **Caching**: Leverage TF2's caching for repeated queries
- **Time Consistency**: Use the same time reference for related transformations

### Common Frame Conventions

| Frame | Purpose | Convention |
|-------|---------|------------|
| `map` | World-fixed frame | Usually georeferenced |
| `odom` | Odometry frame | Drifts over time |
| `base_link` | Robot body frame | Center of robot body |
| `base_footprint` | Robot projection | On ground plane |
| `camera_frame` | Camera optical frame | Z-forward, X-right, Y-down |

---

## Step 12: Real-World TF2 Applications

### Sensor Fusion

TF2 enables fusion of data from multiple sensors:

```python
def fuse_sensor_data(self):
    """
    Example of using TF2 for sensor fusion
    """
    # Get LiDAR point in LiDAR frame
    lidar_point = self.get_lidar_data()  # PointStamped in laser_frame

    # Get camera point in camera frame
    camera_point = self.get_camera_data()  # PointStamped in camera_frame

    try:
        # Transform LiDAR point to base_link
        lidar_base = self.tf_buffer.transform(lidar_point, 'base_link')

        # Transform camera point to base_link
        camera_base = self.tf_buffer.transform(camera_point, 'base_link')

        # Now both points are in the same coordinate system for fusion
        fused_data = self.combine_points(lidar_base.point, camera_base.point)

        return fused_data
    except TransformException as e:
        self.get_logger().error(f'Sensor fusion failed: {e}')
        return None
```

### Robot Navigation

TF2 is essential for navigation systems:

```python
def get_robot_pose_in_map(self):
    """
    Get robot pose in map frame for navigation
    """
    try:
        # Transform robot position from odom to map
        transform = self.tf_buffer.lookup_transform(
            'map',
            'base_link',
            rclpy.time.Time()
        )

        # Extract position and orientation
        position = transform.transform.translation
        orientation = transform.transform.rotation

        return (position, orientation)
    except TransformException as e:
        self.get_logger().error(f'Cannot get robot pose: {e}')
        return None
```

### Multi-Robot Systems

TF2 can handle transforms between multiple robots:

```python
def get_other_robot_position(self, robot_id):
    """
    Get position of another robot in our coordinate system
    """
    try:
        # Transform from other robot's base_link to our base_link
        transform = self.tf_buffer.lookup_transform(
            'base_link',  # Our frame
            f'robot_{robot_id}/base_link',  # Other robot's frame
            rclpy.time.Time()
        )
        return transform
    except TransformException as e:
        self.get_logger().warn(f'Cannot locate robot {robot_id}: {e}')
        return None
```

---

## Step 13: TF2 Troubleshooting

### Common Issues and Solutions

#### Issue 1: "Could not find a connection between frames"

**Symptoms**: Transform lookup fails with connection error

**Solutions**:
```bash
# Check current TF tree
ros2 run tf2_tools view_frames

# Check if specific transforms exist
ros2 run tf2_ros tf2_echo base_link camera_frame

# Verify all required broadcasters are running
ros2 node list | grep -i tf
```

#### Issue 2: "Transform is too old" or "Transform is too new"

**Symptoms**: Transform lookup fails due to timing issues

**Solutions**:
```python
# Use latest available transform
try:
    transform = tf_buffer.lookup_transform(
        target_frame,
        source_frame,
        rclpy.time.Time()  # Use latest available
    )
except TransformException as e:
    # Handle gracefully
    pass

# Or wait for transform to become available
try:
    transform = tf_buffer.lookup_transform(
        target_frame,
        source_frame,
        time_point,
        timeout=rclpy.duration.Duration(seconds=1.0)
    )
except TransformException as e:
    # Handle timeout
    pass
```

#### Issue 3: High Memory Usage

**Symptoms**: TF2 buffer consuming excessive memory

**Solutions**:
```python
# Limit cache time
tf_buffer = Buffer(
    cache_time=rclpy.duration.Duration(seconds=10),
    node=self
)

# Reduce transform publishing frequency
self.timer = self.create_timer(0.1, self.broadcast_transform)  # 10 Hz instead of 100 Hz
```

#### Issue 4: Timing Inconsistencies

**Symptoms**: Transforms appear inconsistent or jerky

**Solutions**:
- Ensure accurate timestamps on all transforms
- Use appropriate transform publishing frequency
- Check system clock synchronization in multi-robot setups

---

## Step 14: Performance Optimization

### Efficient Broadcasting

```python
class OptimizedTFBroadcaster(Node):
    def __init__(self):
        super().__init__('optimized_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Use appropriate timer frequency
        self.timer = self.create_timer(0.05, self.broadcast_transforms)  # 20 Hz

    def broadcast_multiple_transforms(self):
        """
        Broadcast multiple transforms in a single call for efficiency
        """
        transforms = []

        # Create all transforms needed
        # ... populate transforms list ...

        # Send all at once
        self.tf_broadcaster.sendTransform(transforms)
```

### Efficient Listening

```python
class OptimizedTFListener(Node):
    def __init__(self):
        super().__init__('optimized_tf_listener')
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cache frequently used transforms to avoid repeated lookups
        self.cached_transforms = {}
        self.cache_timeout = 0.1  # 100ms cache

    def get_cached_transform(self, target_frame, source_frame):
        """
        Get transform with caching to avoid repeated lookups
        """
        cache_key = f"{target_frame}_{source_frame}"

        # Check if we have a recent cached transform
        if cache_key in self.cached_transforms:
            cached_time, transform = self.cached_transforms[cache_key]
            current_time = self.get_clock().now()

            if (current_time.nanoseconds - cached_time.nanoseconds) < self.cache_timeout * 1e9:
                return transform

        # Get fresh transform
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )

            # Cache it
            self.cached_transforms[cache_key] = (self.get_clock().now(), transform)
            return transform
        except TransformException as e:
            self.get_logger().warn(f'Could not get transform {target_frame} -> {source_frame}: {e}')
            return None
```

---

## Lab Summary

In this lab, you've successfully:

✅ **Defined custom coordinate frames** and understood their hierarchical relationships
✅ **Implemented static transform broadcasters** for fixed frame relationships
✅ **Created dynamic transform broadcasters** for changing frame relationships
✅ **Used TransformBroadcaster and StaticTransformBroadcaster APIs** correctly
✅ **Validated TF2 tree structure** with `view_frames` command and transform querying

### Key Takeaways

- **TF2 manages** coordinate frame relationships over time in a hierarchical tree
- **Static transforms** define fixed relationships (sensor mounts, robot geometry)
- **Dynamic transforms** change over time (robot movement, joint positions)
- **Transform listeners** query relationships between any two connected frames
- **Time consistency** is maintained through accurate timestamps and interpolation

---

## Next Steps

Now that you understand TF2 broadcasting, you're ready to explore:

- **Lab 7**: Launch files for system startup and management
- **Lab 8**: Robot state publishing and joint state management
- **Lab 9**: Advanced navigation and path planning

**Continue to [Lab 7: Launch Files](./lab07-launch-files.md)**

---
**Previous**: [Lab 5: Humanoid URDF](./lab05-humanoid-urdf.md) | **Next**: [Lab 7: Launch Files](./lab07-launch-files.md)