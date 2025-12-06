---
title: TF2 Coordinate Frames
sidebar_label: TF2 Coordinate Frames
sidebar_position: 10
description: Learn TF2 coordinate frames, transforms, and hierarchical relationships for robot localization, mapping, and multi-sensor fusion in ROS 2
---

# TF2 Coordinate Frames

## Overview

In this section, you'll learn about TF2 (Transform Library 2) - ROS 2's system for tracking coordinate frame relationships in time. You'll understand coordinate frames, transforms, and hierarchical relationships essential for robot localization, mapping, and multi-sensor fusion. TF2 is crucial for relating data from different sensors and coordinating robot motion in 3D space.

**Duration**: 1.5 hours

**Learning Objectives**:
- ✅ Define coordinate frames and understand their relationships in TF2 tree structure
- ✅ Implement transform broadcasters for static and dynamic frame relationships
- ✅ Use TF2 listeners to query transforms between arbitrary frames
- ✅ Validate TF2 tree with `ros2 run tf2_tools view_frames` command
- ✅ Implement coordinate frame transformation for sensor fusion applications

---

## Prerequisites

Before starting this section, ensure you have:

✅ **Completed Lab 1** - Talker/Listener basics
✅ **Completed Lab 2** - Custom messages
✅ **Completed Lab 3** - Services
✅ **Completed Lab 4** - Actions
✅ **Completed Lab 5** - Humanoid URDF
✅ **ROS 2 Humble installed** with all standard packages
✅ **Basic understanding** of ROS 2 nodes, topics, and robot models
✅ **Linear algebra knowledge** for understanding coordinate transformations

---

## Step 1: Understanding TF2 Concepts

### What is TF2?

**TF2 (Transform Library 2)** is ROS 2's system for managing coordinate frame relationships over time. It allows you to:
- **Track positions** of robot parts relative to each other
- **Transform data** between different coordinate frames
- **Maintain temporal relationships** between frames
- **Fuse sensor data** from multiple coordinate systems

### Key TF2 Concepts

| Concept | Definition | Example |
|---------|------------|---------|
| **Frame** | Coordinate system (position + orientation) | `base_link`, `camera_frame`, `map` |
| **Transform** | Relationship between two frames | Translation + rotation from `base_link` to `camera_frame` |
| **Tree** | Hierarchical structure of frames | `map` → `odom` → `base_link` → `camera_frame` |
| **Timestamp** | When transform was valid | Critical for time-consistent transformations |

### TF2 vs TF1

TF2 improves over the original TF with:
- **Better performance** - More efficient data structures
- **Advanced interpolation** - Smooth transform estimation
- **Cleaner API** - Simplified usage patterns
- **Memory management** - Better resource handling

---

## Step 2: TF2 Coordinate Frame Hierarchy

### Robot Frame Convention

ROS uses the **Right Hand Rule** coordinate system:
- **X**: Forward
- **Y**: Left
- **Z**: Up

### Common Robot Frames

```
map (world-fixed)
└── odom (odometry reference)
    └── base_link (robot body)
        ├── camera_frame (sensor mount)
        ├── laser_frame (LiDAR mount)
        ├── left_wheel
        ├── right_wheel
        └── imu_frame (inertial measurement)
```

### TF2 Tree Properties

1. **Connected**: Every frame can be transformed to any other frame
2. **Acyclic**: No loops in the transform tree
3. **Directed**: Transforms have parent-child relationships
4. **Temporal**: Transforms are valid at specific times

---

## Step 3: Creating a TF2 Package

First, let's create a package for our TF2 examples:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python tf2_examples --dependencies rclpy tf2_ros geometry_msgs
```

Create the necessary directory:
```bash
mkdir -p ~/ros2_ws/src/tf2_examples/tf2_examples
```

---

## Step 4: Static Transform Broadcasting

Static transforms are constant relationships between frames (e.g., sensor mounts). Let's create a static transform broadcaster:

```bash
touch ~/ros2_ws/src/tf2_examples/tf2_examples/static_transform_broadcaster.py
```

Add the following content to `~/ros2_ws/src/tf2_examples/tf2_examples/static_transform_broadcaster.py`:

```python
#!/usr/bin/env python3
"""
Static transform broadcaster - publishes fixed relationships between coordinate frames
"""

import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class StaticTransformBroadcaster(Node):

    def __init__(self):
        super().__init__('static_transform_broadcaster')

        # Create the static transform broadcaster
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms for a robot with camera and IMU
        self.publish_static_transforms()

    def publish_static_transforms(self):
        """
        Publish static transforms between robot frames
        """
        transforms = []

        # Camera mounted on front of robot
        camera_transform = TransformStamped()
        camera_transform.header.stamp = self.get_clock().now().to_msg()
        camera_transform.header.frame_id = 'base_link'
        camera_transform.child_frame_id = 'camera_frame'
        camera_transform.transform.translation.x = 0.2  # 20 cm in front
        camera_transform.transform.translation.y = 0.0  # centered left-right
        camera_transform.transform.translation.z = 0.5  # 50 cm high
        # Rotate camera to look forward (identity rotation)
        camera_transform.transform.rotation.w = 1.0
        camera_transform.transform.rotation.x = 0.0
        camera_transform.transform.rotation.y = 0.0
        camera_transform.transform.rotation.z = 0.0

        transforms.append(camera_transform)

        # IMU mounted on robot body
        imu_transform = TransformStamped()
        imu_transform.header.stamp = self.get_clock().now().to_msg()
        imu_transform.header.frame_id = 'base_link'
        imu_transform.child_frame_id = 'imu_frame'
        imu_transform.transform.translation.x = 0.0  # centered
        imu_transform.transform.translation.y = 0.0  # centered
        imu_transform.transform.translation.z = 0.3  # 30 cm high
        # Identity rotation - IMU aligned with base
        imu_transform.transform.rotation.w = 1.0
        imu_transform.transform.rotation.x = 0.0
        imu_transform.transform.rotation.y = 0.0
        imu_transform.transform.rotation.z = 0.0

        transforms.append(imu_transform)

        # Left wheel offset
        left_wheel_transform = TransformStamped()
        left_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        left_wheel_transform.header.frame_id = 'base_link'
        left_wheel_transform.child_frame_id = 'left_wheel'
        left_wheel_transform.transform.translation.x = 0.0   # centered front-back
        left_wheel_transform.transform.translation.y = 0.3   # 30 cm to the left
        left_wheel_transform.transform.translation.z = -0.1  # 10 cm below base
        # Identity rotation
        left_wheel_transform.transform.rotation.w = 1.0
        left_wheel_transform.transform.rotation.x = 0.0
        left_wheel_transform.transform.rotation.y = 0.0
        left_wheel_transform.transform.rotation.z = 0.0

        transforms.append(left_wheel_transform)

        # Right wheel offset
        right_wheel_transform = TransformStamped()
        right_wheel_transform.header.stamp = self.get_clock().now().to_msg()
        right_wheel_transform.header.frame_id = 'base_link'
        right_wheel_transform.child_frame_id = 'right_wheel'
        right_wheel_transform.transform.translation.x = 0.0   # centered front-back
        right_wheel_transform.transform.translation.y = -0.3  # 30 cm to the right
        right_wheel_transform.transform.translation.z = -0.1  # 10 cm below base
        # Identity rotation
        right_wheel_transform.transform.rotation.w = 1.0
        right_wheel_transform.transform.rotation.x = 0.0
        right_wheel_transform.transform.rotation.y = 0.0
        right_wheel_transform.transform.rotation.z = 0.0

        transforms.append(right_wheel_transform)

        # Publish all transforms
        for transform in transforms:
            self.get_logger().info(
                f'Publishing static transform: {transform.header.frame_id} -> {transform.child_frame_id}'
            )

        self.tf_static_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)

    static_broadcaster = StaticTransformBroadcaster()

    try:
        # Keep the node alive to maintain static transforms
        rclpy.spin(static_broadcaster)
    except KeyboardInterrupt:
        static_broadcaster.get_logger().info('Static transform broadcaster stopped')
    finally:
        static_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 5: Dynamic Transform Broadcasting

Dynamic transforms change over time (e.g., robot movement). Let's create a dynamic transform broadcaster:

```bash
touch ~/ros2_ws/src/tf2_examples/tf2_examples/dynamic_transform_broadcaster.py
```

Add the following content to `~/ros2_ws/src/tf2_examples/tf2_examples/dynamic_transform_broadcaster.py`:

```python
#!/usr/bin/env python3
"""
Dynamic transform broadcaster - publishes changing relationships between coordinate frames
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class DynamicTransformBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_transform_broadcaster')

        # Create the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer to broadcast transforms at 10 Hz
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        self.time_offset = 0.0

        self.get_logger().info('Dynamic transform broadcaster started')

    def broadcast_transform(self):
        """
        Broadcast dynamic transforms between frames
        """
        # Create a transform from odom to base_link (robot moving in a circle)
        t = TransformStamped()

        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Move in a circular path
        self.time_offset += 0.1  # Increment time for animation

        # Circular motion: x = cos(t), y = sin(t)
        t.transform.translation.x = math.cos(self.time_offset * 0.5) * 2.0  # Radius 2m
        t.transform.translation.y = math.sin(self.time_offset * 0.5) * 2.0  # Radius 2m
        t.transform.translation.z = 0.0  # Ground level

        # Calculate robot orientation (tangent to circular path)
        # For circular motion, orientation is perpendicular to radius vector
        angle = self.time_offset * 0.5 + math.pi/2  # +pi/2 for tangent direction
        t.transform.rotation.w = math.cos(angle / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(angle / 2.0)

        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

        # Log the current position
        self.get_logger().info(
            f'Published transform: odom->base_link at '
            f'({t.transform.translation.x:.2f}, {t.transform.translation.y:.2f}) '
            f'orientation: {angle:.2f} rad'
        )


def main(args=None):
    rclpy.init(args=args)

    dynamic_broadcaster = DynamicTransformBroadcaster()

    try:
        rclpy.spin(dynamic_broadcaster)
    except KeyboardInterrupt:
        dynamic_broadcaster.get_logger().info('Dynamic transform broadcaster stopped')
    finally:
        dynamic_broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 6: TF2 Listening and Transform Queries

Now let's create a TF2 listener to query transforms:

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
from tf2_ros import TransformListener, Buffer
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

        # Counter for logging
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
                f'Transform base_link -> camera_frame: '
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
                    f'Robot position in odom frame: '
                    f'({odom_to_base.transform.translation.x:.2f}, '
                    f'{odom_to_base.transform.translation.y:.2f})'
                )
            except Exception as e:
                self.get_logger().warn(f'Could not lookup odom->base_link: {e}')

        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

        # Demonstrate transforming a point
        self.transform_point_example()

    def transform_point_example(self):
        """
        Example of transforming a point from one frame to another
        """
        # Create a point in camera_frame (e.g., detected object)
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

        except Exception as e:
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

## Step 7: Update Package Configuration

Update the setup.py file:

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
            'static_transform_broadcaster = tf2_examples.static_transform_broadcaster:main',
            'dynamic_transform_broadcaster = tf2_examples.dynamic_transform_broadcaster:main',
            'transform_listener = tf2_examples.transform_listener:main',
        ],
    },
)
```

Update the package.xml:

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
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## Step 8: Building the TF2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select tf2_examples
source ~/ros2_ws/install/setup.bash
```

---

## Step 9: Testing TF2 Functionality

### Running the TF2 Example Nodes

Let's run the TF2 nodes to see them in action:

**Terminal 1 - Static Transform Broadcaster**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_examples static_transform_broadcaster
```

**Terminal 2 - Dynamic Transform Broadcaster** (open a new terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_examples dynamic_transform_broadcaster
```

**Terminal 3 - Transform Listener** (open a third terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run tf2_examples transform_listener
```

### Viewing the TF2 Tree

You can visualize the TF2 tree using ROS 2 tools:

```bash
# Generate a PDF of the current TF tree
ros2 run tf2_tools view_frames
```

This command creates a `frames.pdf` file showing all the connected frames and their relationships.

### Checking TF2 Status

```bash
# Get detailed information about TF2
ros2 run tf2_ros tf2_monitor
```

```bash
# Echo specific transforms
ros2 run tf2_ros tf2_echo odom base_link
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
        # Query transform at specific time in the past
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            timestamp,  # Specific time
            timeout=rclpy.duration.Duration(seconds=0.1)
        )
        return transform
    except Exception as e:
        self.get_logger().error(f'Transform lookup failed: {e}')
        return None
```

### Transform Interpolation

TF2 interpolates between stored transforms for smooth motion:

```python
def lookup_closest_transform(self, target_frame, source_frame, timestamp):
    """
    Lookup transform closest to requested time with interpolation
    """
    try:
        # TF2 automatically interpolates between nearby transforms
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            timestamp
        )
        return transform
    except Exception as e:
        # Handle case where no transform is available
        self.get_logger().warn(f'No transform available: {e}')
        return None
```

### Buffer Management

Manage the TF2 buffer to control memory usage:

```python
def setup_tf_buffer_with_timeout(self):
    """
    Create TF buffer with specific timeout and cache size
    """
    # Create buffer with 30-second cache
    self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30))
    self.tf_listener = TransformListener(self.tf_buffer, self)
```

---

## Step 11: TF2 Best Practices

### Frame Naming Conventions

- **Use descriptive names**: `base_link`, `camera_frame`, `laser_frame`
- **Follow REP-105**: Standard frame names for interoperability
- **Be consistent**: Use the same naming scheme throughout your robot

### Transform Broadcasting

- **Static vs Dynamic**: Use StaticTransformBroadcaster for fixed relationships
- **Frequency**: Broadcast dynamic transforms at appropriate rates (10-100 Hz)
- **Timing**: Use accurate timestamps for time-consistent transformations

### Transform Listening

- **Error Handling**: Always handle transform lookup failures gracefully
- **Timeouts**: Use appropriate timeouts to avoid blocking
- **Caching**: Leverage TF2's caching for repeated queries

### Common Frame Conventions

| Frame | Purpose | Convention |
|-------|---------|------------|
| `map` | World-fixed frame | Usually georeferenced |
| `odom` | Odometry frame | Drifts over time |
| `base_link` | Robot body frame | RHR, X-forward, Z-up |
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
    except Exception as e:
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
        transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

        # Extract position and orientation
        position = transform.transform.translation
        orientation = transform.transform.rotation

        return (position, orientation)
    except Exception as e:
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
    except Exception as e:
        self.get_logger().warn(f'Cannot locate robot {robot_id}: {e}')
        return None
```

---

## Step 13: TF2 Troubleshooting

### Common Issues and Solutions

### Issue 1: "Could not find a connection between X and Y"

**Symptoms**: Transform lookup fails with connection error

**Solutions**:
```bash
# Check current TF tree
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_ros tf2_echo map base_link

# Verify all required broadcasters are running
ros2 node list | grep tf
```

### Issue 2: "Transform is too old" or "Transform is too new"

**Symptoms**: Transform lookup fails due to timing issues

**Solutions**:
```python
# Use latest available transform
transform = tf_buffer.lookup_transform(target, source, rclpy.time.Time())

# Or wait for transform to become available
try:
    transform = tf_buffer.lookup_transform(
        target,
        source,
        rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=1.0)
    )
except Exception as e:
    # Handle timeout gracefully
    pass
```

### Issue 3: High Memory Usage

**Symptoms**: TF2 buffer consuming excessive memory

**Solutions**:
```python
# Limit cache time
tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))

# Reduce transform publishing frequency
self.timer = self.create_timer(0.1, self.broadcast_transform)  # 10 Hz instead of 100 Hz
```

### Issue 4: Timing Issues

**Symptoms**: Transforms appear inconsistent or jerky

**Solutions**:
- Ensure accurate timestamps on all transforms
- Use appropriate transform publishing frequency
- Check system clock synchronization in multi-robot setups

---

## Step 14: TF2 Performance Optimization

### Efficient Broadcasting

```python
class OptimizedBroadcaster(Node):
    def __init__(self):
        super().__init__('optimized_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Batch multiple transforms when possible
        self.transforms = []

        # Use appropriate timer frequency
        self.timer = self.create_timer(0.05, self.broadcast_batch)  # 20 Hz

    def broadcast_batch(self):
        """
        Broadcast multiple transforms in a single call
        """
        # Prepare all transforms
        self.transforms.clear()

        # Add all necessary transforms
        # ... populate transforms list ...

        # Broadcast all at once
        self.tf_broadcaster.sendTransform(self.transforms)
```

### Efficient Listening

```python
class OptimizedListener(Node):
    def __init__(self):
        super().__init__('optimized_tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cache frequently used transforms
        self.cached_transforms = {}

    def get_cached_transform(self, target_frame, source_frame):
        """
        Cache frequently accessed transforms
        """
        cache_key = f"{target_frame}_{source_frame}"

        if cache_key in self.cached_transforms:
            # Check if cached transform is still valid
            cached_time = self.cached_transforms[cache_key]['timestamp']
            current_time = self.get_clock().now()

            if (current_time.nanoseconds - cached_time.nanoseconds) < 1e8:  # 100ms
                return self.cached_transforms[cache_key]['transform']

        # Get fresh transform
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )

            # Cache it
            self.cached_transforms[cache_key] = {
                'transform': transform,
                'timestamp': self.get_clock().now()
            }

            return transform
        except Exception as e:
            return None
```

---

## Section Summary

In this section, you've successfully:

✅ **Understood TF2 concepts** including frames, transforms, and hierarchical relationships
✅ **Implemented static transform broadcasters** for fixed frame relationships
✅ **Created dynamic transform broadcasters** for changing relationships
✅ **Used TF2 listeners** to query transforms between arbitrary frames
✅ **Validated TF2 tree** with `view_frames` command and applied to sensor fusion

### Key Takeaways

- **TF2 manages** coordinate frame relationships over time in a hierarchical tree
- **Static transforms** define fixed relationships (sensor mounts, robot geometry)
- **Dynamic transforms** change over time (robot movement, joint positions)
- **Transform listeners** query relationships between any two connected frames
- **Time consistency** is maintained through accurate timestamps and interpolation

---

## Next Steps

Now that you understand TF2 coordinate frames, you're ready to explore:

- **Lab 6**: TF2 broadcasting and listening in detail
- **Lab 7**: Robot state publishing and joint state management
- **Lab 8**: Launch files for system startup

**Continue to [Lab 6: TF2 Broadcasting](./lab06-tf2-broadcasting.md)**

---
**Previous**: [Lab 5: Humanoid URDF](./lab05-humanoid-urdf.md) | **Next**: [Lab 6: TF2 Broadcasting](./lab06-tf2-broadcasting.md)