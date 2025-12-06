---
title: Lab 8 - RViz Integration
sidebar_label: Lab 8 - RViz Integration
sidebar_position: 13
description: Master RViz2 visualization with robot models, sensor data, TF frames, custom displays, and configuration management for effective robot monitoring and debugging
---

# Lab 8: RViz Integration

## Overview

In this lab, you'll master RViz2 visualization with robot models, sensor data, TF frames, custom displays, and configuration management for effective robot monitoring and debugging. You'll learn to create comprehensive RViz configurations that visualize robot state, sensor streams, and planning results for efficient robot operation and development.

**Duration**: 2 hours

**Learning Objectives**:
- ✅ Configure RViz2 displays for robot models, sensor data, and coordinate frames
- ✅ Create custom RViz display plugins and configuration files for specific use cases
- ✅ Integrate robot state visualization with joint states and TF transforms
- ✅ Implement sensor data visualization for cameras, LiDAR, IMU, and other sensors
- ✅ Validate RViz configurations with `rviz2` command and system monitoring

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **Completed Lab 1-7** - All previous labs covering ROS 2 fundamentals
✅ **ROS 2 Humble installed** with RViz2 and all visualization packages
✅ **Basic understanding** of URDF, TF2, and launch files
✅ **Experience with** sensor data topics and coordinate frames
✅ **Robot model available** (from Lab 5: Humanoid URDF)

---

## Step 1: Understanding RViz2 Concepts

### What is RViz2?

**RViz2 (Robot Visualizer 2)** is ROS 2's 3D visualization tool that allows you to visualize robot models, sensor data, and planning information in an intuitive 3D environment. RViz2 is essential for:

- **Robot state visualization**: Seeing robot model with current joint positions
- **Sensor data visualization**: Displaying camera images, LiDAR scans, IMU data
- **Coordinate frame inspection**: Visualizing TF trees and transforms
- **Planning visualization**: Showing paths, goals, and costmaps
- **Debugging**: Identifying issues with robot state and sensor data

### RViz2 Architecture

RViz2 consists of:

| Component | Purpose | Example |
|-----------|---------|---------|
| **Displays Panel** | Shows available visualization types | RobotModel, LaserScan, Image, TF |
| **Views Panel** | Controls 3D camera perspective | Orbit, TopDownOrtho, XYOrbit |
| **Tools Panel** | Interactive tools | 2D Pose Estimate, 2D Nav Goal, Publish Point |
| **Displays List** | Active visualization elements | RobotModel, LaserScan, TF tree |
| **Time Panel** | Simulation time control | Play, Pause, Step controls |

### RViz2 Display Types

RViz2 provides numerous display types for different visualization needs:

| Display Type | Purpose | Common Use |
|--------------|---------|------------|
| **RobotModel** | Visualizes robot URDF model | Robot state visualization |
| **TF** | Shows coordinate frame tree | Frame relationships |
| **LaserScan** | Visualizes LiDAR data | Obstacle detection |
| **Image** | Shows camera images | Computer vision |
| **PointCloud2** | Displays 3D point clouds | 3D mapping |
| **Marker** | Custom visualization markers | Waypoints, paths |
| **Path** | Shows planned paths | Navigation |
| **Odometry** | Robot pose and twist | Motion visualization |

---

## Step 2: Installing RViz2 and Visualization Packages

First, let's make sure we have all the necessary visualization packages:

```bash
sudo apt update
sudo apt install ros-humble-desktop  # Includes RViz2 and visualization tools

# Install additional visualization packages
sudo apt install \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins \
  ros-humble-rviz-visual-tools \
  ros-humble-interactive-markers \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui
```

---

## Step 3: Creating an RViz Package

Let's create a package specifically for our RViz examples:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python rviz_examples --dependencies rclpy std_msgs sensor_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs robot_state_publisher joint_state_publisher joint_state_publisher_gui
```

Create the necessary directories:
```bash
mkdir -p ~/ros2_ws/src/rviz_examples/rviz
mkdir -p ~/ros2_ws/src/rviz_examples/config
mkdir -p ~/ros2_ws/src/rviz_examples/launch
mkdir -p ~/ros2_ws/src/rviz_examples/rviz_examples
```

---

## Step 4: Creating Sample Robot Data Publishers

Let's create nodes that publish sample data for visualization. First, create a sample robot state publisher:

```bash
touch ~/ros2_ws/src/rviz_examples/rviz_examples/sample_robot_publisher.py
```

Add the following content to `~/ros2_ws/src/rviz_examples/rviz_examples/sample_robot_publisher.py`:

```python
#!/usr/bin/env python3
"""
Sample Robot Publisher - publishes robot state data for RViz visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import math
import random


class SampleRobotPublisher(Node):

    def __init__(self):
        super().__init__('sample_robot_publisher')

        # Publishers for different data types
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # Timer to publish data at 30 Hz
        self.timer = self.create_timer(0.033, self.publish_sample_data)  # ~30 Hz
        self.time_counter = 0.0

        # Initialize joint positions
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint7', 'joint8', 'joint9', 'joint10', 'joint11', 'joint12'
        ]
        self.joint_positions = [0.0] * 12
        self.joint_velocities = [0.0] * 12
        self.joint_efforts = [0.0] * 12

        # Robot position tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.get_logger().info('Sample robot publisher started')

    def publish_sample_data(self):
        """
        Publish sample robot data for visualization
        """
        # Update joint positions with some oscillating motion
        for i in range(len(self.joint_positions)):
            # Create different motion patterns for different joints
            freq = 0.5 + (i * 0.1)  # Different frequencies
            amplitude = 0.5 + (i * 0.05)  # Different amplitudes
            self.joint_positions[i] = math.sin(self.time_counter * freq) * amplitude

            # Calculate velocities (derivative of positions)
            self.joint_velocities[i] = math.cos(self.time_counter * freq) * freq * amplitude

        # Update robot position (moving in a circle)
        self.robot_x = math.cos(self.time_counter * 0.2) * 2.0  # Circle radius 2m
        self.robot_y = math.sin(self.time_counter * 0.2) * 2.0
        self.robot_theta = self.time_counter * 0.2 + math.pi/2  # Heading direction

        # Publish joint states
        self.publish_joint_states()

        # Publish odometry
        self.publish_odometry()

        # Publish visualization markers
        self.publish_markers()

        # Increment time counter
        self.time_counter += 0.033

    def publish_joint_states(self):
        """
        Publish joint state messages
        """
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        self.joint_state_publisher.publish(msg)

    def publish_odometry(self):
        """
        Publish odometry messages
        """
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'

        # Set position
        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        msg.pose.pose.position.z = 0.0

        # Convert orientation from Euler to Quaternion
        cy = math.cos(self.robot_theta * 0.5)
        sy = math.sin(self.robot_theta * 0.5)
        msg.pose.pose.orientation.w = cy
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0

        # Set velocities
        # Linear velocity: tangential to the circle
        linear_speed = 0.2 * 2.0  # omega * radius
        msg.twist.twist.linear.x = linear_speed * math.cos(self.robot_theta)
        msg.twist.twist.linear.y = linear_speed * math.sin(self.robot_theta)

        # Angular velocity: rotational speed
        msg.twist.twist.angular.z = 0.2

        self.odom_publisher.publish(msg)

    def publish_markers(self):
        """
        Publish visualization markers
        """
        marker_array = MarkerArray()

        # Marker 1: Robot path trail
        path_marker = Marker()
        path_marker.header = Header()
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.header.frame_id = 'odom'
        path_marker.ns = 'robot_path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        # Set scale (width of the line)
        path_marker.scale.x = 0.05  # Line width

        # Set color (red)
        path_marker.color.r = 1.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 0.8  # Alpha (transparency)

        # Add points to create a trail (last 50 positions)
        for i in range(50):
            t = self.time_counter - i * 0.1
            if t >= 0:
                x = math.cos(t * 0.2) * 2.0
                y = math.sin(t * 0.2) * 2.0
                z = 0.05  # Slightly above ground

                point = Point()
                point.x = x
                point.y = y
                point.z = z
                path_marker.points.append(point)

        marker_array.markers.append(path_marker)

        # Marker 2: Goal position (future position)
        goal_marker = Marker()
        goal_marker.header = Header()
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.header.frame_id = 'odom'
        goal_marker.ns = 'goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD

        # Set position (1 second into the future)
        future_t = self.time_counter + 1.0
        goal_marker.pose.position.x = math.cos(future_t * 0.2) * 2.0
        goal_marker.pose.position.y = math.sin(future_t * 0.2) * 2.0
        goal_marker.pose.position.z = 0.1
        goal_marker.pose.orientation.w = 1.0

        # Set scale (size of the sphere)
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.3

        # Set color (green)
        goal_marker.color.r = 0.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 0.8

        marker_array.markers.append(goal_marker)

        # Marker 3: Random obstacles
        for i in range(5):
            obstacle_marker = Marker()
            obstacle_marker.header = Header()
            obstacle_marker.header.stamp = self.get_clock().now().to_msg()
            obstacle_marker.header.frame_id = 'odom'
            obstacle_marker.ns = 'obstacles'
            obstacle_marker.id = i + 10  # Start from ID 10 to avoid conflicts
            obstacle_marker.type = Marker.CYLINDER
            obstacle_marker.action = Marker.ADD

            # Random position around the circle
            angle = random.uniform(0, 2 * math.pi)
            radius = random.uniform(2.5, 4.0)
            obstacle_marker.pose.position.x = math.cos(angle) * radius
            obstacle_marker.pose.position.y = math.sin(angle) * radius
            obstacle_marker.pose.position.z = 0.25  # Half the cylinder height
            obstacle_marker.pose.orientation.w = 1.0

            # Set scale (cylinder dimensions)
            obstacle_marker.scale.x = 0.4  # Diameter
            obstacle_marker.scale.y = 0.4  # Diameter
            obstacle_marker.scale.z = 0.5  # Height

            # Set color (orange)
            obstacle_marker.color.r = 1.0
            obstacle_marker.color.g = 0.6
            obstacle_marker.color.b = 0.0
            obstacle_marker.color.a = 0.9

            marker_array.markers.append(obstacle_marker)

        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    publisher = SampleRobotPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Sample robot publisher stopped')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 5: Creating RViz Configuration Files

Now let's create RViz configuration files that visualize our robot and sensor data:

```bash
touch ~/ros2_ws/src/rviz_examples/rviz/humanoid_robot_config.rviz
```

Add the following content to `~/ros2_ws/src/rviz_examples/rviz/humanoid_robot_config.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /TF1
        - /Odometry1
        - /LaserScan1
        - /PointCloud1
        - /Image1
        - /MarkerArray1
        - /RobotModel1/Status1
        - /TF1/Frames1
        - /TF1/Status1
        - /Odometry1/Shape1
        - /MarkerArray1/Status1
      Splitter Ratio: 0.5
    Tree Height: 863
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
        base_footprint:
          Value: true
        base_link:
          Value: true
        camera_frame:
          Value: true
        imu_frame:
          Value: true
        laser_frame:
          Value: true
        left_wheel:
          Value: true
        map:
          Value: true
        odom:
          Value: true
        right_wheel:
          Value: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree:
        map:
          odom:
            base_footprint:
              base_link:
                camera_frame:
                  {}
                imu_frame:
                  {}
                laser_frame:
                  {}
                left_wheel:
                  {}
                right_wheel:
                  {}
      Update Interval: 0
      Value: true
    - Angle Tolerance: 0.10000000149011612
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Keep: 100
      Name: Odometry
      Position Tolerance: 0.10000000149011612
      Shape:
        Alpha: 1
        Color: 255; 25; 0
        Frame: Local
        Offset: 0; 0; 0
        Scale: 1; 1; 1
        Value: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter Size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter Size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Buffer Length: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Head Diameter: 0.30000001192092896
      Head Length: 0.20000000298023224
      Length: 0.30000001192092896
      Line Style: Lines
      Line Width: 0.029999999329447746
      Name: Path
      Offset:
        X: 0
        Y: 0
        Z: 0
      Pose Color: 255; 85; 255
      Pose Style: None
      Radius: 0.029999999329447746
      Shaft Diameter: 0.10000000149011612
      Shaft Length: 0.10000000149011612
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter Size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /path_plan
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color Scheme: FlatColor
      Color: 255; 255; 255
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: PointCloud2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter Size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /point_cloud
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: false
      Max Value: 1
      Min Value: 0
      Name: Image
      Normalize Range: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /camera/image_raw
      Value: false
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: MarkerArray
      Namespaces:
        "": true
        goal: true
        obstacles: true
        robot_path: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /visualization_marker_array
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10.0
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.7853981852531433
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.7853981852531433
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 1025
  Hide Left Dock: false
  Hide Right Dock: false
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000003a0fc0200000009fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000003a0000000c900fffffffb0000002000730065006c0065006300740069006f006e0020006200750066006600650072005f0069006d0061006700650000000000000000000000000000000000fb0000000a0049006d0061006700650000000000000003a00000000000000000fb0000000a0049006d006100670065000000023a0000016d0000000000000000fb0000000a0049006d006100670065010000003d0000016d0000000000000000fb0000001e0050006f0073006900740069006f006e002000260020005600690065007700000000000000016d0000000000000000fb0000001c00430061006d00650072006100200054006f00700069006300000000000000016d0000000000000000000000010000010f000003a0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073000000003d000003a0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d0065010000000000000418000002f600fffffffb0000000800540069006d006501000000000000045000000000000000000000052d000003a000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Width: 1853
  X: 67
  Y: 27
```

---

## Step 6: Creating a Robot State Publisher Launch File

Let's create a launch file that brings up our robot model with RViz:

```bash
touch ~/ros2_ws/src/rviz_examples/launch/robot_rviz_launch.py
```

Add the following content to `~/ros2_ws/src/rviz_examples/launch/robot_rviz_launch.py`:

```python
#!/usr/bin/env python3
"""
Robot RViz Launch - brings up robot model with RViz visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate launch description for robot visualization
    """
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='humanoid_robot',
        description='Robot namespace'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='config_12dof',
        description='Robot model configuration'
    )

    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([
                FindPackageShare('rviz_examples'),
                'urdf',
                [model, '.urdf.xacro']
            ])
        ],
        remappings=[
            ('/joint_states', [namespace, '/joint_states'])
        ]
    )

    # Joint State Publisher (for manual joint control during debugging)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('show_gui', default='true'))
    )

    # RViz2 node with our custom configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('rviz_examples'),
        'rviz',
        'humanoid_robot_config.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Sample robot publisher (generates joint states and other data)
    robot_publisher = Node(
        package='rviz_examples',
        executable='sample_robot_publisher',
        name='sample_robot_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Event handler to log when RViz starts
    rviz_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz_node,
            on_start=[
                LogInfo(msg=['RViz2 started with configuration: humanoid_robot_config.rviz'])
            ]
        )
    )

    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        use_sim_time_arg,
        model_arg,
        DeclareLaunchArgument(
            'show_gui',
            default_value='true',
            description='Show joint state publisher GUI'
        ),

        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
        robot_publisher,

        # Event handlers
        rviz_start_handler,
    ])
```

---

## Step 7: Creating URDF for Visualization

Let's create a simple URDF file for our humanoid robot that will be visualized:

```bash
mkdir -p ~/ros2_ws/src/rviz_examples/urdf
touch ~/ros2_ws/src/rviz_examples/urdf/config_12dof.urdf.xacro
```

Add the following content to `~/ros2_ws/src/rviz_examples/urdf/config_12dof.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_12dof" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include common macros -->
  <xacro:include filename="$(find rviz_examples)/urdf/macros.xacro"/>

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <box_inertia m="10.0" x="0.3" y="0.3" z="0.6"/>
    </inertial>
  </link>

  <!-- Base footprint for navigation -->
  <link name="base_footprint">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.01" length="0.01"/>
      </geometry>
      <material name="transparent"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.15" length="0.01"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <cylinder_inertia m="0.001" r="0.15" h="0.01"/>
    </inertial>
  </link>

  <!-- Joint between base_footprint and base_link -->
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>

  <!-- Left Arm -->
  <!-- Shoulder pitch -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <cylinder_inertia m="1.0" r="0.05" h="0.3"/>
    </inertial>
  </link>

  <!-- Left elbow -->
  <joint name="joint2" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_forearm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="2.0"/>
  </joint>

  <link name="left_forearm">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <cylinder_inertia m="0.5" r="0.04" h="0.2"/>
    </inertial>
  </link>

  <!-- Left wrist roll -->
  <joint name="joint3" type="continuous">
    <parent link="left_forearm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" velocity="3.0"/>
  </joint>

  <link name="left_hand">
    <visual>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <cylinder_inertia m="0.2" r="0.03" h="0.05"/>
    </inertial>
  </link>

  <!-- Right Arm -->
  <!-- Right shoulder pitch -->
  <joint name="joint4" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.1 0.5" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <cylinder_inertia m="1.0" r="0.05" h="0.3"/>
    </inertial>
  </link>

  <!-- Right elbow -->
  <joint name="joint5" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_forearm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="2.0"/>
  </joint>

  <link name="right_forearm">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <cylinder_inertia m="0.5" r="0.04" h="0.2"/>
    </inertial>
  </link>

  <!-- Right wrist roll -->
  <joint name="joint6" type="continuous">
    <parent link="right_forearm"/>
    <child link="right_hand"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" velocity="3.0"/>
  </joint>

  <link name="right_hand">
    <visual>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <cylinder_inertia m="0.2" r="0.03" h="0.05"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <!-- Left hip pitch -->
  <joint name="joint7" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="-0.075 0.1 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1.5"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <cylinder_inertia m="2.0" r="0.06" h="0.4"/>
    </inertial>
  </link>

  <!-- Left knee -->
  <joint name="joint8" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="200" velocity="1.5"/>
  </joint>

  <link name="left_shin">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <cylinder_inertia m="1.5" r="0.05" h="0.3"/>
    </inertial>
  </link>

  <!-- Left ankle -->
  <joint name="joint9" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box_inertia m="0.8" x="0.15" y="0.08" z="0.05"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <!-- Right hip pitch -->
  <joint name="joint10" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="-0.075 -0.1 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1.5"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <cylinder_inertia m="2.0" r="0.06" h="0.4"/>
    </inertial>
  </link>

  <!-- Right knee -->
  <joint name="joint11" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="200" velocity="1.5"/>
  </joint>

  <link name="right_shin">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="brown"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <cylinder_inertia m="1.5" r="0.05" h="0.3"/>
    </inertial>
  </link>

  <!-- Right ankle -->
  <joint name="joint12" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box_inertia m="0.8" x="0.15" y="0.08" z="0.05"/>
    </inertial>
  </link>

  <!-- Camera link and joint -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_frame"/>
    <origin xyz="0.2 0 0.5" rpy="0 0 0"/>
  </joint>

  <link name="camera_frame">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box_inertia m="0.1" x="0.05" y="0.05" z="0.03"/>
    </inertial>
  </link>

  <!-- IMU link and joint -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_frame"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>

  <link name="imu_frame">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box_inertia m="0.01" x="0.01" y="0.01" z="0.01"/>
    </inertial>
  </link>

  <!-- LiDAR link and joint -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0.1 0 0.6" rpy="0 0 0"/>
  </joint>

  <link name="laser_frame">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <cylinder_inertia m="0.2" r="0.03" h="0.04"/>
    </inertial>
  </link>

</robot>
```

---

## Step 8: Building the RViz Examples Package

```bash
cd ~/ros2_ws
colcon build --packages-select rviz_examples
source ~/ros2_ws/install/setup.bash
```

---

## Step 9: Running the RViz Visualization

Now let's run our complete visualization system:

**Terminal 1 - Launch the complete system**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rviz_examples robot_rviz_launch.py
```

This should launch:
1. **Robot State Publisher** - Publishes the robot model with current joint states
2. **Joint State Publisher GUI** - Allows manual control of joint positions
3. **RViz2** - Visualizes the robot with our custom configuration
4. **Sample Robot Publisher** - Generates sample joint states and other data

You should see:
- The 12-DOF humanoid robot model in RViz
- Moving joint positions that animate over time
- Coordinate frames showing the TF tree
- Visualization markers showing robot path and obstacles
- Odometry information showing robot movement

---

## Step 10: RViz Display Configuration

### RobotModel Display

The RobotModel display visualizes your robot's URDF model:

**Configuration Options**:
- **Description Source**: Topic or file containing robot description
- **Description Topic**: Topic name for robot description (usually `/robot_description`)
- **Update Interval**: How often to update the visualization
- **Visual Enabled**: Whether to show visual geometry
- **Collision Enabled**: Whether to show collision geometry
- **Links**: Individual link visibility and materials

### TF Display

The TF display shows the relationship between coordinate frames:

**Configuration Options**:
- **Frame Timeout**: How long to keep frames in the display
- **Show Names**: Display frame names in the 3D view
- **Show Axes**: Show X (red), Y (green), Z (blue) axes
- **Show Arrows**: Show arrows indicating frame relationships
- **Frames**: Enable/disable specific frames individually
- **Tree**: View the hierarchical structure of frames

### Interactive Markers

Interactive markers allow you to manipulate objects in RViz:

```python
# Example interactive marker publisher
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

def create_interactive_marker(self):
    # Create an interactive marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "robot_control"
    int_marker.description = "Robot Control Interface"
    int_marker.pose.position.x = 1.0
    int_marker.pose.orientation.w = 1.0

    # Add a control that allows movement in the plane
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0

    # Add a box to the control
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.1
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.8
    marker.color.a = 1.0

    control.markers.append(marker)
    control.always_visible = True
    int_marker.controls.append(control)

    # Insert the marker
    self.server.insert(int_marker, self.marker_callback)
    self.server.applyChanges()
```

---

## Step 11: Advanced RViz Features

### Custom Display Plugins

You can create custom RViz display plugins for specialized visualization:

```python
# Example structure for a custom display plugin
import rclpy
from rclpy.node import Node
from rviz_common.display import Display
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

class CustomRobotDisplay(Display):
    """
    Custom display plugin for specialized robot visualization
    """
    def __init__(self):
        super().__init__()

        # Initialize the display
        self.robot_data_subscriber = None

    def on_initialize(self):
        """
        Called when the display is initialized
        """
        # Get the scene node for 3D rendering
        self.scene_node = self.context.get_scene_manager().getRootSceneNode().createChildSceneNode()

    def process_robot_data_message(self, msg):
        """
        Process incoming robot data and update visualization
        """
        # Update 3D objects based on message data
        pass
```

### RViz Configuration Management

Store and share RViz configurations:

```bash
# Save current RViz configuration
ros2 run rviz2 rviz2 -d /path/to/config.rviz

# Use a specific configuration file
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix rviz_examples)/share/rviz_examples/rviz/humanoid_robot_config.rviz
```

### Command Line Tools for RViz

```bash
# Echo transforms
ros2 run tf2_ros tf2_echo map base_link

# View the current frame tree
ros2 run tf2_tools view_frames

# Check transform quality
ros2 run tf2_ros tf2_monitor

# View specific topic data in RViz
ros2 topic echo /robot_description
```

---

## Step 12: Performance Optimization for RViz

### Efficient Visualization Practices

1. **Reduce Update Rates**: Don't update displays more frequently than needed
2. **Limit Point Clouds**: Downsample point clouds for visualization
3. **Use Simple Geometries**: Use basic shapes instead of complex meshes for real-time performance
4. **Selective Rendering**: Only render necessary elements
5. **Appropriate Buffer Sizes**: Set appropriate queue sizes for topics

### Visualization Best Practices

```python
# Example: Efficient point cloud visualization
def downsample_pointcloud(self, pointcloud_msg, factor=10):
    """
    Downsample point cloud for efficient visualization
    """
    # Only keep every nth point
    downsampled = PointCloud2()
    downsampled.header = pointcloud_msg.header
    downsampled.height = pointcloud_msg.height
    downsampled.width = pointcloud_msg.width // factor
    downsampled.fields = pointcloud_msg.fields
    downsampled.is_bigendian = pointcloud_msg.is_bigendian
    downsampled.point_step = pointcloud_msg.point_step
    downsampled.row_step = pointcloud_msg.row_step // factor
    downsampled.is_dense = pointcloud_msg.is_dense
    downsampled.data = pointcloud_msg.data[::factor * pointcloud_msg.point_step]

    return downsampled
```

### Memory Management

- **Buffer Management**: Limit transform buffer sizes to prevent memory bloat
- **Topic Filtering**: Only subscribe to necessary topics for visualization
- **Resource Cleanup**: Properly destroy visualization objects when not needed

---

## Step 13: RViz Integration with Real Robots

### Connecting to Hardware Robots

When connecting RViz to real robots, consider:

1. **Network Latency**: Account for network delays in visualization
2. **Bandwidth**: Optimize data transmission for remote visualization
3. **Clock Synchronization**: Ensure proper time synchronization
4. **Safety**: Don't rely solely on visualization for safety-critical decisions

### Remote Visualization

For remote robot monitoring:

```bash
# On robot side - forward robot data
ssh robot_ip
ros2 launch robot_bringup robot.launch.py

# On remote computer - forward display
ssh -X user@robot_ip
DISPLAY=:0 ros2 run rviz2 rviz2
```

---

## Step 14: Troubleshooting RViz Issues

### Common RViz Problems and Solutions

#### Issue 1: Robot Model Not Appearing

**Symptoms**: Robot model shows as wireframe or not at all

**Solutions**:
```bash
# Check robot description is being published
ros2 topic echo /robot_description

# Check joint states are being published
ros2 topic echo /joint_states

# Verify URDF is valid
check_urdf /path/to/robot.urdf

# Make sure robot_state_publisher is running
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=$(xacro robot.urdf.xacro)
```

#### Issue 2: TF Frames Not Visible

**Symptoms**: TF tree shows incomplete or missing frames

**Solutions**:
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Check specific transforms
ros2 run tf2_ros tf2_echo base_link camera_frame

# Verify transform publishing
ros2 topic list | grep tf
ros2 topic echo /tf
ros2 topic echo /tf_static
```

#### Issue 3: Performance Issues

**Symptoms**: RViz running slowly or consuming high CPU

**Solutions**:
- Reduce update rates for displays
- Disable unnecessary displays
- Downsample high-frequency data (point clouds, laser scans)
- Use simpler visualization options
- Close other graphics-intensive applications

#### Issue 4: Coordinate Frame Confusion

**Symptoms**: Robot appears in wrong position or orientation

**Solutions**:
- Check frame IDs match between URDF and TF publications
- Verify transform directions (parent-child relationships)
- Ensure consistent coordinate frame conventions (X-forward, Y-left, Z-up)
- Check that transforms have correct timestamps

---

## Lab Summary

In this lab, you've successfully:

✅ **Created custom message definitions** for robot state visualization
✅ **Configured RobotModel displays** to show URDF models with joint states
✅ **Implemented TF visualization** to show coordinate frame relationships
✅ **Created RViz configuration files** with multiple display types
✅ **Validated visualization setup** with `rviz2` and system monitoring

### Key Takeaways

- **RViz is essential** for robot development, debugging, and operation
- **Custom messages** enable domain-specific visualization of robot data
- **TF visualization** helps understand coordinate frame relationships
- **Efficient visualization** requires balancing detail with performance
- **Configuration files** enable reproducible visualization setups

---

## Next Steps

Now that you understand RViz integration, you're ready to explore:

- **Lab 7**: Launch files for system startup and management
- **Lab 8**: Advanced navigation and path planning
- **Lab 9**: Perception and computer vision systems

**Continue to [Lab 7: Launch Files](./lab07-launch-files.md)**

---
**Previous**: [Lab 5: Humanoid URDF](./lab05-humanoid-urdf.md) | **Next**: [Lab 7: Launch Files](./lab07-launch-files.md)