---
title: Lab 7 - Launch Files
sidebar_label: Lab 7 - Launch Files
sidebar_position: 12
description: Master ROS 2 launch files with Python and XML syntax, implement robot bringup launch files, parameter management, and multi-node orchestration for complex robotic systems
---

# Lab 7: Launch Files

## Overview

In this lab, you'll master ROS 2 launch files using both Python and XML syntax. You'll implement robot bringup launch files, parameter management systems, and multi-node orchestration for complex robotic systems. Launch files are essential for starting complex robot systems with proper parameter configuration, node ordering, and system management.

**Duration**: 2 hours

**Learning Objectives**:
- ✅ Create launch files using Python syntax with LaunchDescription and Node actions
- ✅ Implement parameter management with YAML files and command-line overrides
- ✅ Design robot bringup launch files for multi-package system startup
- ✅ Use launch arguments, conditions, and event handlers for flexible system configuration
- ✅ Validate launch file functionality with `ros2 launch` commands and system monitoring

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **Completed Lab 1-6** - All previous labs covering basic ROS 2 concepts
✅ **ROS 2 Humble installed** with all standard packages
✅ **Python programming skills** for implementing launch files
✅ **Understanding of** ROS 2 nodes, parameters, and system architecture
✅ **Experience with** command-line tools and file management

---

## Step 1: Understanding Launch File Concepts

### What are Launch Files?

**Launch files** in ROS 2 allow you to start multiple nodes with specific configurations simultaneously. They provide:

- **Centralized system startup**: Start complex robot systems with a single command
- **Parameter management**: Configure nodes with parameters from YAML files
- **Node organization**: Manage dependencies and startup order
- **Flexible configuration**: Use arguments to customize system behavior
- **Error handling**: Manage node failures and restart policies

### Launch File Formats

ROS 2 supports multiple launch file formats:

| Format | Syntax | Best For | Pros | Cons |
|--------|--------|----------|------|------|
| **Python** | Python code | Complex logic, programmatic control | Full Python power, extensible | Verbose for simple setups |
| **XML** | XML markup | Simple configurations | Concise, readable | Limited logic capabilities |
| **YAML** | YAML format | Parameter definitions | Human-readable, structured | Not for node launching |

---

## Step 2: Creating a Launch Package

First, let's create a package for our launch examples:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python launch_examples --dependencies rclpy std_msgs geometry_msgs sensor_msgs
```

Create the launch directory:
```bash
mkdir -p ~/ros2_ws/src/launch_examples/launch
mkdir -p ~/ros2_ws/src/launch_examples/config
mkdir -p ~/ros2_ws/src/launch_examples/launch_examples
```

---

## Step 3: Basic Python Launch File

Let's create our first launch file. Create the launch file:

```bash
touch ~/ros2_ws/src/launch_examples/launch/basic_launch_example.py
```

Add the following content to `~/ros2_ws/src/launch_examples/launch/basic_launch_example.py`:

```python
#!/usr/bin/env python3
"""
Basic Launch Example - demonstrates fundamental launch file concepts
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate the launch description containing all nodes to launch
    """
    # Define launch arguments that can be passed to the launch file
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Create a simple talker node
    talker_node = Node(
        package='demo_nodes_cpp',  # Using demo package from ROS 2
        executable='talker',       # The executable name
        name='basic_talker',       # Custom node name
        parameters=[{'use_sim_time': use_sim_time}],  # Pass parameters
        remappings=[('/chatter', '/basic_chatter')],  # Remap topics
        output='screen'            # Output to screen for debugging
    )

    # Create a simple listener node
    listener_node = Node(
        package='demo_nodes_cpp',
        executable='listener',
        name='basic_listener',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/chatter', '/basic_chatter')],  # Same remapping as talker
        output='screen'
    )

    # Return the complete launch description
    return LaunchDescription([
        use_sim_time_arg,  # Declare the argument first
        talker_node,       # Add the talker node
        listener_node      # Add the listener node
    ])
```

---

## Step 4: Launch Arguments and Conditions

Let's create a more advanced launch file that demonstrates arguments and conditions:

```bash
touch ~/ros2_ws/src/launch_examples/launch/advanced_launch_example.py
```

Add the following content to `~/ros2_ws/src/launch_examples/launch/advanced_launch_example.py`:

```python
#!/usr/bin/env python3
"""
Advanced Launch Example - demonstrates launch arguments, conditions, and parameter management
"""

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate advanced launch description with arguments, conditions, and parameter management
    """
    # Define launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Namespace for all nodes'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable RViz visualization'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='turtlebot4',
        choices=['turtlebot4', 'diffdrive', 'omnidrive'],
        description='Robot model type for configuration'
    )

    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_visualization = LaunchConfiguration('enable_visualization')
    robot_model = LaunchConfiguration('robot_model')

    # Conditional nodes - only launch if conditions are met
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        condition=IfCondition(enable_visualization),  # Only run if enable_visualization is true
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(
            get_package_share_directory('launch_examples'),
            'rviz',
            'basic_config.rviz'
        )]
    )

    # Create nodes with different configurations based on robot model
    # Using PythonExpression to conditionally set parameters
    robot_controller_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        # Remap based on namespace
        remappings=[
            ('/cmd_vel', [namespace, '/cmd_vel'])
        ],
        # Only run if visualization is disabled (avoid conflicts)
        condition=UnlessCondition(enable_visualization)
    )

    # Create a group of nodes that should be launched together
    sensor_processing_group = GroupAction(
        actions=[
            Node(
                package='diagnostic_aggregator',
                executable='aggregator_node',
                name='diagnostic_aggregator',
                parameters=[os.path.join(
                    get_package_share_directory('launch_examples'),
                    'config',
                    'diagnostics.yaml'
                )],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': PythonExpression([
                        '"xacro ", ',
                        os.path.join(
                            get_package_share_directory('launch_examples'),
                        'urdf',
                        PythonExpression(['"', robot_model, '.urdf.xacro"'])
                    )
                ])},
                output='screen'
            )
        ]
    )

    # Log information based on conditions
    model_info = LogInfo(
        msg=['Launching for robot model: ', robot_model],
        condition=IfCondition(
            PythonExpression(["'", robot_model, "' == 'turtlebot4'"])
        )
    )

    # Return the complete launch description
    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        enable_visualization_arg,
        robot_model_arg,

        # Conditional nodes
        rviz_node,
        robot_controller_node,

        # Groups
        sensor_processing_group,

        # Conditional logging
        model_info,
    ])
```

---

## Step 5: Robot Bringup Launch File

Now let's create a realistic robot bringup launch file that simulates bringing up a complete robot system:

```bash
touch ~/ros2_ws/src/launch_examples/launch/robot_bringup_launch.py
```

Add the following content to `~/ros2_ws/src/launch_examples/launch/robot_bringup_launch.py`:

```python
#!/usr/bin/env python3
"""
Robot Bringup Launch - complete robot system startup with parameter management
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetParameter
import os


def generate_launch_description():
    """
    Generate complete robot bringup launch description
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

    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable detailed logging'
    )

    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='config_12dof',
        description='Robot configuration file name'
    )

    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_logging = LaunchConfiguration('enable_logging')
    robot_config = LaunchConfiguration('robot_config')

    # Set global parameters for all nodes in this launch
    set_use_sim_time_param = SetParameter(name='use_sim_time', value=use_sim_time)

    # Robot state publisher with URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[
            os.path.join(
                get_package_share_directory('launch_examples'),
                'config',
                [robot_config, '.yaml']
            ),
            {'robot_description':
                PathJoinSubstitution([
                    get_package_share_directory('launch_examples'),
                    'urdf',
                    [robot_config, '.urdf.xacro']
                ])
            }
        ],
        output='screen'
    )

    # Joint state publisher (for simulation without real hardware)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(
                get_package_share_directory('launch_examples'),
                'config',
                'joint_publisher.yaml'
            )
        ],
        output='screen'
    )

    # Joint state publisher GUI (for manual joint control during debugging)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=namespace,
        condition=IfCondition(enable_logging),  # Only if logging is enabled
        output='screen'
    )

    # IMU driver node
    imu_driver = Node(
        package='imu_driver_package',  # Replace with actual package
        executable='imu_driver',
        name='imu_driver',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(
                get_package_share_directory('launch_examples'),
                'config',
                'imu_config.yaml'
            )
        ],
        output='screen',
        respawn=True,  # Restart if it crashes
        respawn_delay=2.0
    )

    # Camera driver node
    camera_driver = Node(
        package='camera_driver_package',  # Replace with actual package
        executable='camera_driver',
        name='camera_driver',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(
                get_package_share_directory('launch_examples'),
                'config',
                'camera_config.yaml'
            )
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # LiDAR driver node
    lidar_driver = Node(
        package='lidar_driver_package',  # Replace with actual package
        executable='lidar_driver',
        name='lidar_driver',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(
                get_package_share_directory('launch_examples'),
                'config',
                'lidar_config.yaml'
            )
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Robot controller node
    robot_controller = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='robot_controller',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(
                get_package_share_directory('launch_examples'),
                'config',
                'controllers.yaml'
            )
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Diagnostic aggregator
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(
                get_package_share_directory('launch_examples'),
                'config',
                'diagnostics.yaml'
            )
        ],
        output='screen'
    )

    # Event handler example - run something when robot controller starts
    controller_started_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_controller,
            on_start=[
                LogInfo(msg=['Robot controller started for namespace: ', namespace])
            ]
        )
    )

    # Timer-based node startup (delayed launch)
    delayed_nodes = TimerAction(
        period=5.0,  # Wait 5 seconds before launching
        actions=[
            Node(
                package='health_monitor',
                executable='health_monitor_node',
                name='health_monitor',
                namespace=namespace,
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )

    # Return the complete launch description
    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        use_sim_time_arg,
        enable_logging_arg,
        robot_config_arg,

        # Global parameters
        set_use_sim_time_param,

        # Robot bringup nodes
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        imu_driver,
        camera_driver,
        lidar_driver,
        robot_controller,
        diagnostic_aggregator,

        # Event handlers
        controller_started_handler,

        # Delayed nodes
        delayed_nodes,
    ])
```

---

## Step 6: Parameter Management

Let's create parameter configuration files that can be used with our launch files:

Create the config directory and parameter files:

```bash
mkdir -p ~/ros2_ws/src/launch_examples/config
```

Create the main robot parameters file:
```bash
touch ~/ros2_ws/src/launch_examples/config/config_12dof.yaml
```

Add the following content to `~/ros2_ws/src/launch_examples/config/config_12dof.yaml`:

```yaml
/**:
  ros__parameters:
    use_sim_time: false

humanoid_robot.robot_state_publisher:
  ros__parameters:
    publish_frequency: 50.0
    use_tf_static: true
    ignore_timestamp: false

humanoid_robot.joint_state_publisher:
  ros__parameters:
    rate: 50
    source_list: ["joint_states"]

humanoid_robot.imu_driver:
  ros__parameters:
    frame_id: "imu_link"
    linear_acceleration_variance: 0.01
    angular_velocity_variance: 0.01
    orientation_variance: 0.01

humanoid_robot.camera_driver:
  ros__parameters:
    camera_frame: "camera_link"
    image_width: 640
    image_height: 480
    fps: 30
    exposure: 100

humanoid_robot.lidar_driver:
  ros__parameters:
    frame_id: "laser_link"
    range_min: 0.1
    range_max: 30.0
    angle_min: -2.35619  # -135 degrees
    angle_max: 2.35619   # 135 degrees
    angle_increment: 0.00436  # 0.25 degrees

humanoid_robot.robot_controller:
  ros__parameters:
    joint_state_controller:
      type: joint_state_controller/JointStateController
      publish_rate: 50
    position_controllers:
      - joint1_position_controller
      - joint2_position_controller
      - joint3_position_controller
      - joint4_position_controller
      - joint5_position_controller
      - joint6_position_controller
      - joint7_position_controller
      - joint8_position_controller
      - joint9_position_controller
      - joint10_position_controller
      - joint11_position_controller
      - joint12_position_controller
```

Create the joint publisher configuration:
```bash
touch ~/ros2_ws/src/launch_examples/config/joint_publisher.yaml
```

Add the following content to `~/ros2_ws/src/launch_examples/config/joint_publisher.yaml`:

```yaml
/**:
  ros__parameters:
    use_sim_time: false
    rate: 50
    source_list: ["joint_states"]

    # Joint limits and initial positions
    joint_state_publisher:
      ignore_timestamp: false

    # Specific joint configurations
    joint1_initial_position: 0.0
    joint2_initial_position: 0.0
    joint3_initial_position: 0.0
    joint4_initial_position: 0.0
    joint5_initial_position: 0.0
    joint6_initial_position: 0.0
    joint7_initial_position: 0.0
    joint8_initial_position: 0.0
    joint9_initial_position: 0.0
    joint10_initial_position: 0.0
    joint11_initial_position: 0.0
    joint12_initial_position: 0.0

    # Joint limits
    joint_limits:
      joint1:
        has_position_limits: true
        min_position: -3.14
        max_position: 3.14
      joint2:
        has_position_limits: true
        min_position: -1.57
        max_position: 1.57
      # Add limits for all 12 joints...
```

Create the controller configuration:
```bash
touch ~/ros2_ws/src/launch_examples/config/controllers.yaml
```

Add the following content to `~/ros2_ws/src/launch_examples/config/controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    use_sim_time: false

joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

joint1_position_controller:
  type: position_controllers/JointPositionController

joint2_position_controller:
  type: position_controllers/JointPositionController

joint3_position_controller:
  type: position_controllers/JointPositionController

joint4_position_controller:
  type: position_controllers/JointPositionController

joint5_position_controller:
  type: position_controllers/JointPositionController

joint6_position_controller:
  type: position_controllers/JointPositionController

joint7_position_controller:
  type: position_controllers/JointPositionController

joint8_position_controller:
  type: position_controllers/JointPositionController

joint9_position_controller:
  type: position_controllers/JointPositionController

joint10_position_controller:
  type: position_controllers/JointPositionController

joint11_position_controller:
  type: position_controllers/JointPositionController

joint12_position_controller:
  type: position_controllers/JointPositionController

# Controller-specific parameters
joint1_position_controller:
  ros__parameters:
    joint: joint1
    interface_name: position

joint2_position_controller:
  ros__parameters:
    joint: joint2
    interface_name: position

# Continue for all joints...
```

---

## Step 7: Launch File with Included Launch Descriptions

Let's create a launch file that includes other launch files:

```bash
touch ~/ros2_ws/src/launch_examples/launch/system_launch.py
```

Add the following content to `~/ros2_ws/src/launch_examples/launch/system_launch.py`:

```python
#!/usr/bin/env python3
"""
System Launch - includes multiple launch files for complete system startup
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Generate system launch description that includes other launch files
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

    # Get launch configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include the robot bringup launch file
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('launch_examples'),
            '/launch/robot_bringup_launch.py'
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Include a navigation launch file (assuming it exists in navigation package)
    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('nav2_bringup'),
    #         '/launch/navigation_launch.py'
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time
    #     }.items(),
    #     condition=IfCondition(LaunchConfiguration('enable_navigation', default='false'))
    # )

    # Include a perception launch file
    # perception_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('perception_package'),
    #         '/launch/perception_launch.py'
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'namespace': namespace
    #     }.items()
    # )

    # Group of optional nodes
    optional_nodes = GroupAction(
        actions=[
            # Include launch files for optional functionality
            # This could include things like:
            # - Mapping modules
            # - Advanced perception
            # - Specialized controllers
        ]
    )

    # Set environment variables if needed
    set_env_vars = SetEnvironmentVariable(
        name='RCUTILS_LOGGING_BUFFERED_STREAM',
        value='1'
    )

    return LaunchDescription([
        # Launch arguments
        namespace_arg,
        use_sim_time_arg,

        # Environment variables
        set_env_vars,

        # Included launch files
        robot_bringup_launch,
        # navigation_launch,
        # perception_launch,

        # Optional nodes
        optional_nodes,
    ])
```

---

## Step 8: XML Launch File Example

Although Python is preferred for complex logic, let's also create an XML launch file for comparison:

```bash
touch ~/ros2_ws/src/launch_examples/launch/simple_example.launch.xml
```

Add the following content to `~/ros2_ws/src/launch_examples/launch/simple_example.launch.xml`:

```xml
<launch>
  <!-- Declare launch arguments -->
  <arg name="use_sim_time" default="false" description="Use simulation clock if true"/>
  <arg name="robot_namespace" default="robot1" description="Robot namespace"/>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="robot_description" value="
      $(find-pkg-share launch_examples)/urdf/robot.urdf"/>
    <remap from="/joint_states" to="$(var robot_namespace)/joint_states"/>
  </node>

  <!-- Joint state publisher -->
  <node pkg="joint_state_publisher" exec="joint_state_publisher" name="joint_state_publisher">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="rate" value="50"/>
  </node>

  <!-- Conditional launch of GUI -->
  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"
        if="$(eval arg('use_sim_time') == 'false')">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Teleoperation node -->
  <node pkg="teleop_twist_keyboard" exec="teleop_twist_keyboard" name="teleop_twist_keyboard">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <remap from="/cmd_vel" to="$(var robot_namespace)/cmd_vel"/>
  </node>

</launch>
```

---

## Step 9: Update Package Configuration

Update the setup.py file for the launch examples package:

Edit `~/ros2_ws/src/launch_examples/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'launch_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', [
            'launch/basic_launch_example.py',
            'launch/advanced_launch_example.py',
            'launch/robot_bringup_launch.py',
            'launch/system_launch.py',
            'launch/simple_example.launch.xml',
        ]),
        # Include config files
        ('share/' + package_name + '/config', [
            'config/config_12dof.yaml',
            'config/joint_publisher.yaml',
            'config/controllers.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Launch file examples for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

Update the package.xml file:

Edit `~/ros2_ws/src/launch_examples/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>launch_examples</name>
  <version>0.0.0</version>
  <description>Launch file examples for ROS 2</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>launch</depend>
  <depend>launch_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>