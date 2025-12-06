---
title: URDF Fundamentals
sidebar_label: URDF Fundamentals
sidebar_position: 8
description: Learn the fundamentals of Universal Robot Description Format (URDF) for robot modeling, including links, joints, visual/collision geometry, and hierarchical robot structure definition
---

# URDF Fundamentals

## Overview

In this section, you'll learn the fundamentals of Universal Robot Description Format (URDF) for robot modeling. You'll understand links, joints, visual and collision geometry, and how to define hierarchical robot structures. URDF is essential for representing robot models in ROS 2, enabling simulation, visualization, and motion planning.

**Duration**: 1.5 hours

**Learning Objectives**:
- ✅ Define robot structure using links and joints in URDF XML
- ✅ Configure visual and collision geometry with meshes and primitives
- ✅ Create hierarchical robot structures with proper parent-child relationships
- ✅ Validate URDF models with `check_urdf` command
- ✅ Implement 5-DOF humanoid arm with torso, shoulders, elbows, and wrists

---

## Prerequisites

Before starting this section, ensure you have:

✅ **Completed Lab 1** - Talker/Listener basics
✅ **Completed Lab 2** - Custom messages
✅ **Completed Lab 3** - Services
✅ **Completed Lab 4** - Actions
✅ **ROS 2 Humble installed** with all standard packages
✅ **Basic understanding** of ROS 2 nodes, topics, services, and actions
✅ **XML knowledge** for understanding URDF structure

---

## Step 1: Understanding URDF Concepts

### What is URDF?

**URDF (Unified Robot Description Format)** is an XML format used to represent robot models in ROS. It defines:
- **Links**: Rigid bodies (parts of the robot)
- **Joints**: Connections between links
- **Visual**: How the robot looks (for visualization)
- **Collision**: How the robot interacts with the environment (for physics simulation)

### URDF vs Xacro

While URDF is pure XML, **Xacro (XML Macros)** extends URDF with:
- **Macros**: Reusable components
- **Math expressions**: Calculations within the XML
- **Parameters**: Configurable values
- **Inclusions**: Import other Xacro files

### Basic URDF Structure

```
<robot name="robot_name">
  <!-- Links: rigid bodies -->
  <link name="link_name">
    <visual>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 1 1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints: connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="parent_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  </joint>
</robot>
```

---

## Step 2: URDF Links

### Link Definition

A **link** represents a rigid body part of the robot:

```xml
<link name="link_name">
  <!-- Visual appearance -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- One of: box, cylinder, sphere, mesh -->
      <box size="1 1 1"/>
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision geometry -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>

  <!-- Physical properties for simulation -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
  </inertial>
</link>
```

### Link Elements

| Element | Purpose | Required |
|---------|---------|----------|
| `name` | Unique identifier for the link | Yes |
| `visual` | How the link appears in visualization | No |
| `collision` | How the link interacts in physics simulation | No |
| `inertial` | Mass and inertia properties for dynamics | Yes (for simulation) |

### Visual Properties

The visual element defines how a link appears:

```xml
<visual>
  <!-- Position and orientation relative to link origin -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Shape of the visual representation -->
  <geometry>
    <!-- Box: rectangular prism -->
    <box size="length width height"/>

    <!-- Cylinder: cylindrical shape -->
    <cylinder radius="0.1" length="1.0"/>

    <!-- Sphere: spherical shape -->
    <sphere radius="0.1"/>

    <!-- Mesh: complex shape from file -->
    <mesh filename="package://robot_description/meshes/link.dae"/>
  </geometry>

  <!-- Color/material -->
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>
</visual>
```

### Collision Properties

The collision element defines physical interaction:

```xml
<collision>
  <!-- Position and orientation relative to link origin -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Shape of the collision geometry -->
  <geometry>
    <!-- Often simplified compared to visual geometry for performance -->
    <box size="1 1 1"/>
    <!-- or <cylinder/>, <sphere/>, <mesh/> -->
  </geometry>
</collision>
```

### Inertial Properties

The inertial element defines physical properties for simulation:

```xml
<inertial>
  <!-- Mass in kg -->
  <mass value="1.0"/>

  <!-- Origin relative to link origin -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Inertia tensor (calculated for the shape) -->
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</inertial>
```

For common shapes, use these formulas:
- **Box**: `ixx = m/12 * (h² + d²)` where m=mass, h=height, d=depth
- **Cylinder**: `ixx = m/12 * (3*r² + h²)`, `izz = m/2 * r²`
- **Sphere**: `ixx = iyy = izz = 2/5 * m * r²`

---

## Step 3: URDF Joints

### Joint Definition

A **joint** connects two links with a specific type of motion:

```xml
<joint name="joint_name" type="joint_type">
  <!-- Parent link (closer to robot base/root) -->
  <parent link="parent_link_name"/>

  <!-- Child link (further from robot base/root) -->
  <child link="child_link_name"/>

  <!-- Position and orientation of joint relative to parent link -->
  <origin xyz="0 0 1" rpy="0 0 0"/>

  <!-- Axis of rotation/translation (for revolute/prismatic joints) -->
  <axis xyz="0 0 1"/>

  <!-- Joint limits (for revolute and prismatic joints) -->
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
</joint>
```

### Joint Types

| Type | Motion | Parameters |
|------|--------|------------|
| `revolute` | Rotational (limited) | `lower`, `upper`, `effort`, `velocity` |
| `continuous` | Rotational (unlimited) | `effort`, `velocity` |
| `prismatic` | Linear (limited) | `lower`, `upper`, `effort`, `velocity` |
| `fixed` | No motion | None |
| `floating` | 6DOF (simulation) | None |
| `planar` | Planar motion | None |

### Joint Limits

Joint limits constrain motion:

```xml
<limit
  lower="-1.57"           <!-- Minimum position (radians for revolute) -->
  upper="1.57"            <!-- Maximum position (radians for revolute) -->
  effort="100.0"          <!-- Maximum torque/force (N-m or N) -->
  velocity="2.0"          <!-- Maximum velocity (rad/s or m/s) -->
/>
```

### Joint Axes

The axis defines the direction of motion:

```xml
<!-- Rotate around Z-axis -->
<axis xyz="0 0 1"/>

<!-- Rotate around Y-axis -->
<axis xyz="0 1 0"/>

<!-- Translate along X-axis -->
<axis xyz="1 0 0"/>
```

---

## Step 4: Creating a Simple Robot Model

Let's create a simple 5-DOF humanoid arm model. First, create the necessary directories:

```bash
mkdir -p ~/ros2_ws/src/humanoid_description/urdf
mkdir -p ~/ros2_ws/src/humanoid_description/meshes
mkdir -p ~/ros2_ws/src/humanoid_description/config
```

Create the main URDF file:
```bash
touch ~/ros2_ws/src/humanoid_description/urdf/humanoid_arm.urdf
```

Add the following content to `~/ros2_ws/src/humanoid_description/urdf/humanoid_arm.urdf`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm">

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <inertia ixx="0.15" ixy="0" ixz="0" iyy="0.15" iyz="0" izz="0.075"/>
    </inertial>
  </link>

  <!-- Shoulder pitch joint -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0.15 0.6" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>  <!-- Rotate around X-axis -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
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
      <inertia ixx="0.0075" ixy="0" ixz="0" iyy="0.0075" iyz="0" izz="0.00125"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>  <!-- Rotate around X-axis -->
    <limit lower="0" upper="3.14" effort="100" velocity="2.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1.0"/>
      </material>
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
      <inertia ixx="0.00167" ixy="0" ixz="0" iyy="0.00167" iyz="0" izz="0.0004"/>
    </inertial>
  </link>

  <!-- Wrist roll joint -->
  <joint name="wrist_roll" type="continuous">
    <parent link="forearm"/>
    <child link="hand"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Rotate around Z-axis -->
    <limit effort="50" velocity="3.0"/>
  </joint>

  <!-- Hand link -->
  <link name="hand">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
      <material name="yellow">
        <color rgba="0.8 0.8 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.000167" ixy="0" ixz="0" iyy="0.000167" iyz="0" izz="0.00009"/>
    </inertial>
  </link>

</robot>
```

---

## Step 5: Validating URDF Models

### Using check_urdf

Always validate your URDF models:

```bash
# Check if the URDF is syntactically correct
check_urdf ~/ros2_ws/src/humanoid_description/urdf/humanoid_arm.urdf
```

**Expected Output**:
```
robot name is: humanoid_arm
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  upper_arm
        child(1):  forearm
            child(1):  hand
```

### Using xacro for Validation

If you have a Xacro file, convert it to URDF first:

```bash
# Convert Xacro to URDF and validate
xacro ~/ros2_ws/src/humanoid_description/urdf/humanoid_arm.xacro > temp.urdf
check_urdf temp.urdf
rm temp.urdf
```

### Common URDF Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| Invalid XML | Parser errors | Check XML syntax, unclosed tags |
| Missing joints | Floating links | Connect all links with joints |
| Circular dependencies | Infinite loop | Ensure DAG structure |
| Invalid joint types | Parsing errors | Use valid joint types |
| Missing parent/child links | Validation errors | Ensure referenced links exist |

---

## Step 6: Visualizing URDF Models

### Using RViz

Load your URDF in RViz for visualization:

```bash
# Launch RViz with robot model
ros2 run rviz2 rviz2
```

In RViz:
1. Add a RobotModel display
2. Set the Robot Description to "robot_description" (or your parameter name)
3. Load the URDF via robot_state_publisher

### Using robot_state_publisher

Launch a static robot model:

```bash
# Create a launch file to visualize the URDF
mkdir -p ~/ros2_ws/src/humanoid_description/launch
touch ~/ros2_ws/src/humanoid_description/launch/view_model.launch.py
```

Add the following to `~/ros2_ws/src/humanoid_description/launch/view_model.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'humanoid_arm.urdf'
    )

    with open(robot_description_path, 'r') as infp:
        robot_desc = infp.read()

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for robot_state_publisher'
    )

    # Robot state publisher node
    params = {
        'robot_description': robot_desc,
        'use_sim_time': False
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        namespace=LaunchConfiguration('namespace')
    )

    return LaunchDescription([
        namespace_arg,
        node_robot_state_publisher
    ])
```

---

## Step 7: Creating a URDF Package

Let's create a proper ROS 2 package for our URDF:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake humanoid_description
```

Update the CMakeLists.txt to install URDF files:

Edit `~/ros2_ws/src/humanoid_description/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(humanoid_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install URDF files
install(DIRECTORY
  urdf
  meshes
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

Update the package.xml:

Edit `~/ros2_ws/src/humanoid_description/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_description</name>
  <version>0.0.0</version>
  <description>URDF description for humanoid robot model</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Step 8: Building the URDF Package

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_description
source ~/ros2_ws/install/setup.bash
```

---

## Step 9: Advanced URDF Features

### Materials and Colors

Define materials in a separate file for reusability:

Create `~/ros2_ws/src/humanoid_description/urdf/materials.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <material name="Black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <material name="Blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <material name="Green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>

  <material name="Grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

  <material name="Orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>

  <material name="Brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>

  <material name="Red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>

  <material name="White">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

</robot>
```

### Xacro Macros for Reusability

Create reusable components with Xacro macros:

Create `~/ros2_ws/src/humanoid_description/urdf/common_functions.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro to create a simple cylinder link -->
  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertia
      ixx="${m*(3*r*r+h*h)/12}"
      ixy="0"
      ixz="0"
      iyy="${m*(3*r*r+h*h)/12}"
      iyz="0"
      izz="${m*r*r/2}"/>
  </xacro:macro>

  <!-- Macro to create a simple box link -->
  <xacro:macro name="box_inertia" params="m w h d">
    <inertia
      ixx="${m*(h*h+d*d)/12}"
      ixy="0"
      ixz="0"
      iyy="${m*(w*w+d*d)/12}"
      iyz="0"
      izz="${m*(w*w+h*h)/12}"/>
  </xacro:macro>

  <!-- Macro to create a sphere link -->
  <xacro:macro name="sphere_inertia" params="m r">
    <inertia
      ixx="${2*m*r*r/5}"
      ixy="0"
      ixz="0"
      iyy="${2*m*r*r/5}"
      iyz="0"
      izz="${2*m*r*r/5}"/>
  </xacro:macro>

</robot>
```

### Complete Xacro Robot Model

Create a Xacro version of our robot:

Create `~/ros2_ws/src/humanoid_description/urdf/humanoid_arm.xacro`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include common functions and materials -->
  <xacro:include filename="$(find humanoid_description)/urdf/common_functions.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/materials.xacro"/>

  <!-- Robot parameters -->
  <xacro:property name="base_length" value="0.6"/>
  <xacro:property name="base_width" value="0.3"/>
  <xacro:property name="base_height" value="0.3"/>
  <xacro:property name="upper_arm_length" value="0.3"/>
  <xacro:property name="upper_arm_radius" value="0.05"/>
  <xacro:property name="forearm_length" value="0.2"/>
  <xacro:property name="forearm_radius" value="0.04"/>
  <xacro:property name="hand_length" value="0.1"/>
  <xacro:property name="hand_radius" value="0.03"/>

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${base_length/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_width} ${base_height} ${base_length}"/>
      </geometry>
      <material name="Grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${base_length/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${base_width} ${base_height} ${base_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 ${base_length/2}" rpy="0 0 0"/>
      <xacro:box_inertia m="5.0" w="${base_width}" h="${base_length}" d="${base_height}"/>
    </inertial>
  </link>

  <!-- Shoulder pitch joint -->
  <joint name="shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 ${base_height/2} ${base_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>
      </geometry>
      <material name="Blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${upper_arm_radius}" length="${upper_arm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
      <xacro:cylinder_inertia m="1.0" r="${upper_arm_radius}" h="${upper_arm_length}"/>
    </inertial>
  </link>

  <!-- Elbow joint -->
  <joint name="elbow" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 ${upper_arm_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="2.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm">
    <visual>
      <origin xyz="0 0 ${forearm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
      <material name="Green"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${forearm_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${forearm_radius}" length="${forearm_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 ${forearm_length/2}" rpy="0 0 0"/>
      <xacro:cylinder_inertia m="0.5" r="${forearm_radius}" h="${forearm_length}"/>
    </inertial>
  </link>

  <!-- Wrist roll joint -->
  <joint name="wrist_roll" type="continuous">
    <parent link="forearm"/>
    <child link="hand"/>
    <origin xyz="0 0 ${forearm_length}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" velocity="3.0"/>
  </joint>

  <!-- Hand link -->
  <link name="hand">
    <visual>
      <origin xyz="0 0 ${hand_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${hand_radius}" length="${hand_length}"/>
      </geometry>
      <material name="Yellow"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${hand_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${hand_radius}" length="${hand_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0 0 ${hand_length/2}" rpy="0 0 0"/>
      <xacro:cylinder_inertia m="0.2" r="${hand_radius}" h="${hand_length}"/>
    </inertial>
  </link>

</robot>
```

---

## Step 10: Testing and Validating the Xacro Model

Convert and validate the Xacro model:

```bash
# Convert Xacro to URDF
xacro ~/ros2_ws/src/humanoid_description/urdf/humanoid_arm.xacro > ~/ros2_ws/src/humanoid_description/urdf/humanoid_arm_converted.urdf

# Validate the converted URDF
check_urdf ~/ros2_ws/src/humanoid_description/urdf/humanoid_arm_converted.urdf

# Clean up
rm ~/ros2_ws/src/humanoid_description/urdf/humanoid_arm_converted.urdf
```

---

## Step 11: URDF Best Practices

### Naming Conventions

- **Links**: Use descriptive names (e.g., `base_link`, `left_wheel`, `camera_mount`)
- **Joints**: Use descriptive names (e.g., `shoulder_pitch`, `wheel_rotation`)
- **Consistency**: Use consistent naming patterns throughout the model

### Structure Organization

- **Hierarchy**: Organize from base/root to end-effectors
- **Mass distribution**: Place heavier components near the base
- **Joint limits**: Set realistic limits based on physical constraints

### Performance Considerations

- **Collision geometry**: Use simplified shapes for collision (boxes/cylinders instead of complex meshes)
- **Meshes**: Use lightweight meshes for visualization
- **Inertias**: Use approximate values for simulation stability

### Reusability

- **Xacro macros**: Create reusable components
- **Parameters**: Use properties for configurable dimensions
- **Modularity**: Break complex robots into sub-assemblies

---

## Step 12: Troubleshooting Common URDF Issues

### Issue 1: Invalid Joint Parents/Children

**Symptoms**: `No child link 'xxx' for joint 'yyy'` or `No parent link 'xxx' for joint 'yyy'`

**Solutions**:
```bash
# Check that all referenced links exist in your URDF
grep -E "link=\"|child|parent" your_robot.urdf
```

### Issue 2: Circular Dependencies

**Symptoms**: Infinite loop during validation

**Solutions**: Ensure the robot model forms a Directed Acyclic Graph (DAG)

### Issue 3: Physics Instability

**Symptoms**: Robot parts flying apart in simulation

**Solutions**:
- Check that all masses are reasonable (not zero or extremely large)
- Verify that inertias are calculated correctly
- Ensure joints have proper limits and types

### Issue 4: Visualization Problems

**Symptoms**: Parts not showing up or appearing in wrong positions

**Solutions**:
- Verify that origins and axes are correctly defined
- Check that visual and collision geometries are properly positioned
- Use RViz to debug the robot model

---

## Section Summary

In this section, you've successfully:

✅ **Learned URDF fundamentals** including links, joints, visual/collision geometry
✅ **Created a 5-DOF humanoid arm** with proper parent-child relationships
✅ **Configured visual and collision geometry** with primitive shapes
✅ **Validated URDF models** with `check_urdf` command
✅ **Implemented reusable Xacro macros** for component definition

### Key Takeaways

- **URDF** defines robot structure with links (rigid bodies) and joints (connections)
- **Visual geometry** determines appearance; **collision geometry** determines physics
- **Xacro macros** enable reusable, parameterized robot components
- **Proper hierarchy** ensures correct kinematic chain formation
- **Validation** with `check_urdf` prevents runtime errors

---

## Next Steps

Now that you understand URDF fundamentals, you're ready to explore:

- **Lab 5**: Complete humanoid URDF with legs and head
- **Lab 6**: TF2 coordinate frames and transforms
- **Lab 7**: Robot state publishing and joint state management

**Continue to [Lab 5: Humanoid URDF](./lab05-humanoid-urdf.md)**

---
**Previous**: [Lab 4: Actions](./lab04-actions.md) | **Next**: [Lab 5: Humanoid URDF](./lab05-humanoid-urdf.md)