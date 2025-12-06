---
title: Lab 5 - Humanoid URDF
sidebar_label: Lab 5 - Humanoid URDF
sidebar_position: 9
description: Design and implement a complete 12-DOF humanoid robot model with torso, arms, legs, head, and proper URDF hierarchy using Xacro macros and collision geometry
---

# Lab 5: Humanoid URDF

## Overview

In this lab, you'll design and implement a complete 12-DOF humanoid robot model with torso, arms, legs, and head. You'll use Xacro macros for reusable components, implement proper URDF hierarchy with parent-child relationships, and configure collision geometry for physics simulation. This comprehensive robot model will serve as the foundation for advanced ROS 2 applications.

**Duration**: 2 hours

**Learning Objectives**:
- ✅ Design 12-DOF humanoid robot with torso, 2 arms (6 DOF each), 2 legs (6 DOF each)
- ✅ Implement Xacro macros for reusable robot components (arms, legs, joints)
- ✅ Create proper URDF hierarchy with 13 links and 12 joints forming valid kinematic tree
- ✅ Configure collision geometry and visual materials for complete robot model
- ✅ Validate URDF model with `check_urdf` and visualize in RViz

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **Completed Lab 1** - Talker/Listener basics
✅ **Completed Lab 2** - Custom messages
✅ **Completed Lab 3** - Services
✅ **Completed Lab 4** - Actions
✅ **Completed URDF Fundamentals** - Basic URDF concepts
✅ **ROS 2 Humble installed** with all standard packages
✅ **Basic understanding** of Xacro macros and robot modeling
✅ **XML knowledge** for understanding URDF/Xacro structure

---

## Step 1: Understanding Humanoid Robot Design

### Humanoid Kinematic Structure

A humanoid robot mimics human body structure with:

**Torso** (1 link):
- `base_link` (pelvis/trunk)

**Arms** (2×3 DOF = 6 DOF total):
- **Left Arm**: `l_shoulder_pitch`, `l_elbow`, `l_wrist_roll`
- **Right Arm**: `r_shoulder_pitch`, `r_elbow`, `r_wrist_roll`

**Legs** (2×3 DOF = 6 DOF total):
- **Left Leg**: `l_hip_pitch`, `l_knee`, `l_ankle`
- **Right Leg**: `r_hip_pitch`, `r_knee`, `r_ankle`

**Head** (1 DOF):
- `neck_yaw`

### Degrees of Freedom (DOF)

| Body Part | Joints | DOF | Motion Type |
|-----------|--------|-----|-------------|
| Left Arm | 3 | 3 | Shoulder Pitch, Elbow Flexion, Wrist Roll |
| Right Arm | 3 | 3 | Shoulder Pitch, Elbow Flexion, Wrist Roll |
| Left Leg | 3 | 3 | Hip Pitch, Knee Flexion, Ankle Pitch |
| Right Leg | 3 | 3 | Hip Pitch, Knee Flexion, Ankle Pitch |
| Neck | 1 | 1 | Yaw Rotation |
| **Total** | **13** | **12** | **12 DOF** |

### URDF Hierarchy

```
base_link (torso)
├── l_hip_pitch
│   ├── l_knee
│   └── l_ankle
├── r_hip_pitch
│   ├── r_knee
│   └── r_ankle
├── neck_yaw
│   └── head
├── l_shoulder_pitch
│   ├── l_elbow
│   └── l_wrist_roll
└── r_shoulder_pitch
    ├── r_elbow
    └── r_wrist_roll
```

---

## Step 2: Creating the Humanoid URDF Package

First, let's create a package for our humanoid robot model:

```bash
cd ~/ros2_ws/src
rm -rf humanoid_robot_description  # Remove if exists
ros2 pkg create --build-type ament_cmake humanoid_robot_description
```

Create the necessary directories:
```bash
mkdir -p ~/ros2_ws/src/humanoid_robot_description/urdf
mkdir -p ~/ros2_ws/src/humanoid_robot_description/urdf/macros
mkdir -p ~/ros2_ws/src/humanoid_robot_description/meshes
mkdir -p ~/ros2_ws/src/humanoid_robot_description/meshes/visual
mkdir -p ~/ros2_ws/src/humanoid_robot_description/meshes/collision
mkdir -p ~/ros2_ws/src/humanoid_robot_description/config
mkdir -p ~/ros2_ws/src/humanoid_robot_description/launch
```

---

## Step 3: Creating Xacro Macros for Reusable Components

### Common Functions Macro

Create the common functions macro:
```bash
touch ~/ros2_ws/src/humanoid_robot_description/urdf/macros/common_functions.xacro
```

Add the following content to `~/ros2_ws/src/humanoid_robot_description/urdf/macros/common_functions.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Inertia macros for common shapes -->

  <!-- Cylinder inertia (mass m, radius r, height h) -->
  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertia
      ixx="${m*(3*r*r+h*h)/12}"
      ixy="0"
      ixz="0"
      iyy="${m*(3*r*r+h*h)/12}"
      iyz="0"
      izz="${m*r*r/2}"/>
  </xacro:macro>

  <!-- Box inertia (mass m, width w, height h, depth d) -->
  <xacro:macro name="box_inertia" params="m w h d">
    <inertia
      ixx="${m*(h*h+d*d)/12}"
      ixy="0"
      ixz="0"
      iyy="${m*(w*w+d*d)/12}"
      iyz="0"
      izz="${m*(w*w+h*h)/12}"/>
  </xacro:macro>

  <!-- Sphere inertia (mass m, radius r) -->
  <xacro:macro name="sphere_inertia" params="m r">
    <inertia
      ixx="${2*m*r*r/5}"
      ixy="0"
      ixz="0"
      iyy="${2*m*r*r/5}"
      iyz="0"
      izz="${2*m*r*r/5}"/>
  </xacro:macro>

  <!-- Generic link macro with visual, collision, and inertial properties -->
  <xacro:macro name="generic_link" params="name mass visual_mesh collision_mesh parent_xyz parent_rpy child_xyz child_rpy material">
    <link name="${name}">
      <visual>
        <origin xyz="${child_xyz}" rpy="${child_rpy}"/>
        <geometry>
          <mesh filename="${visual_mesh}"/>
        </geometry>
        <material name="${material}"/>
      </visual>

      <collision>
        <origin xyz="${child_xyz}" rpy="${child_rpy}"/>
        <geometry>
          <mesh filename="${collision_mesh}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="${mass}"/>
        <origin xyz="${child_xyz}" rpy="${child_rpy}"/>
        <box_inertia m="${mass}" w="0.1" h="0.1" d="0.1"/> <!-- Approximate inertia -->
      </inertial>
    </link>
  </xacro:macro>

</robot>
```

### Materials Macro

Create the materials macro:
```bash
touch ~/ros2_ws/src/humanoid_robot_description/urdf/macros/materials.xacro
```

Add the following content to `~/ros2_ws/src/humanoid_robot_description/urdf/macros/materials.xacro`:

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

  <material name="DarkGrey">
    <color rgba="0.3 0.3 0.3 1.0"/>
  </material>

  <material name="LightGrey">
    <color rgba="0.8 0.8 0.8 1.0"/>
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

  <material name="Yellow">
    <color rgba="0.8 0.8 0.0 1.0"/>
  </material>

  <material name="TransparentGrey">
    <color rgba="0.5 0.5 0.5 0.5"/>
  </material>

</robot>
```

### Limb Macros

Create the limb macros file:
```bash
touch ~/ros2_ws/src/humanoid_robot_description/urdf/macros/limbs.xacro
```

Add the following content to `~/ros2_ws/src/humanoid_robot_description/urdf/macros/limbs.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro for a simple arm segment -->
  <xacro:macro name="arm_segment" params="name parent_link joint_origin_xyz joint_axis joint_type min_angle max_angle effort vel radius length mass material">
    <!-- Joint -->
    <joint name="${name}_joint" type="${joint_type}">
      <parent link="${parent_link}"/>
      <child link="${name}_link"/>
      <origin xyz="${joint_origin_xyz}" rpy="0 0 0"/>
      <axis xyz="${joint_axis}"/>
      <xacro:if value="${joint_type != 'fixed'}">
        <limit lower="${min_angle}" upper="${max_angle}" effort="${effort}" velocity="${vel}"/>
      </xacro:if>
    </joint>

    <!-- Link -->
    <link name="${name}_link">
      <visual>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="${material}"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <cylinder_inertia m="${mass}" r="${radius}" h="${length}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for a simple leg segment -->
  <xacro:macro name="leg_segment" params="name parent_link joint_origin_xyz joint_axis joint_type min_angle max_angle effort vel radius length mass material">
    <!-- Joint -->
    <joint name="${name}_joint" type="${joint_type}">
      <parent link="${parent_link}"/>
      <child link="${name}_link"/>
      <origin xyz="${joint_origin_xyz}" rpy="0 0 0"/>
      <axis xyz="${joint_axis}"/>
      <xacro:if value="${joint_type != 'fixed'}">
        <limit lower="${min_angle}" upper="${max_angle}" effort="${effort}" velocity="${vel}"/>
      </xacro:if>
    </joint>

    <!-- Link -->
    <link name="${name}_link">
      <visual>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="${material}"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <cylinder_inertia m="${mass}" r="${radius}" h="${length}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for a complete arm (shoulder + elbow + wrist) -->
  <xacro:macro name="complete_arm" params="side parent_link shoulder_pos shoulder_axis shoulder_min shoulder_max elbow_min elbow_max wrist_min wrist_max">
    <!-- Shoulder Pitch Joint -->
    <joint name="${side}_shoulder_pitch" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="${shoulder_pos}" rpy="0 0 0"/>
      <axis xyz="${shoulder_axis}"/>
      <limit lower="${shoulder_min}" upper="${shoulder_max}" effort="100" velocity="2.0"/>
    </joint>

    <!-- Upper Arm Link -->
    <link name="${side}_upper_arm">
      <visual>
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
        <material name="Blue"/>
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

    <!-- Elbow Joint -->
    <joint name="${side}_elbow" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_forearm"/>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${elbow_min}" upper="${elbow_max}" effort="100" velocity="2.0"/>
    </joint>

    <!-- Forearm Link -->
    <link name="${side}_forearm">
      <visual>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="0.2"/>
        </geometry>
        <material name="Green"/>
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

    <!-- Wrist Roll Joint -->
    <joint name="${side}_wrist_roll" type="continuous">
      <parent link="${side}_forearm"/>
      <child link="${side}_hand"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit effort="50" velocity="3.0"/>
    </joint>

    <!-- Hand Link -->
    <link name="${side}_hand">
      <visual>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.03" length="0.1"/>
        </geometry>
        <material name="Yellow"/>
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
        <cylinder_inertia m="0.2" r="0.03" h="0.1"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Macro for a complete leg (hip + knee + ankle) -->
  <xacro:macro name="complete_leg" params="side parent_link hip_pos hip_axis hip_min hip_max knee_min knee_max ankle_min ankle_max">
    <!-- Hip Pitch Joint -->
    <joint name="${side}_hip_pitch" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${side}_thigh"/>
      <origin xyz="${hip_pos}" rpy="0 0 0"/>
      <axis xyz="${hip_axis}"/>
      <limit lower="${hip_min}" upper="${hip_max}" effort="200" velocity="1.5"/>
    </joint>

    <!-- Thigh Link -->
    <link name="${side}_thigh">
      <visual>
        <origin xyz="0 0 -0.2" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.06" length="0.4"/>
        </geometry>
        <material name="Red"/>
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

    <!-- Knee Joint -->
    <joint name="${side}_knee" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 -0.4" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${knee_min}" upper="${knee_max}" effort="200" velocity="1.5"/>
    </joint>

    <!-- Shin Link -->
    <link name="${side}_shin">
      <visual>
        <origin xyz="0 0 -0.15" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="0.3"/>
        </geometry>
        <material name="Orange"/>
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

    <!-- Ankle Joint -->
    <joint name="${side}_ankle" type="revolute">
      <parent link="${side}_shin"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 -0.3" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="${ankle_min}" upper="${ankle_max}" effort="100" velocity="1.0"/>
    </joint>

    <!-- Foot Link -->
    <link name="${side}_foot">
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
        <material name="White"/>
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
        <box_inertia m="0.8" w="0.15" h="0.05" d="0.08"/>
      </inertial>
    </link>
  </xacro:macro>

</robot>
```

---

## Step 4: Creating the Complete Humanoid Robot URDF

Create the main humanoid robot URDF file:
```bash
touch ~/ros2_ws/src/humanoid_robot_description/urdf/humanoid_robot.xacro
```

Add the following content to `~/ros2_ws/src/humanoid_robot_description/urdf/humanoid_robot.xacro`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include common macros -->
  <xacro:include filename="$(find humanoid_robot_description)/urdf/macros/common_functions.xacro"/>
  <xacro:include filename="$(find humanoid_robot_description)/urdf/macros/materials.xacro"/>
  <xacro:include filename="$(find humanoid_robot_description)/urdf/macros/limbs.xacro"/>

  <!-- Robot parameters -->
  <xacro:property name="torso_width" value="0.3"/>
  <xacro:property name="torso_depth" value="0.2"/>
  <xacro:property name="torso_height" value="0.6"/>
  <xacro:property name="torso_mass" value="10.0"/>

  <xacro:property name="head_width" value="0.2"/>
  <xacro:property name="head_depth" value="0.15"/>
  <xacro:property name="head_height" value="0.2"/>
  <xacro:property name="head_mass" value="2.0"/>

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="Grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${torso_mass}"/>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <box_inertia m="${torso_mass}" w="${torso_width}" h="${torso_height}" d="${torso_depth}"/>
    </inertial>
  </link>

  <!-- Neck Yaw Joint -->
  <joint name="neck_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <origin xyz="0 0 ${head_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${head_width} ${head_depth} ${head_height}"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${head_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${head_width} ${head_depth} ${head_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${head_mass}"/>
      <origin xyz="0 0 ${head_height/2}" rpy="0 0 0"/>
      <box_inertia m="${head_mass}" w="${head_width}" h="${head_height}" d="${head_depth}"/>
    </inertial>
  </link>

  <!-- Left Shoulder Position -->
  <joint name="left_shoulder_mount" type="fixed">
    <parent link="base_link"/>
    <child link="l_shoulder_link"/>
    <origin xyz="${torso_width/2} 0 ${torso_height*0.7}" rpy="0 0 0"/>
  </joint>

  <!-- Left Shoulder Link (for mounting the arm) -->
  <link name="l_shoulder_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <cylinder_inertia m="0.1" r="0.02" h="0.05"/>
    </inertial>
  </link>

  <!-- Complete left arm using the macro -->
  <xacro:complete_arm
    side="l"
    parent_link="l_shoulder_link"
    shoulder_pos="0 0 0"
    shoulder_axis="1 0 0"
    shoulder_min="-1.57" shoulder_max="1.57"
    elbow_min="0" elbow_max="3.14"
    wrist_min="-3.14" wrist_max="3.14"
  />

  <!-- Right Shoulder Position -->
  <joint name="right_shoulder_mount" type="fixed">
    <parent link="base_link"/>
    <child link="r_shoulder_link"/>
    <origin xyz="${-torso_width/2} 0 ${torso_height*0.7}" rpy="0 0 0"/>
  </joint>

  <!-- Right Shoulder Link (for mounting the arm) -->
  <link name="r_shoulder_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <cylinder_inertia m="0.1" r="0.02" h="0.05"/>
    </inertial>
  </link>

  <!-- Complete right arm using the macro -->
  <xacro:complete_arm
    side="r"
    parent_link="r_shoulder_link"
    shoulder_pos="0 0 0"
    shoulder_axis="1 0 0"
    shoulder_min="-1.57" shoulder_max="1.57"
    elbow_min="0" elbow_max="3.14"
    wrist_min="-3.14" wrist_max="3.14"
  />

  <!-- Left Hip Position -->
  <joint name="left_hip_mount" type="fixed">
    <parent link="base_link"/>
    <child link="l_hip_link"/>
    <origin xyz="${torso_width/4} 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Left Hip Link (for mounting the leg) -->
  <link name="l_hip_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <cylinder_inertia m="0.1" r="0.02" h="0.05"/>
    </inertial>
  </link>

  <!-- Complete left leg using the macro -->
  <xacro:complete_leg
    side="l"
    parent_link="l_hip_link"
    hip_pos="0 0 0"
    hip_axis="1 0 0"
    hip_min="-1.57" hip_max="1.57"
    knee_min="0" knee_max="2.35"
    ankle_min="-1.57" ankle_max="1.57"
  />

  <!-- Right Hip Position -->
  <joint name="right_hip_mount" type="fixed">
    <parent link="base_link"/>
    <child link="r_hip_link"/>
    <origin xyz="${-torso_width/4} 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Right Hip Link (for mounting the leg) -->
  <link name="r_hip_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <cylinder_inertia m="0.1" r="0.02" h="0.05"/>
    </inertial>
  </link>

  <!-- Complete right leg using the macro -->
  <xacro:complete_leg
    side="r"
    parent_link="r_hip_link"
    hip_pos="0 0 0"
    hip_axis="1 0 0"
    hip_min="-1.57" hip_max="1.57"
    knee_min="0" knee_max="2.35"
    ankle_min="-1.57" ankle_max="1.57"
  />

  <!-- Add a simple camera to the head for perception -->
  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 ${head_height*0.7}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="Black"/>
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
      <box_inertia m="0.1" w="0.05" h="0.03" d="0.05"/>
    </inertial>
  </link>

  <!-- Add IMU sensor to the torso -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${torso_height*0.5}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box_inertia m="0.01" w="0.01" h="0.01" d="0.01"/>
    </inertial>
  </link>

</robot>
```

---

## Step 5: Creating a Simplified Version for Learning

Let's also create a simplified version that focuses on the essential 12-DOF structure:

Create the simplified humanoid URDF:
```bash
touch ~/ros2_ws/src/humanoid_robot_description/urdf/humanoid_simple.xacro
```

Add the following content to `~/ros2_ws/src/humanoid_robot_description/urdf/humanoid_simple.xacro`:

```xml
<?xml version="1.0"?>
<robot name="humanoid_simple" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include common macros -->
  <xacro:include filename="$(find humanoid_robot_description)/urdf/macros/common_functions.xacro"/>
  <xacro:include filename="$(find humanoid_robot_description)/urdf/macros/materials.xacro"/>

  <!-- Robot parameters -->
  <xacro:property name="torso_width" value="0.3"/>
  <xacro:property name="torso_depth" value="0.2"/>
  <xacro:property name="torso_height" value="0.6"/>
  <xacro:property name="torso_mass" value="10.0"/>

  <xacro:property name="head_size" value="0.2"/>
  <xacro:property name="head_mass" value="2.0"/>

  <xacro:property name="upper_arm_length" value="0.3"/>
  <xacro:property name="upper_arm_radius" value="0.05"/>
  <xacro:property name="upper_arm_mass" value="1.0"/>

  <xacro:property name="forearm_length" value="0.2"/>
  <xacro:property name="forearm_radius" value="0.04"/>
  <xacro:property name="forearm_mass" value="0.5"/>

  <xacro:property name="hand_length" value="0.1"/>
  <xacro:property name="hand_radius" value="0.03"/>
  <xacro:property name="hand_mass" value="0.2"/>

  <xacro:property name="thigh_length" value="0.4"/>
  <xacro:property name="thigh_radius" value="0.06"/>
  <xacro:property name="thigh_mass" value="2.0"/>

  <xacro:property name="shin_length" value="0.3"/>
  <xacro:property name="shin_radius" value="0.05"/>
  <xacro:property name="shin_mass" value="1.5"/>

  <xacro:property name="foot_size" value="0.15 0.08 0.05"/>
  <xacro:property name="foot_mass" value="0.8"/>

  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="Grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${torso_mass}"/>
      <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0"/>
      <box_inertia m="${torso_mass}" w="${torso_width}" h="${torso_height}" d="${torso_depth}"/>
    </inertial>
  </link>

  <!-- HEAD (1 DOF: neck yaw) -->
  <!-- Neck Yaw Joint -->
  <joint name="neck_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_height}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <origin xyz="0 0 ${head_size/2}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${head_size/2}"/>
      </geometry>
      <material name="LightGrey"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${head_size/2}" rpy="0 0 0"/>
      <geometry>
        <sphere radius="${head_size/2}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${head_mass}"/>
      <origin xyz="0 0 ${head_size/2}" rpy="0 0 0"/>
      <sphere_inertia m="${head_mass}" r="${head_size/2}"/>
    </inertial>
  </link>

  <!-- LEFT ARM (3 DOF: shoulder pitch, elbow flexion, wrist roll) -->
  <!-- Left Shoulder Pitch Joint -->
  <joint name="l_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="l_upper_arm"/>
    <origin xyz="${torso_width/2} 0 ${torso_height*0.7}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <!-- Left Upper Arm Link -->
  <link name="l_upper_arm">
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
      <mass value="${upper_arm_mass}"/>
      <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${upper_arm_mass}" r="${upper_arm_radius}" h="${upper_arm_length}"/>
    </inertial>
  </link>

  <!-- Left Elbow Joint -->
  <joint name="l_elbow" type="revolute">
    <parent link="l_upper_arm"/>
    <child link="l_forearm"/>
    <origin xyz="0 0 ${upper_arm_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="2.0"/>
  </joint>

  <!-- Left Forearm Link -->
  <link name="l_forearm">
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
      <mass value="${forearm_mass}"/>
      <origin xyz="0 0 ${forearm_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${forearm_mass}" r="${forearm_radius}" h="${forearm_length}"/>
    </inertial>
  </link>

  <!-- Left Wrist Roll Joint -->
  <joint name="l_wrist_roll" type="continuous">
    <parent link="l_forearm"/>
    <child link="l_hand"/>
    <origin xyz="0 0 ${forearm_length}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" velocity="3.0"/>
  </joint>

  <!-- Left Hand Link -->
  <link name="l_hand">
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
      <mass value="${hand_mass}"/>
      <origin xyz="0 0 ${hand_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${hand_mass}" r="${hand_radius}" h="${hand_length}"/>
    </inertial>
  </link>

  <!-- RIGHT ARM (3 DOF: shoulder pitch, elbow flexion, wrist roll) -->
  <!-- Right Shoulder Pitch Joint -->
  <joint name="r_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="r_upper_arm"/>
    <origin xyz="${-torso_width/2} 0 ${torso_height*0.7}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
  </joint>

  <!-- Right Upper Arm Link -->
  <link name="r_upper_arm">
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
      <mass value="${upper_arm_mass}"/>
      <origin xyz="0 0 ${upper_arm_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${upper_arm_mass}" r="${upper_arm_radius}" h="${upper_arm_length}"/>
    </inertial>
  </link>

  <!-- Right Elbow Joint -->
  <joint name="r_elbow" type="revolute">
    <parent link="r_upper_arm"/>
    <child link="r_forearm"/>
    <origin xyz="0 0 ${upper_arm_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="2.0"/>
  </joint>

  <!-- Right Forearm Link -->
  <link name="r_forearm">
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
      <mass value="${forearm_mass}"/>
      <origin xyz="0 0 ${forearm_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${forearm_mass}" r="${forearm_radius}" h="${forearm_length}"/>
    </inertial>
  </link>

  <!-- Right Wrist Roll Joint -->
  <joint name="r_wrist_roll" type="continuous">
    <parent link="r_forearm"/>
    <child link="r_hand"/>
    <origin xyz="0 0 ${forearm_length}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="50" velocity="3.0"/>
  </joint>

  <!-- Right Hand Link -->
  <link name="r_hand">
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
      <mass value="${hand_mass}"/>
      <origin xyz="0 0 ${hand_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${hand_mass}" r="${hand_radius}" h="${hand_length}"/>
    </inertial>
  </link>

  <!-- LEFT LEG (3 DOF: hip pitch, knee flexion, ankle pitch) -->
  <!-- Left Hip Pitch Joint -->
  <joint name="l_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="l_thigh"/>
    <origin xyz="${torso_width/4} 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1.5"/>
  </joint>

  <!-- Left Thigh Link -->
  <link name="l_thigh">
    <visual>
      <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${thigh_radius}" length="${thigh_length}"/>
      </geometry>
      <material name="Red"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${thigh_radius}" length="${thigh_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${thigh_mass}"/>
      <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${thigh_mass}" r="${thigh_radius}" h="${thigh_length}"/>
    </inertial>
  </link>

  <!-- Left Knee Joint -->
  <joint name="l_knee" type="revolute">
    <parent link="l_thigh"/>
    <child link="l_shin"/>
    <origin xyz="0 0 ${-thigh_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="200" velocity="1.5"/>
  </joint>

  <!-- Left Shin Link -->
  <link name="l_shin">
    <visual>
      <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${shin_mass}"/>
      <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${shin_mass}" r="${shin_radius}" h="${shin_length}"/>
    </inertial>
  </link>

  <!-- Left Ankle Joint -->
  <joint name="l_ankle" type="revolute">
    <parent link="l_shin"/>
    <child link="l_foot"/>
    <origin xyz="0 0 ${-shin_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Left Foot Link -->
  <link name="l_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${foot_size}"/>
      </geometry>
      <material name="White"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${foot_size}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${foot_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box_inertia m="${foot_mass}" w="${foot_size[0]}" h="${foot_size[2]}" d="${foot_size[1]}"/>
    </inertial>
  </link>

  <!-- RIGHT LEG (3 DOF: hip pitch, knee flexion, ankle pitch) -->
  <!-- Right Hip Pitch Joint -->
  <joint name="r_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="r_thigh"/>
    <origin xyz="${-torso_width/4} 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1.5"/>
  </joint>

  <!-- Right Thigh Link -->
  <link name="r_thigh">
    <visual>
      <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${thigh_radius}" length="${thigh_length}"/>
      </geometry>
      <material name="Red"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${thigh_radius}" length="${thigh_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${thigh_mass}"/>
      <origin xyz="0 0 ${-thigh_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${thigh_mass}" r="${thigh_radius}" h="${thigh_length}"/>
    </inertial>
  </link>

  <!-- Right Knee Joint -->
  <joint name="r_knee" type="revolute">
    <parent link="r_thigh"/>
    <child link="r_shin"/>
    <origin xyz="0 0 ${-thigh_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="200" velocity="1.5"/>
  </joint>

  <!-- Right Shin Link -->
  <link name="r_shin">
    <visual>
      <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
      <material name="Orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${shin_radius}" length="${shin_length}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${shin_mass}"/>
      <origin xyz="0 0 ${-shin_length/2}" rpy="0 0 0"/>
      <cylinder_inertia m="${shin_mass}" r="${shin_radius}" h="${shin_length}"/>
    </inertial>
  </link>

  <!-- Right Ankle Joint -->
  <joint name="r_ankle" type="revolute">
    <parent link="r_shin"/>
    <child link="r_foot"/>
    <origin xyz="0 0 ${-shin_length}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Right Foot Link -->
  <link name="r_foot">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${foot_size}"/>
      </geometry>
      <material name="White"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${foot_size}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${foot_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <box_inertia m="${foot_mass}" w="${foot_size[0]}" h="${foot_size[2]}" d="${foot_size[1]}"/>
    </inertial>
  </link>

</robot>
```

---

## Step 6: Update Package Configuration

Update the CMakeLists.txt for the humanoid robot description package:

Edit `~/ros2_ws/src/humanoid_robot_description/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(humanoid_robot_description)

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
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

Update the package.xml:

Edit `~/ros2_ws/src/humanoid_robot_description/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_robot_description</name>
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

## Step 7: Building the Humanoid Robot Package

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_robot_description
source ~/ros2_ws/install/setup.bash
```

---

## Step 8: Validating the Humanoid URDF Model

### Using check_urdf

Validate the complete humanoid model:

```bash
# Convert Xacro to URDF temporarily for validation
xacro $(ros2 pkg prefix humanoid_robot_description)/share/humanoid_robot_description/urdf/humanoid_simple.xacro > /tmp/humanoid_robot.urdf

# Check if the URDF is syntactically correct
check_urdf /tmp/humanoid_robot.urdf

# Clean up
rm /tmp/humanoid_robot.urdf
```

**Expected Output**:
```
robot name is: humanoid_simple
---------- Successfully Parsed XML ---------------
root Link: base_link has 13 child(ren)
    child(1):  head
        ...
    child(2):  l_upper_arm
        ...
    child(3):  r_upper_arm
        ...
    child(4):  l_thigh
        ...
    child(5):  r_thigh
        ...
```

### Counting Links and Joints

Verify the model has the correct number of links and joints:

```bash
# Count links
ros2 run xacro xacro $(ros2 pkg prefix humanoid_robot_description)/share/humanoid_robot_description/urdf/humanoid_simple.xacro | grep -c "<link name="
# Expected: 13 links (base_link, head, 6 arm links, 6 leg links)

# Count joints
ros2 run xacro xacro $(ros2 pkg prefix humanoid_robot_description)/share/humanoid_robot_description/urdf/humanoid_simple.xacro | grep -c "<joint name="
# Expected: 12 joints (1 neck, 6 arm joints, 6 leg joints)
```

---

## Step 9: Visualizing the Humanoid Robot

### Creating a Launch File for Visualization

Create a launch file to visualize the robot:

```bash
touch ~/ros2_ws/src/humanoid_robot_description/launch/view_humanoid.launch.py
```

Add the following content to `~/ros2_ws/src/humanoid_robot_description/launch/view_humanoid.launch.py`:

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
        get_package_share_directory('humanoid_robot_description'),
        'urdf',
        'humanoid_simple.xacro'
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

    # Joint state publisher GUI node
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        namespace=LaunchConfiguration('namespace')
    )

    # RViz2 node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('humanoid_robot_description'), 'config', 'view_robot.rviz')],
        namespace=LaunchConfiguration('namespace')
    )

    return LaunchDescription([
        namespace_arg,
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz
    ])
```

### Creating an RViz Configuration

Create the RViz configuration file:

```bash
mkdir -p ~/ros2_ws/src/humanoid_robot_description/config
touch ~/ros2_ws/src/humanoid_robot_description/config/view_robot.rviz
```

Add the following content to `~/ros2_ws/src/humanoid_robot_description/config/view_robot.rviz`:

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
      Splitter Ratio: 0.5
    Tree Height: 617
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
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
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        head:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        l_foot:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        l_hand:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        l_shin:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        l_thigh:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        l_upper_arm:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        l_wrist_roll:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        neck_yaw:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        r_ankle:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        r_elbow:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        r_foot:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        r_forearm:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        r_hand:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        r_hip_pitch:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        r_knee:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        r_shin:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        r_shoulder_pitch:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        r_thigh:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        r_upper_arm:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        r_wrist_roll:
          Alpha: 1
          Show Axes: false
          Show Trail: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
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
      Distance: 2.5
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
      Pitch: 0.5
      Target Frame: base_link
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002f4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002f4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002f4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002f4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004180000024400fffffffb0000000800540069006d006501000000000000045000000000000000000000023b000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 72
  Y: 60
```

---

## Step 10: Launching and Testing the Humanoid Robot

Launch the visualization to see your humanoid robot:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch humanoid_robot_description view_humanoid.launch.py
```

This will open:
1. **RViz2** showing your humanoid robot model
2. **Joint State Publisher GUI** to manually control the joints
3. **Robot State Publisher** to publish the robot state

You should see a 12-DOF humanoid robot with:
- 1 link torso (base_link)
- 1 DOF head (neck_yaw joint)
- 6 DOF arms (3 per arm: shoulder, elbow, wrist)
- 6 DOF legs (3 per leg: hip, knee, ankle)

---

## Step 11: Advanced URDF Techniques for Humanoid Robots

### Transmission Elements

For real robot control, you need transmission elements:

Create `~/ros2_ws/src/humanoid_robot_description/urdf/transmissions.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro for simple transmission -->
  <xacro:macro name="simple_transmission" params="joint_name">
    <transmission name="${joint_name}_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${joint_name}">
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      </joint>
      <actuator name="${joint_name}_motor">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
  </xacro:macro>

  <!-- Example: Add transmissions to all joints -->
  <xacro:simple_transmission joint_name="neck_yaw"/>
  <xacro:simple_transmission joint_name="l_shoulder_pitch"/>
  <xacro:simple_transmission joint_name="l_elbow"/>
  <xacro:simple_transmission joint_name="l_wrist_roll"/>
  <xacro:simple_transmission joint_name="r_shoulder_pitch"/>
  <xacro:simple_transmission joint_name="r_elbow"/>
  <xacro:simple_transmission joint_name="r_wrist_roll"/>
  <xacro:simple_transmission joint_name="l_hip_pitch"/>
  <xacro:simple_transmission joint_name="l_knee"/>
  <xacro:simple_transmission joint_name="l_ankle"/>
  <xacro:simple_transmission joint_name="r_hip_pitch"/>
  <xacro:simple_transmission joint_name="r_knee"/>
  <xacro:simple_transmission joint_name="r_ankle"/>

</robot>
```

### Gazebo Integration

For simulation in Gazebo, add Gazebo-specific elements:

Create `~/ros2_ws/src/humanoid_robot_description/urdf/gazebo.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Gazebo material definitions -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="l_upper_arm">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="l_forearm">
    <material>Gazebo/Green</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="l_hand">
    <material>Gazebo/Yellow</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="r_upper_arm">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="r_forearm">
    <material>Gazebo/Green</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="r_hand">
    <material>Gazebo/Yellow</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="l_thigh">
    <material>Gazebo/Red</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="l_shin">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="l_foot">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <gazebo reference="r_thigh">
    <material>Gazebo/Red</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="r_shin">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="r_foot">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
  </gazebo>

  <!-- Gazebo plugin for ros_control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
    </plugin>
  </gazebo>

</robot>
```

---

## Step 12: Humanoid Robot Best Practices

### Design Principles

1. **Kinematic Chain**: Ensure each chain (arm, leg) has proper parent-child relationships
2. **Mass Distribution**: Place heavier components near the center of mass
3. **Joint Limits**: Set realistic limits based on human anatomy
4. **Collision Geometry**: Use simplified shapes for performance

### Performance Considerations

- **Meshes**: Use lightweight meshes for visualization
- **Inertias**: Calculate accurate inertias for stable simulation
- **Joints**: Limit degrees of freedom to necessary movements
- **Sensors**: Add only necessary sensors for your application

### Modularity

- **Xacro Macros**: Create reusable components
- **Separate Files**: Organize by functionality (materials, limbs, etc.)
- **Parameters**: Use properties for easy customization

---

## Step 13: Troubleshooting Common Humanoid URDF Issues

### Issue 1: Invalid Kinematic Tree

**Symptoms**: `check_urdf` shows multiple roots or disconnected links

**Solutions**:
```bash
# Check the kinematic tree structure
check_urdf $(ros2 run xacro xacro $(ros2 pkg prefix humanoid_robot_description)/share/humanoid_robot_description/urdf/humanoid_simple.xacro)
```

### Issue 2: Joint Direction Problems

**Symptoms**: Arms/legs moving in wrong directions

**Solutions**:
- Check joint axis definitions (xyz values)
- Verify joint origin placements
- Ensure consistent coordinate frame conventions

### Issue 3: Physics Instability

**Symptoms**: Robot parts oscillating or behaving unexpectedly in simulation

**Solutions**:
- Check that all masses are positive and reasonable
- Verify inertia tensors are physically plausible
- Ensure joint limits are appropriate
- Check that collision geometry is properly defined

### Issue 4: Visualization Issues

**Symptoms**: Parts not showing up or appearing in wrong positions

**Solutions**:
- Verify origin transformations for visual/collision elements
- Check that materials are properly defined
- Ensure mesh files exist and are properly referenced

---

## Lab Summary

In this lab, you've successfully:

✅ **Designed a 12-DOF humanoid robot** with torso, arms, legs, and head
✅ **Implemented Xacro macros** for reusable robot components
✅ **Created proper URDF hierarchy** with 13 links and 12 joints
✅ **Configured collision geometry** and visual materials for complete model
✅ **Validated URDF model** with `check_urdf` and visualized in RViz

### Key Takeaways

- **Humanoid robots** require careful kinematic design with proper DOF distribution
- **Xacro macros** enable modular, reusable robot component definitions
- **URDF hierarchy** must form a valid kinematic tree with single root
- **Collision geometry** should be simplified for simulation performance
- **Validation** is essential before using robot models in applications

---

## Next Steps

Now that you understand humanoid URDF modeling, you're ready to explore:

- **Lab 6**: TF2 coordinate frames and transforms
- **Lab 7**: Robot state publishing and joint state management
- **Lab 8**: RViz integration for visualization

**Continue to [Lab 6: TF2 Coordinate Frames](./lab06-tf2-broadcasting.md)**

---
**Previous**: [URDF Fundamentals](./urdf-fundamentals.md) | **Next**: [Lab 6: TF2 Coordinate Frames](./lab06-tf2-broadcasting.md)