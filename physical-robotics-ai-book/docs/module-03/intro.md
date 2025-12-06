# Introduction
## Overview

Module 03 focuses on creating realistic digital twins of humanoid robots using Gazebo Harmonic simulation environment. This module teaches students how to build sensor-equipped humanoid robots, configure realistic apartment environments, and optionally connect Unity for real-time visualization. The goal is to provide hands-on experience with industry-standard simulation tools that bridge the gap between digital AI and physical robotics.

This module builds upon the ROS 2 foundation established in Module 01, applying those concepts to embodied robotics simulation. Students will learn to create complete simulation environments that mirror real-world robotics applications, preparing them for advanced physical AI development.

## Prerequisites

Before starting this module, students must have:

- **Module 01 Completion**: Successfully completed Module 01 (Introduction & Physical AI Foundations) with a working Ubuntu 24.04 development environment
- **ROS 2 Proficiency**: Understanding of ROS 2 concepts (nodes, topics, services) and ability to create basic publisher/subscriber nodes
- **Python Programming**: Intermediate Python skills including object-oriented programming and basic file handling
- **Linux Command Line**: Comfortable with bash commands, file system navigation, and package management
- **Basic Physics Understanding**: Familiarity with concepts like coordinate systems, kinematics, and basic mechanics

## Learning Goals

Upon completing this module, students will be able to:

1. **Install and Configure Gazebo Harmonic**: Set up the latest Gazebo simulation environment with optimal performance settings for humanoid robotics applications.

2. **Create Sensor-Equipped Humanoid Robots**: Design and implement humanoid robot models with four key sensor types (depth camera, LiDAR, IMU, contact sensors) using URDF and SDF formats.

3. **Build Realistic Apartment Environments**: Construct detailed simulation worlds with proper physics properties, lighting, and asset optimization for high-performance simulation.

4. **Connect Unity for Visualization**: Optionally integrate Unity 3D for real-time visualization and human-robot interaction, bridging Gazebo simulation with Unity's rendering capabilities.

## Hardware Requirements

To achieve optimal performance and meet the success criteria of this module, the following hardware specifications are required:

- **Graphics Card**: NVIDIA RTX 4070 Ti, 4080, or 4090 (laptop or desktop)
  - Minimum: RTX 3060 for basic functionality (with reduced performance)
  - VRAM: 12GB+ recommended for complex scenes with multiple sensors
  - CUDA Compute Capability: 7.5 or higher for GPU-accelerated sensors

- **System Memory**: 16GB RAM minimum, 32GB recommended
  - For complex apartment worlds with detailed meshes and multiple sensors
  - Gazebo Harmonic memory usage scales with scene complexity

- **CPU**: Multi-core processor (Intel i7-12700K or AMD Ryzen 7 5800X equivalent or better)
  - Required for real-time physics simulation at 1kHz update rates
  - Multi-threading support for sensor processing and ROS 2 communication

- **Storage**: 50GB+ free SSD space
  - Gazebo Harmonic installation: ~15GB
  - Simulation assets and models: ~10GB
  - ROS 2 workspace and build files: ~10GB
  - Unity installation (optional): ~15GB

## Success Criteria

Students will have successfully completed this module when they achieve:

- **Simulation Performance**: Achieve ≥60 FPS in Gazebo with humanoid robot and apartment world loaded, maintaining real-time factor (RTF) ≥1.0
- **Launch Time**: Successfully launch complete simulation environment (robot + world + sensors) within 10 minutes from clean start
- **Sensor Validation**: All four sensor types publish data at expected rates: depth camera (30Hz), LiDAR (10Hz), IMU (100Hz), contact sensors (100Hz)
- **Unity Connection**: Optional Unity visualization connects to simulation in < 5 seconds with < 30ms latency for depth camera data
- **Documentation Completion**: Complete all four chapters (intro.md, 01-gazebo-fundamentals.md, 02-sensors-and-humanoid.md, 03-apartment-worlds-and-unity.md) totaling 40-50 formatted pages

## Module Structure

This module is organized into four comprehensive chapters:

- **Chapter 1: Gazebo Fundamentals** - Installation, URDF workflow, plugins, and launch files
- **Chapter 2: Sensors and Humanoid Integration** - Humanoid robot design with sensor configuration
- **Chapter 3: Apartment Worlds and Unity Integration** - World building and visualization
- **Module Conclusion** - Integration, validation, and next steps

## Getting Started

Begin with Chapter 1 to establish your Gazebo Harmonic environment, then progress sequentially through the chapters. Each chapter builds upon the previous, creating a complete simulation system by the end of the module.