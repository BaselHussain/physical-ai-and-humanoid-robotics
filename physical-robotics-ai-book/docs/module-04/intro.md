# Introduction
Welcome to Module 04, where you'll explore the cutting-edge NVIDIA Isaac ecosystem for building intelligent robotic systems. This module focuses on the AI "brain" of modern robots, using NVIDIA's Isaac Sim 2025 and Isaac ROS for GPU-accelerated perception, Visual Simultaneous Localization and Mapping (VSLAM), navigation, and synthetic data generation.

## Learning Goals

By the end of this module, you will be able to:

- Launch NVIDIA Isaac Sim 2025 with humanoid robots at ≥60 FPS performance
- Deploy Isaac ROS perception pipelines including cuVSLAM, DetectNet, and PeopleSegNet on Jetson Orin Nano
- Build 3D maps with VSLAM in under 30 seconds using RealSense D435i camera input
- Configure Nav2 for bipedal robot navigation
- Generate 5,000+ labeled synthetic images in under 30 minutes for training perception models
- Create Docker-based deployment configurations for both workstation and Jetson platforms
- Bridge the gap between simulation and real-world robotics (sim-to-real transfer)

## Prerequisites

Before starting this module, you should have:

- Completed Module 03 (Gazebo simulation) or equivalent experience
- Ubuntu 22.04 LTS installed on your workstation
- NVIDIA RTX 4070 Ti or equivalent GPU (for Isaac Sim ≥60 FPS)
- NVIDIA Jetson Orin Nano development kit
- Intel RealSense D435i camera
- ROS 2 Humble Hawksbill installed and configured
- Basic understanding of ROS 2 concepts (topics, services, actions)
- Familiarity with Docker containers and containerization

## Hardware Requirements

- **Workstation**: Ubuntu 22.04 LTS with RTX 4070 Ti or better GPU
- **Robot Platform**: NVIDIA Jetson Orin Nano
- **Sensor**: Intel RealSense D435i RGB-D camera
- **Robot**: Humanoid robot platform (or simulation equivalent)

## Software Stack

This module leverages the NVIDIA Isaac ecosystem:

- **Isaac Sim 2025**: Advanced simulation environment with GPU-accelerated rendering
- **Isaac ROS**: GPU-accelerated perception and navigation packages
- **cuVSLAM**: NVIDIA's CUDA-accelerated Visual SLAM solution
- **DetectNet**: Real-time object detection neural network
- **PeopleSegNet**: Human segmentation for tracking and navigation safety
- **Nav2**: Navigation stack adapted for bipedal robots
- **ROS 2 Humble**: Robot operating system framework

## Module Structure

This module is organized into three progressive sections:

1. **Isaac Sim Basics**: Setting up Isaac Sim 2025 with ROS 2 bridge and humanoid robot loading
2. **Perception and VSLAM**: Deploying Isaac ROS perception gems on Jetson with cuVSLAM and real sensors
3. **Nav2 and Synthetic Data**: Bipedal navigation with Nav2 and synthetic data generation for sim-to-real

Each section builds upon the previous, allowing you to develop a comprehensive understanding of the AI-robot brain architecture from simulation to deployment.

## Performance Targets

Throughout this module, we'll focus on achieving specific performance benchmarks:

- Isaac Sim + ROS 2 bridge with humanoid at ≥60 FPS
- VSLAM map building in &lt; 30 seconds on Jetson
- Synthetic data generation: 5,000+ labeled images in &lt; 30 minutes

These targets ensure your implementations are production-ready and performant.

Let's begin your journey into the AI-robot brain with NVIDIA Isaac!