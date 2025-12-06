# Quickstart Guide: Isaac Robot Brain (NVIDIA Isaac™ Platform)

## Prerequisites

- Ubuntu 22.04 LTS
- RTX 4070 Ti or equivalent GPU (for Isaac Sim ≥60 FPS)
- NVIDIA Jetson Orin Nano with latest JetPack
- RealSense D435i camera
- ROS 2 Humble Hawksbill installed
- Isaac Sim 2025 installed

## Installation Steps

### 1. Workstation Setup (RTX 4070 Ti + Ubuntu 22.04)

```bash
# Install Isaac Sim 2025
# Download from NVIDIA Developer website
# Follow installation instructions with ROS 2 bridge

# Verify Isaac Sim installation
isaac-sim --version

# Install Isaac ROS dependencies
sudo apt update
sudo apt install ros-humble-isaac-ros-* ros-humble-nav2-*

# Clone this repository
git clone [repo-url]
cd [repo-name]
```

### 2. Docker Setup for Workstation

```bash
# Build workstation Docker image
cd [repo-name]
docker build -f Dockerfile.workstation -t isaac-workstation .

# Run Isaac Sim container
docker run --gpus all --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix isaac-workstation
```

### 3. Jetson Setup

```bash
# Flash Jetson Orin Nano with latest JetPack
# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop ros-humble-isaac-ros-* ros-humble-nav2-*

# Build Docker image for Jetson
docker build -f Dockerfile.jetson -t isaac-jetson .

# Connect RealSense D435i and verify
rs-enumerate-devices
```

## Basic Operation

### 1. Launch Isaac Sim with Humanoid Robot

```bash
# Start Isaac Sim
isaac-sim

# Or run via Docker
docker run --gpus all --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix isaac-workstation

# In Isaac Sim:
# - Load humanoid robot model
# - Connect to ROS 2 bridge
# - Verify FPS is ≥60
```

### 2. Deploy VSLAM on Jetson

```bash
# Start cuVSLAM on Jetson
docker run --device=/dev/video0 --gpus all -it isaac-jetson
ros2 launch isaac_ros_cuvslam launch.py

# Connect RealSense D435i and verify mapping
# Map should build in <30 seconds
```

### 3. Run Navigation with Nav2

```bash
# Launch Nav2 for bipedal robot
ros2 launch nav2_bringup navigation_launch.py

# Send navigation goal
ros2 run nav2_msgs navigation_goal.py --x 1.0 --y 1.0
```

### 4. Generate Synthetic Data

```bash
# Run synthetic data generation in Isaac Sim
# Configure scene with humanoid and objects
# Set generation parameters for 5000 images
# Verify generation completes in <30 minutes
```

## Verification Steps

1. **FPS Check**: Isaac Sim with humanoid running ≥60 FPS
2. **VSLAM Test**: Map builds in <30 seconds on Jetson
3. **Synthetic Data**: 5,000 images generated in <30 minutes
4. **Navigation**: Robot successfully navigates to waypoints
5. **ROS Bridge**: All topics communicating properly

## Troubleshooting

- **Low FPS**: Reduce scene complexity or check GPU drivers
- **VSLAM failure**: Verify RealSense camera connection and calibration
- **Navigation issues**: Check robot kinematics configuration for bipedal
- **Docker errors**: Ensure GPU access and proper device mounting