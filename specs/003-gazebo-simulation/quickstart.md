# Quickstart: Module 03 — The Digital Twin (Gazebo & Unity)

**Feature**: 003-gazebo-simulation
**Date**: 2025-12-06
**Purpose**: Fast-track setup guide for launching sensor-equipped humanoid in Gazebo

## Prerequisites

**Required**:
- Ubuntu 24.04 LTS (Noble) - Native installation recommended (not WSL2)
- ROS 2 Jazzy installed and configured
- RTX 4070 Ti or equivalent GPU (minimum: GTX 1660 Ti)
- 16 GB RAM minimum
- 50 GB free disk space

**Knowledge Prerequisites**:
- Completed Module 01 (ROS 2 basics)
- Familiar with ROS 2 launch files, topics, and tf2
- Basic terminal/command-line proficiency

## Time-to-Launch Target: < 10 Minutes

This quickstart is designed to get a fully sensor-equipped humanoid running in Gazebo in under 10 minutes from a properly configured ROS 2 environment.

## Step 1: Install Gazebo Harmonic (3 minutes)

### Install Gazebo Harmonic

```bash
# Add Gazebo packages repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install
sudo apt-get update
sudo apt-get install -y gz-harmonic

# Verify installation
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

### Install ROS 2 - Gazebo Bridge

```bash
# Install ros_gz packages for Jazzy
sudo apt-get install -y ros-jazzy-ros-gz

# Source ROS 2
source /opt/ros/jazzy/setup.bash
```

## Step 2: Set Up Workspace (2 minutes)

### Create ROS 2 Workspace

```bash
# Create workspace
mkdir -p ~/gazebo_humanoid_ws/src
cd ~/gazebo_humanoid_ws/src

# Clone humanoid robot packages (placeholder - will be provided in module)
# git clone https://github.com/example/humanoid_description.git
# git clone https://github.com/example/apartment_worlds.git

# For now, create placeholder structure
mkdir -p humanoid_description/urdf
mkdir -p humanoid_description/meshes/visual
mkdir -p humanoid_description/meshes/collision
mkdir -p humanoid_description/launch
mkdir -p apartment_worlds/worlds
mkdir -p apartment_worlds/models

# Build workspace
cd ~/gazebo_humanoid_ws
colcon build
source install/setup.bash
```

### Set Environment Variables

```bash
# Add Gazebo model paths (add to ~/.bashrc for persistence)
export GZ_SIM_RESOURCE_PATH=~/gazebo_humanoid_ws/src/apartment_worlds/models:~/gazebo_humanoid_ws/src/humanoid_description
export GZ_SIM_SYSTEM_PLUGIN_PATH=~/gazebo_humanoid_ws/install/lib

# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source ~/gazebo_humanoid_ws/install/setup.bash
```

## Step 3: Launch Simulation (< 1 minute)

### Quick Launch - Humanoid in Empty World

```bash
# Launch Gazebo with humanoid robot
ros2 launch humanoid_description humanoid_gazebo.launch.py
```

This launch file will:
1. Start Gazebo Harmonic with apartment world
2. Spawn humanoid robot with all sensors
3. Start ros_gz_bridge for sensor topics
4. Publish robot description to /robot_description

### Verify Sensors Are Running

Open a new terminal:

```bash
# Source workspace
source ~/gazebo_humanoid_ws/install/setup.bash

# Check available topics
ros2 topic list

# Expected topics:
# /robot/joint_states
# /robot/depth_camera/image_raw
# /robot/depth_camera/camera_info
# /robot/lidar/points
# /robot/imu/data
# /robot/contact/left_foot
# /robot/contact/right_foot
# /clock

# Check sensor update rates
ros2 topic hz /robot/depth_camera/image_raw
# Expected: ~30 Hz

ros2 topic hz /robot/lidar/points
# Expected: ~10 Hz

ros2 topic hz /robot/imu/data
# Expected: ~100 Hz
```

## Step 4: Visualize in RViz2 (2 minutes)

### Launch RViz2

```bash
# Launch RViz2 with preset configuration
ros2 run rviz2 rviz2 -d ~/gazebo_humanoid_ws/src/humanoid_description/rviz/simulation.rviz
```

### Manual RViz2 Setup (if no config provided)

1. **Add Robot Model**:
   - Click "Add" → "RobotModel"
   - Set "Description Topic" to `/robot_description`
   - Set "Fixed Frame" to `world` or `base_link`

2. **Add Depth Camera**:
   - Click "Add" → "Image"
   - Set "Image Topic" to `/robot/depth_camera/image_raw`

3. **Add LiDAR Point Cloud**:
   - Click "Add" → "PointCloud2"
   - Set "Topic" to `/robot/lidar/points`
   - Set "Size" to `0.01`
   - Set "Color Transformer" to "Intensity" or "AxisColor"

4. **Add IMU**:
   - Click "Add" → "Imu"
   - Set "Topic" to `/robot/imu/data`

5. **Set Fixed Frame** to `world` or `odom`

## Step 5: Performance Check (1 minute)

### Monitor Simulation Performance

```bash
# Check Gazebo stats
gz topic -e -t /stats

# Look for:
# - real_time_factor: Should be ≥ 1.0 (target: 1.0-3.5)
# - sim_time: Advancing at expected rate
# - iterations: Physics steps per second (~1000 Hz)

# Monitor GPU usage
nvidia-smi -l 1

# Expected on RTX 4070 Ti:
# - GPU Utilization: 40-60%
# - Memory Usage: 1-2 GB
# - Temperature: 50-70°C
```

### Check FPS

In Gazebo GUI, press `Ctrl+T` to show statistics overlay. Target: ≥60 FPS.

## Optional: Unity Visualization Setup (10-15 minutes)

### Install Unity Hub and Unity Editor

1. Download Unity Hub: https://unity.com/download
2. Install Unity 2022.3 LTS or later
3. Install Unity ROS TCP Connector package via Package Manager

### Install ROS TCP Endpoint

```bash
# Install ROS TCP endpoint
sudo apt-get install -y ros-jazzy-ros-tcp-endpoint

# Run endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

### Configure Unity Project

1. Create new Unity 3D project
2. Install ROS TCP Connector via Package Manager:
   - Click "Window" → "Package Manager"
   - Click "+" → "Add package from git URL"
   - Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

3. Configure ROSConnection:
   - Click "Robotics" → "ROS Settings"
   - Set ROS IP Address: `127.0.0.1`
   - Set ROS Port: `10000`
   - Protocol: `ROS2`

4. Create subscriber scripts for `/robot/joint_states` and `/robot/depth_camera/image_raw`

5. Build and test visualization

### Launch Complete System

```bash
# Terminal 1: Gazebo simulation
ros2 launch humanoid_description humanoid_gazebo.launch.py

# Terminal 2: ROS-Gazebo bridge (if not in launch file)
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=~/gazebo_humanoid_ws/src/humanoid_description/config/bridge_config.yaml

# Terminal 3: ROS TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000

# Terminal 4 (optional): RViz2
ros2 run rviz2 rviz2
```

## Troubleshooting

### Gazebo Won't Start

**Issue**: `gz sim` command not found
**Solution**: Ensure Gazebo Harmonic is installed and sourced:
```bash
gz sim --version
# If not found, reinstall Gazebo Harmonic
```

### Low Frame Rate (< 30 FPS)

**Issue**: Simulation running slowly
**Possible Causes**:
1. Shadows enabled (disable in world SDF: `<shadows>false</shadows>`)
2. High polygon count meshes (check and decimate)
3. Too many sensors active (reduce update rates)
4. Running in WSL2 (use native Ubuntu for 2-3x performance)

**Solution**: Check RTF with `gz topic -e -t /stats`. If RTF < 1.0:
- Disable shadows
- Reduce sensor update rates
- Simplify collision meshes

### Sensors Not Publishing

**Issue**: `ros2 topic list` doesn't show sensor topics
**Solution**: Verify ros_gz_bridge is running:
```bash
ros2 node list
# Should show: /ros_gz_bridge

# Check bridge configuration
ros2 param get /ros_gz_bridge config_file
```

### Unity Can't Connect

**Issue**: Unity shows "Connection failed"
**Solution**:
1. Verify ros_tcp_endpoint is running: `ros2 node list`
2. Check firewall settings (allow port 10000)
3. Verify ROS_IP is correct (use `127.0.0.1` for local)
4. Check Unity console for detailed error messages

### GPU Not Used

**Issue**: `nvidia-smi` shows 0% GPU usage
**Solution**: Ensure Gazebo is using GPU rendering:
```bash
# Check rendering engine
gz model --list-engines

# Should show: ogre2 (GPU-accelerated)
```

## Performance Benchmarks

Expected performance on RTX 4070 Ti with Ubuntu 24.04:

| Metric | Target | Typical |
|--------|--------|---------|
| Real-time Factor | ≥ 1.0 | 1.0-3.5 |
| Rendering FPS | ≥ 60 | 60-120 |
| Physics Rate | 1000 Hz | 1000 Hz |
| GPU Utilization | 40-60% | 40-60% |
| CPU Utilization | 60-80% | 60-80% |
| Memory Usage | < 4 GB | 2-4 GB |
| Startup Time | < 10 min | 5-8 min |

## Next Steps

After successful launch:

1. **Explore Module Content**:
   - Read `docs/module-02/01-gazebo-fundamentals.md` for detailed Gazebo concepts
   - Read `docs/module-02/02-sensors-and-humanoid.md` for sensor configuration
   - Read `docs/module-02/03-apartment-worlds-and-unity.md` for world building

2. **Experiment with Sensors**:
   - Subscribe to sensor topics in Python
   - Visualize depth images with `rqt_image_view`
   - Process point clouds with PCL

3. **Build Your Own World**:
   - Use gazebo-world-builder skill to create custom environments
   - Add furniture and obstacles
   - Test humanoid navigation

4. **Advanced Topics**:
   - Implement basic walking controller
   - Add object detection with depth camera
   - Create custom sensor plugins

## Resources

- Gazebo Harmonic Documentation: https://gazebosim.org/docs/harmonic
- ROS 2 Jazzy Documentation: https://docs.ros.org/en/jazzy
- Unity ROS TCP Connector: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- Module GitHub Repository: [To be provided]

## Support

If you encounter issues not covered here:
1. Check module documentation in `docs/module-02/`
2. Review GitHub Issues: [To be provided]
3. Post questions on ROS Answers: https://answers.ros.org
4. Join Gazebo Community: https://community.gazebosim.org

---

**Congratulations!** You now have a fully sensor-equipped humanoid running in Gazebo Harmonic. Continue to the module chapters to learn advanced simulation techniques and world building.
