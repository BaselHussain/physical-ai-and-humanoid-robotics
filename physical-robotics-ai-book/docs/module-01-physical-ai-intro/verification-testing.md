---
title: Verification & Testing
sidebar_label: Verification
sidebar_position: 6
description: Verify your complete Physical AI development environment with automated testing
---

# Verification & Testing

## Overview

This guide validates your complete Physical AI development environment using automated scripts and manual tests.

**Estimated Time**: 15-30 minutes

---

## Automated Verification Script

### Download Verification Script

```bash
cd ~/Downloads
wget https://raw.githubusercontent.com/physical-ai/physical-robotics-ai-book/main/scripts/verify-environment.sh
chmod +x verify-environment.sh
```

**Or if you've cloned the repository**:
```bash
cd ~/physical-robotics-ai-book/scripts
chmod +x verify-environment.sh
```

### Run Verification

```bash
./verify-environment.sh
```

### Expected Output (All Pass)

```
==========================================
  Physical AI Environment Verification
==========================================

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [1/7] System Requirements
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ PASS: Ubuntu 22.04 LTS detected
✅ PASS: CPU cores: 16 (≥6 required)
✅ PASS: RAM: 32 GB (≥16 GB required)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [2/7] NVIDIA GPU
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ PASS: NVIDIA driver 535.129.03 detected
✅ PASS: GPU: NVIDIA GeForce RTX 4070
✅ PASS: GPU memory: 12 GB (≥12 GB required)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [3/7] ROS 2 Humble
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ PASS: ROS 2 Humble environment sourced
✅ PASS: ros2 command available: ros2 version 0.26.1
✅ PASS: ROS 2 packages: 487 (≥400 expected)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [4/7] ROS 2 Development Tools
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ PASS: colcon build tool installed
✅ PASS: rosdep dependency manager installed

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [5/7] Isaac Sim
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ PASS: Isaac Sim 2024.1.1 found at /home/user/.local/share/ov/pkg/isaac-sim-2024.1.1
✅ PASS: Isaac Sim launch script is executable

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [6/7] ROS 2 Workspace
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ PASS: ROS 2 workspace found at /home/user/ros2_ws
✅ PASS: ROS 2 workspace built (install/ directory exists)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 [7/7] ROS 2 Demo Test (Talker/Listener)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Starting ROS 2 talker node (5 seconds)...
✅ PASS: ROS 2 talker node running

==========================================
  Verification Summary
==========================================

Passed: 15
Failed: 0

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ ALL CHECKS PASSED - Environment Ready!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Next steps:
  1. Test Isaac Sim: Launch Carter demo
  2. Clone book repository
  3. Start Module 02: Your First Humanoid
```

---

## Manual Test: Carter Robot Demo

### Launch Isaac Sim Carter Demo

**Method 1: Via Omniverse Launcher**
1. Open Omniverse Launcher
2. Launch Isaac Sim 2024.1.1
3. Menu: **Isaac Examples** → **ROS2** → **Navigation**
4. Select **"Carter Warehouse Navigation"**
5. Click **"Load"** (takes 30-60 seconds)

**Method 2: Via Command Line**
```bash
~/.local/share/ov/pkg/isaac-sim-2024.1.1/python.sh \
  standalone_examples/api/omni.isaac.ros2_bridge/carter_stereo.py
```

### Verify ROS 2 Topics

**Open new terminal**:
```bash
ros2 topic list
```

**Expected Output** (at least these 5 topics):
```
/clock
/cmd_vel
/odom
/scan
/camera/left/image_raw
/joint_states
```

### Check Topic Rates

```bash
# Scan topic (lidar)
ros2 topic hz /scan
```

**Expected**: 20 Hz

```bash
# Camera topic
ros2 topic hz /camera/left/image_raw
```

**Expected**: 30 Hz

```bash
# Odometry topic
ros2 topic hz /odom
```

**Expected**: 50 Hz

### Send Test Command

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}' \
  --rate 10
```

**Expected**: Carter robot moves forward slowly and turns left in Isaac Sim

**Stop** (Ctrl+C)

---

## Performance Benchmarks

### Minimum Passing Criteria

| Metric | Target | Measurement Command |
|--------|--------|---------------------|
| Isaac Sim Load Time | < 3 minutes | Time from launch to Carter loaded |
| ROS 2 Topics Active | ≥5 | `ros2 topic list \| wc -l` |
| `/scan` Topic Rate | ≥15 Hz | `ros2 topic hz /scan` |
| `/camera/left/image_raw` Rate | ≥25 Hz | `ros2 topic hz /camera/left/image_raw` |
| `/odom` Topic Rate | ≥40 Hz | `ros2 topic hz /odom` |
| GPU Memory Usage | < 8 GB | `nvidia-smi` (check used memory) |
| CPU Usage (idle sim) | < 60% | `htop` |

### Record Your Results

```bash
# Create benchmark log
echo "===== Physical AI Environment Benchmarks =====" > ~/env-benchmark.txt
echo "Date: $(date)" >> ~/env-benchmark.txt
echo "" >> ~/env-benchmark.txt

# System info
echo "[System]" >> ~/env-benchmark.txt
echo "CPU: $(lscpu | grep 'Model name' | cut -d':' -f2 | xargs)" >> ~/env-benchmark.txt
echo "RAM: $(free -h | awk '/^Mem:/{print $2}')" >> ~/env-benchmark.txt
echo "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader)" >> ~/env-benchmark.txt
echo "Driver: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader)" >> ~/env-benchmark.txt
echo "" >> ~/env-benchmark.txt

# ROS 2 info
echo "[ROS 2]" >> ~/env-benchmark.txt
echo "Distro: $ROS_DISTRO" >> ~/env-benchmark.txt
echo "Packages: $(ros2 pkg list | wc -l)" >> ~/env-benchmark.txt
echo "" >> ~/env-benchmark.txt

cat ~/env-benchmark.txt
```

---

## Troubleshooting Failed Checks

### Check 1 Failed: Ubuntu Version

**Issue**: Not running Ubuntu 22.04 LTS

**Solution**: Install Ubuntu 22.04 LTS (fresh install or dual boot)
- See: [Ubuntu + ROS 2 Setup](./ubuntu-ros2-setup.md)

### Check 2 Failed: GPU Not Detected

**Issue**: `nvidia-smi` not found

**Solution**: Install NVIDIA drivers
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
nvidia-smi
```

### Check 3 Failed: ROS 2 Not Sourced

**Issue**: `$ROS_DISTRO` is empty

**Solution**: Source ROS 2 setup
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Check 4 Failed: colcon Not Found

**Issue**: Build tool missing

**Solution**: Install colcon
```bash
sudo apt install python3-colcon-common-extensions
```

### Check 5 Failed: Isaac Sim Not Found

**Issue**: Isaac Sim not installed

**Solution**: Install via Omniverse Launcher
- See: [Isaac Sim Installation](./isaac-sim-installation.md)

### Check 6 Failed: ROS 2 Workspace Not Built

**Issue**: `~/ros2_ws/install` doesn't exist

**Solution**: Build workspace
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Check 7 Failed: Talker Node Won't Start

**Issue**: ROS 2 demo nodes not installed

**Solution**: Install demo packages
```bash
sudo apt install ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py
```

---

## Advanced Validation Tests

### Test 1: Multi-Robot Simulation

Launch 2 Carter robots in Isaac Sim:
```bash
cd ~/.local/share/ov/pkg/isaac-sim-2024.1.1
./python.sh standalone_examples/api/omni.isaac.ros2_bridge/multiple_robot.py --robots 2
```

**Verify**:
```bash
ros2 topic list | grep cmd_vel
```

**Expected**:
```
/carter01/cmd_vel
/carter02/cmd_vel
```

### Test 2: RViz2 Visualization

```bash
rviz2
```

**In RViz2**:
1. Add → By topic → `/scan` → LaserScan
2. Fixed Frame: `odom`
3. Add → TF

**Expected**: See lidar scan visualization updating in real-time

### Test 3: Topic Echo

```bash
ros2 topic echo /odom --once
```

**Expected Output** (odometry message):
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: odom
pose:
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
```

---

## Success Checklist

Mark each item when verified:

- [ ] ✅ Ubuntu 22.04 LTS running
- [ ] ✅ NVIDIA GPU driver ≥525.x installed
- [ ] ✅ ROS 2 Humble sourced automatically
- [ ] ✅ ros2 command works
- [ ] ✅ colcon build tool installed
- [ ] ✅ Isaac Sim 2024.1.1 launches
- [ ] ✅ Carter demo loads in < 3 minutes
- [ ] ✅ 5+ ROS 2 topics active
- [ ] ✅ `/scan` publishing at ≥15 Hz
- [ ] ✅ `/cmd_vel` commands move robot in sim
- [ ] ✅ RViz2 visualizes sensor data

**If all checked**: ✅ **Environment ready for Module 02!**

---

## Next Steps

Your Physical AI development environment is now fully configured and validated. You're ready to:

1. **Start Module 02**: Create your first humanoid URDF model
2. **Explore Isaac Sim**: Try other robot examples (Franka arm, Quadruped)
3. **Clone book repository**: `git clone https://github.com/physical-ai/physical-robotics-ai-book.git`

---

## External Resources

- [ROS 2 Troubleshooting](https://docs.ros.org/en/humble/Installation/Troubleshooting.html) | [Archive](https://archive.is/PLACEHOLDER_ROS2_TROUBLESHOOT)
- [Isaac Sim Known Issues](https://docs.omniverse.nvidia.com/isaacsim/latest/known_issues.html) | [Archive](https://archive.is/PLACEHOLDER_ISAAC_ISSUES)
- [NVIDIA GPU Troubleshooting](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#troubleshooting) | [Archive](https://archive.is/PLACEHOLDER_CUDA_TROUBLESHOOT)

---

**Previous**: [Isaac Sim Installation](./isaac-sim-installation.md) | **Next**: [Hardware Requirements](./hardware-requirements.md)
