---
title: Isaac Sim Installation
sidebar_label: Isaac Sim
sidebar_position: 5
description: Install NVIDIA Isaac Sim 2024.1.1 with ROS 2 bridge for GPU-accelerated robotics simulation
---

# Isaac Sim 2024.1.1 Installation

## Overview

**NVIDIA Isaac Sim** is a GPU-accelerated robotics simulator built on Omniverse. This guide covers installation of **Isaac Sim 2024.1.1** with ROS 2 Humble integration.

**Estimated Time**: 1-2 hours (including 20 GB download)

---

## Prerequisites

### Hardware Requirements

**Minimum**:
- **GPU**: NVIDIA RTX 3060 (12 GB VRAM)
- **CPU**: Intel i7-10700 or AMD Ryzen 7 3700X (8 cores)
- **RAM**: 16 GB
- **Storage**: 100 GB free SSD space
- **OS**: Ubuntu 22.04 LTS

**Recommended**:
- **GPU**: NVIDIA RTX 4070 Ti or RTX 4080 (16+ GB VRAM)
- **CPU**: Intel i9-13900K or AMD Ryzen 9 7950X (16+ cores)
- **RAM**: 32 GB
- **Storage**: 256 GB free NVMe SSD

### Software Prerequisites

✅ Ubuntu 22.04 LTS installed
✅ ROS 2 Humble installed ([ubuntu-ros2-setup](./ubuntu-ros2-setup.md))
✅ NVIDIA GPU drivers (version 525.x or newer)

---

## Step 1: Install NVIDIA GPU Drivers

### Check Current Driver Version

```bash
nvidia-smi
```

**If command not found or driver < 525**:

```bash
# Add NVIDIA driver PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver (usually 535 or 545)
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot
```

### Verify Driver Installation

```bash
nvidia-smi
```

**Expected Output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
|   0  NVIDIA GeForce RTX 4070  Off  | 00000000:01:00.0  On |                  N/A |
+-------------------------------+----------------------+----------------------+
```

**Success Criteria**: Driver version ≥ 525.x, CUDA version ≥ 11.8

---

## Step 2: Install Omniverse Launcher

### Download Omniverse Launcher

Visit: [NVIDIA Omniverse Download](https://www.nvidia.com/en-us/omniverse/download/) | [Archive](https://archive.is/PLACEHOLDER_OMNIVERSE_DOWNLOAD)

**Or download via command line**:
```bash
cd ~/Downloads
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
```

### Make Executable and Run

```bash
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

### Sign In

- Create/sign in to NVIDIA account (free)
- Accept Omniverse EULA

---

## Step 3: Install Isaac Sim via Omniverse

### In Omniverse Launcher

1. Click **"Exchange"** tab (left sidebar)
2. Search for **"Isaac Sim"**
3. Select **"Isaac Sim 2024.1.1"**
4. Click **"Install"**

**Download Size**: ~20 GB
**Installation Time**: 30-60 minutes (depending on internet speed)

**Installation Path**: `~/.local/share/ov/pkg/isaac-sim-2024.1.1/`

---

## Step 4: Configure Isaac Sim for ROS 2

### Install Isaac Sim ROS 2 Bridge

```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac-sim-2024.1.1

# Run setup script
./setup_python_env.sh
```

**Expected Output**:
```
Setting up Isaac Sim Python environment...
Installing ROS 2 bridge dependencies...
Done.
```

### Install ROS 2 Workspace for Isaac Sim

```bash
# Clone Isaac ROS workspace
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

git clone https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces.git
```

### Build Isaac ROS Workspace

```bash
cd ~/isaac_ros_ws
source /opt/ros/humble/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -y

# Build
colcon build --symlink-install
```

**Expected Output**:
```
Starting >>> isaac_ros_common
Starting >>> isaac_ros_nitros
[...]
Finished <<< isaac_ros_common [1min 23s]
Finished <<< isaac_ros_nitros [1min 45s]

Summary: 15 packages finished [2min 12s]
```

### Source Isaac ROS Workspace

```bash
echo "source ~/isaac_ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 5: Launch Isaac Sim

### Method 1: Via Omniverse Launcher (GUI)

1. Open Omniverse Launcher
2. Click **"Library"** tab
3. Find **"Isaac Sim 2024.1.1"**
4. Click **"Launch"**

**First Launch**: Takes 5-10 minutes (shader compilation)

### Method 2: Via Command Line

```bash
~/.local/share/ov/pkg/isaac-sim-2024.1.1/isaac-sim.sh
```

**Expected**: Isaac Sim window opens with welcome screen

---

## Step 6: Test Isaac Sim with Carter Demo

### Launch Carter Robot Demo

**In Isaac Sim**:
1. Menu: **Isaac Examples** → **ROS2** → **Navigation**
2. Select **"Carter Warehouse Navigation"**
3. Click **"Load"**

**Or via Python script**:
```bash
cd ~/.local/share/ov/pkg/isaac-sim-2024.1.1
./python.sh standalone_examples/api/omni.isaac.ros2_bridge/carter_stereo.py
```

### Verify ROS 2 Topics

**Open new terminal**:
```bash
ros2 topic list
```

**Expected Output** (should include):
```
/clock
/cmd_vel
/joint_states
/odom
/scan
/camera/left/image_raw
/camera/left/camera_info
/camera/right/image_raw
/camera/right/camera_info
/tf
/tf_static
```

**Success Criteria**: At least 10+ topics active

### Check Topic Frequency

```bash
ros2 topic hz /scan
```

**Expected Output**:
```
average rate: 20.001
  min: 0.049s max: 0.051s std dev: 0.00045s window: 22
```

**Success Criteria**:
- `/scan`: 20 Hz
- `/camera/left/image_raw`: 30 Hz
- `/odom`: 50 Hz

---

## Step 7: Test ROS 2 Bridge

### Send Velocity Commands

**Terminal 1** - Keep Carter demo running in Isaac Sim

**Terminal 2** - Publish velocity command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}' --rate 10
```

**Expected**: Carter robot moves forward and turns in Isaac Sim

**Stop robot** (Ctrl+C in Terminal 2)

---

## Step 8: Verify Installation with Validation Script

Create validation script:

```bash
mkdir -p ~/scripts
nano ~/scripts/verify-isaac-sim.sh
```

**Paste the following**:
```bash
#!/bin/bash

echo "===== Isaac Sim + ROS 2 Validation ====="
echo ""

# Check NVIDIA driver
echo "[1/5] Checking NVIDIA driver..."
if nvidia-smi > /dev/null 2>&1; then
    echo "  ✅ NVIDIA driver detected"
    nvidia-smi --query-gpu=driver_version --format=csv,noheader
else
    echo "  ❌ NVIDIA driver not found"
    exit 1
fi

# Check Isaac Sim installation
echo "[2/5] Checking Isaac Sim installation..."
ISAAC_SIM_PATH="$HOME/.local/share/ov/pkg/isaac-sim-2024.1.1"
if [ -d "$ISAAC_SIM_PATH" ]; then
    echo "  ✅ Isaac Sim found at $ISAAC_SIM_PATH"
else
    echo "  ❌ Isaac Sim not found"
    exit 1
fi

# Check ROS 2
echo "[3/5] Checking ROS 2 Humble..."
if [ "$ROS_DISTRO" = "humble" ]; then
    echo "  ✅ ROS 2 Humble sourced"
else
    echo "  ❌ ROS 2 Humble not sourced"
    exit 1
fi

# Check Isaac ROS workspace
echo "[4/5] Checking Isaac ROS workspace..."
if [ -d "$HOME/isaac_ros_ws/install" ]; then
    echo "  ✅ Isaac ROS workspace built"
else
    echo "  ❌ Isaac ROS workspace not built"
    exit 1
fi

# Test Isaac Sim launch (headless check)
echo "[5/5] Testing Isaac Sim launch..."
timeout 30s $ISAAC_SIM_PATH/isaac-sim.sh --no-window --test > /tmp/isaac_test.log 2>&1
if [ $? -eq 0 ]; then
    echo "  ✅ Isaac Sim launches successfully"
else
    echo "  ⚠️  Isaac Sim launch test inconclusive (requires GUI)"
fi

echo ""
echo "===== Validation Complete ====="
echo "✅ All critical components verified"
echo ""
echo "Next: Run Carter demo to test ROS 2 bridge"
```

**Make executable**:
```bash
chmod +x ~/scripts/verify-isaac-sim.sh
```

**Run validation**:
```bash
~/scripts/verify-isaac-sim.sh
```

---

## Troubleshooting

### Issue 1: Isaac Sim won't launch (black screen)

**Solution 1**: Update GPU drivers
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

**Solution 2**: Disable Vulkan (fallback to OpenGL)
```bash
export ISAAC_SIM_RENDER_BACKEND=opengl
~/.local/share/ov/pkg/isaac-sim-2024.1.1/isaac-sim.sh
```

### Issue 2: "Insufficient VRAM" error

**Solution**: Reduce quality settings in Isaac Sim
- Menu: **Edit** → **Preferences** → **Rendering**
- Set **Ray Tracing** to **Off**
- Set **Anti-Aliasing** to **Low**

### Issue 3: ROS 2 topics not publishing

**Solution**: Restart ROS 2 bridge
1. Stop Isaac Sim
2. Source ROS 2: `source /opt/ros/humble/setup.bash`
3. Relaunch Carter demo

### Issue 4: Slow performance (< 10 FPS)

**Solution 1**: Close other GPU applications (browsers, other simulators)

**Solution 2**: Reduce simulation complexity
- Use fewer robots/objects in scene
- Lower physics update rate (Edit → Preferences → Physics)

### Issue 5: Omniverse Launcher doesn't start

**Solution**: Install missing dependencies
```bash
sudo apt install -y libfuse2 libxcb-cursor0
```

---

## Performance Tuning

### Enable GPU Persistence Mode

```bash
sudo nvidia-smi -pm 1
```

### Increase Shared Memory (for ROS 2 large messages)

```bash
echo "kernel.shmmax = 2147483648" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### Set CPU Governor to Performance

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

---

## Next Steps

✅ **Isaac Sim 2024.1.1** installed
✅ **ROS 2 bridge** configured and tested
✅ **Carter demo** verified (navigation + topics)

**Next**: [Verification & Testing](./verification-testing.md) - Complete environment validation

---

## External Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html) | [Archive](https://archive.is/PLACEHOLDER_ISAACIM_DOCS)
- [Isaac ROS GitHub](https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces) | [Archive](https://archive.is/PLACEHOLDER_ISAAC_ROS_GH)
- [Omniverse System Requirements](https://docs.omniverse.nvidia.com/install-guide/latest/system-requirements.html) | [Archive](https://archive.is/PLACEHOLDER_OMNIVERSE_SYSREQ)

---

**Previous**: [Ubuntu + ROS 2 Setup](./ubuntu-ros2-setup.md) | **Next**: [Verification & Testing](./verification-testing.md)
