---
title: Ubuntu 22.04 + ROS 2 Humble Setup
sidebar_label: Ubuntu + ROS 2
sidebar_position: 4
description: Step-by-step installation guide for Ubuntu 22.04 LTS and ROS 2 Humble for Physical AI development
---

# Ubuntu 22.04 + ROS 2 Humble Setup

## Overview

This guide walks you through installing **Ubuntu 22.04 LTS** and **ROS 2 Humble** on a clean system. All commands are **copy-paste functional** and tested on Ubuntu 22.04.

**Estimated Time**: 2-3 hours (including downloads)

---

## System Requirements

### Minimum Hardware
- **CPU**: Intel Core i5-8400 or AMD Ryzen 5 3600 (6 cores)
- **RAM**: 16 GB DDR4
- **Storage**: 256 GB SSD (100 GB free for ROS 2 + workspace)
- **GPU**: NVIDIA GTX 1660 or better (for Isaac Sim later)

### Recommended Hardware
- **CPU**: Intel Core i7-12700 or AMD Ryzen 7 5800X (8+ cores)
- **RAM**: 32 GB DDR4
- **Storage**: 512 GB NVMe SSD
- **GPU**: NVIDIA RTX 4060 Ti or better

### Software Prerequisites
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Internet**: Stable connection for downloading ~5 GB of packages

---

## Step 1: Install Ubuntu 22.04 LTS

### Option A: Fresh Installation (Recommended)

1. **Download Ubuntu 22.04 LTS ISO**:
   - Primary: [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop) | [Archive](https://archive.is/PLACEHOLDER_UBUNTU_DOWNLOAD)
   - File: `ubuntu-22.04.3-desktop-amd64.iso` (4.7 GB)

2. **Create Bootable USB** (Windows/Mac/Linux):
   ```bash
   # Linux: Use dd command
   sudo dd if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX bs=4M status=progress && sync

   # Windows: Use Rufus (https://rufus.ie/)
   # Mac: Use balenaEtcher (https://www.balena.io/etcher/)
   ```

3. **Install Ubuntu**:
   - Boot from USB drive
   - Select "Install Ubuntu"
   - Choose language, keyboard layout
   - **Installation Type**: "Erase disk and install Ubuntu" (CAUTION: This wipes existing OS)
   - Set username, password
   - Wait 20-30 minutes for installation

4. **Reboot** and log in to Ubuntu desktop

### Option B: Dual Boot with Windows

Follow official guide: [Ubuntu Dual Boot Installation](https://help.ubuntu.com/community/WindowsDualBoot) | [Archive](https://archive.is/PLACEHOLDER_DUAL_BOOT)

### Option C: Virtual Machine (Not Recommended for Isaac Sim)

Use VirtualBox/VMware with:
- 8+ GB RAM allocated
- 100+ GB disk space
- Enable 3D acceleration
- **Note**: Isaac Sim will NOT run in VM (requires native GPU access)

---

## Step 2: Update System

After Ubuntu installation, update all packages:

```bash
sudo apt update && sudo apt upgrade -y
```

**Expected Output**:
```
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Calculating upgrade... Done
The following packages will be upgraded:
  [list of packages]
```

**Reboot** if kernel was updated:
```bash
sudo reboot
```

---

## Step 3: Install ROS 2 Humble

### Set Locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add ROS 2 APT Repository

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
```

Add ROS 2 GPG key:
```bash
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add repository to sources list:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Humble Desktop

Update package index:
```bash
sudo apt update
```

Install ROS 2 Humble Desktop (full GUI tools):
```bash
sudo apt install -y ros-humble-desktop
```

**Download Size**: ~1.2 GB
**Installation Time**: 10-15 minutes

**Expected Output**:
```
The following NEW packages will be installed:
  ros-humble-desktop ros-humble-ros-base ros-humble-rviz2 [...]
After this operation, 3,500 MB of additional disk space will be used.
```

---

## Step 4: Install Development Tools

### Colcon Build Tool

```bash
sudo apt install -y python3-colcon-common-extensions
```

### ROS 2 Development Dependencies

```bash
sudo apt install -y \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  build-essential \
  python3-vcstool
```

### Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

**Expected Output**:
```
reading in sources list data from /etc/ros/rosdep/sources.list.d
Query rosdistro index https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml
Add distro "rolling"
Add distro "humble"
updated cache in /home/username/.ros/rosdep/sources.cache
```

---

## Step 5: Configure Environment

### Source ROS 2 Setup

Add ROS 2 setup script to `.bashrc` for automatic sourcing:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify** ROS 2 is sourced:
```bash
echo $ROS_DISTRO
```

**Expected Output**:
```
humble
```

### Set ROS Domain ID (Optional)

If working in a shared network with multiple ROS 2 robots:
```bash
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**Note**: Domain ID 0-101 are reserved, use 42-200 for custom projects.

---

## Step 6: Verify ROS 2 Installation

### Test 1: Check ROS 2 Commands

```bash
ros2 --help
```

**Expected Output**:
```
usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

optional arguments:
  -h, --help            show this help message and exit
  [...]
```

### Test 2: Run Talker-Listener Demo

**Terminal 1** - Run talker:
```bash
ros2 run demo_nodes_cpp talker
```

**Expected Output**:
```
[INFO] [1701234567.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1701234568.123456789] [talker]: Publishing: 'Hello World: 2'
[INFO] [1701234569.123456789] [talker]: Publishing: 'Hello World: 3'
```

**Terminal 2** - Run listener:
```bash
ros2 run demo_nodes_cpp listener
```

**Expected Output**:
```
[INFO] [1701234567.123456789] [listener]: I heard: [Hello World: 1]
[INFO] [1701234568.123456789] [listener]: I heard: [Hello World: 2]
[INFO] [1701234569.123456789] [listener]: I heard: [Hello World: 3]
```

**Success Criteria**: Listener receives messages from talker in real-time.

---

## Step 7: Install Additional ROS 2 Packages

### Navigation Stack (Nav2)

```bash
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup
```

### MoveIt2 (Manipulation)

```bash
sudo apt install -y ros-humble-moveit
```

### Gazebo Integration

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

### Robot State Publisher & Joint State Publisher

```bash
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui
```

### RViz2 Plugins

```bash
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rviz-common \
  ros-humble-rviz-default-plugins
```

---

## Step 8: Create ROS 2 Workspace

### Create Workspace Directory

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### Build Empty Workspace

```bash
colcon build
```

**Expected Output**:
```
Starting >>> [empty workspace]
Finished <<< [empty workspace]

Summary: 0 packages finished [0.12s]
```

### Source Workspace

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 9: Install Python Dependencies

### Core Python Packages

```bash
sudo apt install -y python3-pip
pip3 install --upgrade pip
```

### ROS 2 Python Client

```bash
pip3 install rclpy
```

### Additional Tools

```bash
pip3 install \
  numpy \
  scipy \
  matplotlib \
  opencv-python \
  pyyaml
```

---

## Step 10: Verify Complete Setup

### Check ROS 2 Environment Variables

```bash
printenv | grep -i ros
```

**Expected Output**:
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
AMENT_PREFIX_PATH=/opt/ros/humble
CMAKE_PREFIX_PATH=/opt/ros/humble
[...]
```

### Check Installed ROS 2 Packages

```bash
ros2 pkg list | wc -l
```

**Expected Output**:
```
400+
```
(At least 400 ROS 2 packages installed)

---

## Troubleshooting

### Issue 1: "command 'ros2' not found"

**Solution**: Source ROS 2 setup script
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Issue 2: "Could not find a package configuration file provided by..."

**Solution**: Install missing dependency with rosdep
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

### Issue 3: colcon build fails with "CMake Error"

**Solution**: Install build dependencies
```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions
```

### Issue 4: Slow build times

**Solution**: Use parallel builds
```bash
colcon build --parallel-workers 4
```

### Issue 5: GPU not detected (for Isaac Sim later)

**Solution**: Install NVIDIA drivers
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

Verify GPU:
```bash
nvidia-smi
```

---

## Performance Optimization

### Increase colcon Build Speed

Add to `~/.bashrc`:
```bash
export COLCON_BUILD_PARALLEL_WORKERS=8  # Adjust based on CPU cores
export MAKEFLAGS="-j8"                   # Parallel make jobs
```

### Disable Unused Services

```bash
sudo systemctl disable bluetooth.service
sudo systemctl disable cups.service  # If you don't use printers
```

---

## Next Steps

✅ **Ubuntu 22.04 LTS** installed
✅ **ROS 2 Humble** installed and verified
✅ **Development tools** (colcon, rosdep) configured
✅ **ROS 2 workspace** created

**Next**: [Isaac Sim Installation](./isaac-sim-installation.md) for GPU-accelerated simulation

---

## External Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/) | [Archive](https://archive.is/PLACEHOLDER_ROS2_DOCS)
- [Ubuntu 22.04 LTS Release Notes](https://releases.ubuntu.com/22.04/) | [Archive](https://archive.is/PLACEHOLDER_UBUNTU_RELEASE)
- [ROS 2 Installation Troubleshooting](https://docs.ros.org/en/humble/Installation/Troubleshooting.html) | [Archive](https://archive.is/PLACEHOLDER_ROS2_TROUBLESHOOT)

---

**Previous**: [Repository Structure](./repository-structure.md) | **Next**: [Isaac Sim Installation](./isaac-sim-installation.md)
