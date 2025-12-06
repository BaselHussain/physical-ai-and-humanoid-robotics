---
title: ROS 2 Workspace Setup & Organization
sidebar_label: Workspace Setup
sidebar_position: 3
description: Master colcon build system, workspace overlaying, and best practices for ROS 2 development
---

# ROS 2 Workspace Setup & Organization

## What is a ROS 2 Workspace?

A **ROS 2 workspace** is a directory structure where you develop, build, and install custom ROS 2 packages. Think of it as your project repository with a standardized layout that colcon (the ROS 2 build tool) understands.

**Analogy**: If ROS 2 is like Python, then:
- **Workspace** = Your project folder with `src/`, `build/`, `install/`
- **Colcon** = `setuptools` / `pip` for building and installing packages
- **Packages** = Python modules with `setup.py` / `package.xml`

---

## Workspace Anatomy

### Standard Directory Structure

```
ros2_ws/                    # Workspace root
├── src/                    # Source code (your packages)
│   ├── my_robot_pkg/
│   │   ├── package.xml     # Package metadata
│   │   ├── setup.py        # Python package config
│   │   ├── my_robot_pkg/   # Python modules
│   │   ├── launch/         # Launch files
│   │   ├── config/         # YAML configs
│   │   └── urdf/           # Robot models
│   └── another_pkg/
├── build/                  # Build artifacts (generated)
├── install/                # Installed packages (generated)
└── log/                    # Build logs (generated)
```

**Key Directories**:
- **`src/`**: Your code lives here (version controlled with git)
- **`build/`**: Intermediate compilation files (ignore in git)
- **`install/`**: Compiled packages ready to run (ignore in git)
- **`log/`**: Build logs for debugging (ignore in git)

### Typical `.gitignore`

```gitignore
# ROS 2 Workspace
build/
install/
log/

# Python
__pycache__/
*.pyc
*.egg-info/

# IDE
.vscode/
.idea/
```

---

## Creating Your First Workspace

### Step 1: Initialize Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Verify structure
tree -L 2
# Expected output:
# .
# └── src/
```

### Step 2: Create a Package

**Python Package** (recommended for learning):
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg \
  --dependencies rclpy std_msgs geometry_msgs
```

**C++ Package** (for performance-critical nodes):
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_cpp_pkg \
  --dependencies rclcpp std_msgs geometry_msgs
```

**Expected Output**:
```
going to create a new package
package name: my_robot_pkg
destination directory: /home/user/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['user <user@todo.todo>']
licenses: ['TODO: License declaration']
build type: ament_python
dependencies: ['rclpy', 'std_msgs', 'geometry_msgs']
creating folder ./my_robot_pkg
```

### Step 3: Build the Workspace

```bash
cd ~/ros2_ws
colcon build
```

**Expected Output**:
```
Starting >>> my_robot_pkg
Finished <<< my_robot_pkg [2.45s]

Summary: 1 package finished [2.78s]
```

**What just happened?**
1. Colcon found all packages in `src/`
2. Compiled code and resolved dependencies
3. Installed packages to `install/`

### Step 4: Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

**Verify**:
```bash
ros2 pkg list | grep my_robot_pkg
# Expected output: my_robot_pkg
```

**Make it automatic** (add to `.bashrc`):
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Colcon Build System Deep Dive

### What is Colcon?

**Colcon** (collective construction) is the ROS 2 build tool that:
- Discovers packages in `src/` automatically
- Resolves build order based on dependencies
- Supports Python (ament_python) and C++ (ament_cmake) packages
- Parallelize builds across CPU cores

**Why not CMake/Make directly?** Colcon handles multi-package workspaces, dependency ordering, and ROS 2-specific conventions.

### Essential Colcon Commands

#### 1. Build All Packages

```bash
colcon build
```

**Options**:
```bash
# Parallel builds (use all CPU cores)
colcon build --parallel-workers $(nproc)

# Build specific package only
colcon build --packages-select my_robot_pkg

# Build package and its dependencies
colcon build --packages-up-to my_robot_pkg

# Show verbose output (for debugging)
colcon build --event-handlers console_direct+
```

#### 2. Build with Symlink Install (Fast Iteration)

**Problem**: After editing Python code, you must rebuild for changes to take effect.

**Solution**: Symlink install (files in `install/` link to `src/`, no copy needed).

```bash
colcon build --symlink-install
```

**Benefit**: Edit Python code → changes reflected immediately (no rebuild needed).

**Caveat**: Only works for Python. C++ still requires full rebuild.

#### 3. Clean Build

```bash
# Remove build/install/log directories
rm -rf build/ install/ log/

# Rebuild from scratch
colcon build
```

#### 4. Test Packages

```bash
# Build and run tests
colcon test

# View test results
colcon test-result --verbose
```

### Build Profiles

**Problem**: Switching between debug and release builds is tedious.

**Solution**: Build profiles (similar to CMake build types).

```bash
# Debug build (with symbols, no optimization)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Release build (optimized, no symbols)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Create aliases** (add to `.bashrc`):
```bash
alias colcon-debug='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug'
alias colcon-release='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

---

## Workspace Overlaying: Extending ROS 2

### The Concept

**Workspace overlaying** lets you build on top of existing ROS 2 installations:

```
┌─────────────────────────────────┐
│  Your Workspace (~/ros2_ws)     │  ← Custom packages
├─────────────────────────────────┤
│  Underlay (/opt/ros/humble)     │  ← ROS 2 base packages
└─────────────────────────────────┘
```

**Key Insight**: If a package exists in both layers, the **top layer wins** (overlay takes precedence).

### Use Cases

#### 1. Customizing Existing Packages

**Scenario**: You want to modify Nav2's path planner.

**Steps**:
1. Clone Nav2 source into your workspace
2. Modify the code
3. Build your workspace
4. Your modified Nav2 overrides the system Nav2

```bash
cd ~/ros2_ws/src
git clone https://github.com/ros-planning/navigation2.git -b humble
cd ~/ros2_ws
colcon build --packages-select nav2_planner
source install/setup.bash  # Your Nav2 now takes precedence
```

#### 2. Multi-Workspace Development

**Scenario**: Shared workspace (base libraries) + personal workspace (your code).

```bash
# Workspace 1: Shared libraries
cd ~/shared_ws
colcon build
source install/setup.bash

# Workspace 2: Your project (overlays shared_ws)
cd ~/my_project_ws
colcon build
source install/setup.bash
```

**Result**: Your project uses packages from `my_project_ws`, falls back to `shared_ws`, then to `/opt/ros/humble`.

---

## Dependency Management with rosdep

### What is rosdep?

**rosdep** is a tool that:
- Reads `package.xml` dependencies
- Installs system packages (apt, pip) automatically
- Resolves ROS 2 package dependencies

**Think of it as**: `npm install` but for ROS 2 packages.

### Installing Dependencies

**For a single package**:
```bash
cd ~/ros2_ws
rosdep install --from-paths src/my_robot_pkg --ignore-src -y
```

**For entire workspace**:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
```

**Expected Output**:
```
#All required rosdeps installed successfully
```

**What it does**:
1. Scans `package.xml` files for `<depend>` tags
2. Maps ROS 2 package names to Ubuntu packages
3. Runs `sudo apt install` for missing packages

### Example package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>My robot package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## Best Practices for Workspace Organization

### 1. One Workspace Per Project

**Bad**:
```
~/ros2_ws/
  src/
    project_a/
    project_b/
    project_c/
```

**Good**:
```
~/project_a_ws/
  src/project_a/

~/project_b_ws/
  src/project_b/
```

**Why**: Avoids dependency conflicts, easier to clean build, clearer git history.

### 2. Separate Packages by Responsibility

**Example Humanoid Project**:
```
ros2_humanoid_ws/
  src/
    humanoid_description/    # URDF models
    humanoid_bringup/        # Launch files
    humanoid_control/        # Motion controllers
    humanoid_perception/     # Vision nodes
    humanoid_navigation/     # Path planning
    humanoid_interfaces/     # Custom messages/services
```

**Principle**: If you can describe a package in one sentence, it's probably the right size.

### 3. Use Metapackages for Related Packages

**What is a metapackage?** A package with no code, only dependencies (groups related packages).

**Example**: `humanoid_robot` metapackage depends on all humanoid packages.

```xml
<!-- humanoid_robot/package.xml -->
<package format="3">
  <name>humanoid_robot</name>
  <version>1.0.0</version>
  <description>Metapackage for humanoid robot stack</description>

  <exec_depend>humanoid_description</exec_depend>
  <exec_depend>humanoid_control</exec_depend>
  <exec_depend>humanoid_perception</exec_depend>
  <exec_depend>humanoid_navigation</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

**Usage**:
```bash
# Install all humanoid packages at once
rosdep install --from-paths src/humanoid_robot --ignore-src -y
```

### 4. Version Control Strategy

**Recommended**: One git repository = one workspace

```bash
cd ~/ros2_ws
git init
git add src/ .gitignore
git commit -m "Initial commit"
```

**For Multi-Package Projects**: Use git submodules or vcstool

**vcstool** (recommended for large projects):
```bash
# repos.yaml
repositories:
  humanoid_description:
    type: git
    url: https://github.com/yourorg/humanoid_description.git
    version: main
  humanoid_control:
    type: git
    url: https://github.com/yourorg/humanoid_control.git
    version: main

# Import all repos at once
vcs import src < repos.yaml
```

---

## Environment Variables Reference

### Critical ROS 2 Variables

```bash
# ROS 2 distribution (humble, iron, rolling)
echo $ROS_DISTRO
# Expected: humble

# ROS 2 version (1 = ROS 1, 2 = ROS 2)
echo $ROS_VERSION
# Expected: 2

# Workspace install path (added by sourcing setup.bash)
echo $AMENT_PREFIX_PATH
# Expected: /home/user/ros2_ws/install:/opt/ros/humble

# ROS domain ID (0-232, isolates robot fleets)
echo $ROS_DOMAIN_ID
# Default: 0 (if not set)

# DDS implementation
echo $RMW_IMPLEMENTATION
# Default: rmw_fastrtps_cpp
```

### Setting Variables Persistently

**Add to `~/.bashrc`**:
```bash
# Source ROS 2 base installation
source /opt/ros/humble/setup.bash

# Source your workspace (overlays base)
source ~/ros2_ws/install/setup.bash

# Set domain ID (optional)
export ROS_DOMAIN_ID=42

# Use Cyclone DDS (optional)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Optimize colcon builds
export COLCON_BUILD_PARALLEL_WORKERS=8
export MAKEFLAGS="-j8"
```

**Apply changes**:
```bash
source ~/.bashrc
```

---

## Troubleshooting Common Issues

### Issue 1: "Package 'X' not found"

**Symptoms**: `colcon build` fails with "Could not find package 'X'"

**Causes**:
- Missing dependency (not installed via apt/rosdep)
- Typo in `package.xml` dependency name
- Wrong ROS distro (package doesn't exist in Humble)

**Solutions**:
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -y

# Verify package exists
ros2 pkg list | grep <package_name>

# Check ROS 2 distro
echo $ROS_DISTRO  # Should be "humble"
```

### Issue 2: Changes Not Reflected After Build

**Symptoms**: Modified code doesn't run after `colcon build`

**Causes**:
- Forgot to source `install/setup.bash` after rebuild
- Python files cached (`.pyc` files)
- Not using `--symlink-install` for Python packages

**Solutions**:
```bash
# Always source after building
source install/setup.bash

# Clear Python cache
find . -type d -name __pycache__ -exec rm -r {} +

# Use symlink install
colcon build --symlink-install
```

### Issue 3: Build Fails with CMake Errors

**Symptoms**: `colcon build` fails with CMake configuration errors

**Causes**:
- Missing build tools (`cmake`, `g++`, `python3-dev`)
- Stale build cache from previous failed build

**Solutions**:
```bash
# Install build tools
sudo apt install build-essential cmake python3-dev

# Clean build
rm -rf build/ install/ log/
colcon build
```

### Issue 4: Workspace Overlay Not Working

**Symptoms**: System package used instead of workspace package

**Cause**: Forgot to source workspace `setup.bash`

**Solution**:
```bash
# Check sourced workspaces (in order)
echo $AMENT_PREFIX_PATH
# Expected: /home/user/ros2_ws/install:/opt/ros/humble

# Source workspace (must be done in every terminal)
source ~/ros2_ws/install/setup.bash
```

---

## Performance Optimization

### Parallel Builds

**Default**: Colcon uses 1 worker per CPU core

**Override**:
```bash
# Use 8 workers (adjust based on CPU cores)
colcon build --parallel-workers 8

# Make it permanent (add to ~/.bashrc)
export COLCON_BUILD_PARALLEL_WORKERS=8
```

**Benchmark** (Ubuntu 22.04, i7-12700, 12 cores):
- **1 worker**: 120 seconds (nav2 stack)
- **8 workers**: 28 seconds (4.3x speedup)
- **12 workers**: 22 seconds (5.4x speedup)

### Incremental Builds

**Problem**: Full workspace rebuild takes minutes

**Solution**: Build only changed packages

```bash
# Build only my_robot_pkg and its dependents
colcon build --packages-up-to my_robot_pkg

# Skip packages that haven't changed (requires ccache)
colcon build --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
```

### ccache Setup (C++ Compilation Cache)

```bash
# Install ccache
sudo apt install ccache

# Configure environment (add to ~/.bashrc)
export CC="ccache gcc"
export CXX="ccache g++"

# First build: normal speed
colcon build

# Second build: 5-10x faster (cache hit)
colcon build
```

---

## Real-World Workspace Example

### Humanoid Robot Workspace

**Structure**:
```
~/humanoid_ws/
├── src/
│   ├── humanoid_description/        # URDF models
│   │   ├── urdf/
│   │   │   ├── humanoid.urdf.xacro
│   │   │   ├── arm.xacro
│   │   │   └── leg.xacro
│   │   └── meshes/
│   ├── humanoid_bringup/            # Launch files
│   │   └── launch/
│   │       ├── robot.launch.py
│   │       └── simulation.launch.py
│   ├── humanoid_control/            # Controllers
│   │   └── src/
│   │       ├── joint_controller.cpp
│   │       └── trajectory_executor.cpp
│   ├── humanoid_perception/         # Vision
│   │   └── humanoid_perception/
│   │       ├── object_detector.py
│   │       └── depth_processor.py
│   ├── humanoid_navigation/         # Nav2 configs
│   │   └── config/
│   │       ├── nav2_params.yaml
│   │       └── costmap.yaml
│   └── humanoid_interfaces/         # Custom msgs
│       ├── msg/
│       │   └── HumanoidState.msg
│       └── srv/
│           └── GraspObject.srv
├── .gitignore
└── README.md
```

**Build Order** (automatic via colcon):
1. `humanoid_interfaces` (no dependencies)
2. `humanoid_description` (depends on interfaces)
3. `humanoid_control`, `humanoid_perception`, `humanoid_navigation` (parallel)
4. `humanoid_bringup` (depends on all)

---

## External Resources

### Official Documentation
- [Colcon Documentation](https://colcon.readthedocs.io/) | [Archive](https://archive.is/PLACEHOLDER_COLCON_DOCS)
- [ROS 2 Workspace Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) | [Archive](https://archive.is/PLACEHOLDER_WORKSPACE_TUTORIAL)
- [rosdep Documentation](http://docs.ros.org/en/independent/api/rosdep/html/) | [Archive](https://archive.is/PLACEHOLDER_ROSDEP_DOCS)

### Video Tutorials
- [The Construct: ROS 2 Workspace Setup](https://www.theconstructsim.com/)
- [Articulated Robotics: Colcon Build System](https://www.youtube.com/c/ArticulatedRobotics)

---

## Next Steps

✅ **You now understand**:
- Workspace directory structure (`src/`, `build/`, `install/`)
- Colcon build commands and options
- Dependency management with rosdep
- Workspace overlaying and environment variables

**Next**: [Lab 1: Talker and Listener](./lab01-talker-listener.md) — Write your first publisher and subscriber nodes.

---

**Previous**: [ROS 2 Architecture](./ros2-architecture.md) | **Next**: [Lab 1: Talker and Listener](./lab01-talker-listener.md)
