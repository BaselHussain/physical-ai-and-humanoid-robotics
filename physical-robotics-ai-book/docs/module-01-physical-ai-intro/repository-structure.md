---
title: Repository Structure
sidebar_label: Repository
sidebar_position: 3
description: Understanding the Physical AI book repository structure, CI/CD workflows, and development setup
---

# Repository Structure

## Overview

The **physical-robotics-ai-book** repository is organized for maximum learning efficiency and production-ready development practices. Every module includes runnable code, pre-configured launch files, and automated testing.

---

## Folder Structure

```
physical-robotics-ai-book/
├── docs/                          # Docusaurus documentation site
│   ├── module-00-overview/        # Book introduction
│   ├── module-01-physical-ai-intro/  # This module
│   ├── module-02-first-humanoid/  # Your first URDF model
│   └── ...                        # Modules 03-10
├── src/                           # ROS 2 workspace
│   ├── humanoid_description/      # URDF/Xacro robot models
│   ├── humanoid_gazebo/           # Gazebo worlds and launch files
│   ├── humanoid_perception/       # Isaac ROS perception configs
│   ├── humanoid_control/          # Controllers and planners
│   └── humanoid_vla/              # VLA orchestration scripts
├── scripts/                       # Standalone Python/Bash scripts
│   ├── verify-environment.sh      # Environment validation
│   ├── setup-jetson.sh            # Jetson deployment setup
│   └── domain-randomization.py    # Sim-to-real helpers
├── config/                        # Configuration files
│   ├── ros2/                      # ROS 2 parameter files
│   ├── isaac_sim/                 # Isaac Sim configs
│   └── jetson/                    # Jetson-specific configs
├── docker/                        # Docker containers
│   ├── Dockerfile.dev             # Development environment
│   ├── Dockerfile.jetson          # Jetson deployment image
│   └── docker-compose.yml         # Multi-container orchestration
├── .github/                       # CI/CD workflows
│   └── workflows/
│       ├── build-test.yml         # Build + unit tests
│       ├── integration-test.yml   # Gazebo/Isaac Sim integration
│       └── deploy-docs.yml        # Docusaurus deployment
└── README.md                      # Quick start guide
```

---

## ROS 2 Workspace Structure

### `src/humanoid_description/`
**Purpose**: Robot URDF/Xacro models and mesh files

**Contents**:
```
humanoid_description/
├── urdf/
│   ├── humanoid.urdf.xacro        # Main robot description
│   ├── sensors.urdf.xacro         # Camera, lidar, IMU
│   └── ros2_control.urdf.xacro    # Controller interfaces
├── meshes/
│   ├── visual/                    # High-res meshes for rendering
│   └── collision/                 # Simplified collision meshes
├── config/
│   └── joint_limits.yaml          # Joint position/velocity/effort limits
└── launch/
    └── display.launch.py          # RViz2 visualization
```

**Key Files**:
- `humanoid.urdf.xacro`: Main robot model (20-32 DoF humanoid)
- `sensors.urdf.xacro`: RealSense D435, Hokuyo lidar, IMU sensor integration
- `ros2_control.urdf.xacro`: Position/velocity/effort controllers

---

### `src/humanoid_gazebo/`
**Purpose**: Gazebo simulation worlds and launch files

**Contents**:
```
humanoid_gazebo/
├── worlds/
│   ├── empty.world                # Empty world for testing
│   ├── apartment.world            # 40m² apartment (Module 03)
│   └── warehouse.world            # Warehouse with obstacles (Module 04)
├── models/
│   ├── furniture/                 # Tables, chairs, shelves
│   └── objects/                   # Graspable objects (cups, bottles)
├── launch/
│   ├── gazebo.launch.py           # Launch Gazebo + robot
│   └── spawn_robot.launch.py     # Spawn robot at specific pose
└── config/
    └── physics.yaml               # ODE/Bullet physics parameters
```

**Key Files**:
- `apartment.world`: 3-room apartment with furniture for navigation testing
- `gazebo.launch.py`: Launches Gazebo + robot + ROS 2 bridge + RViz2

---

### `src/humanoid_perception/`
**Purpose**: Isaac ROS perception pipelines

**Contents**:
```
humanoid_perception/
├── launch/
│   ├── cuvslam.launch.py          # Visual SLAM pipeline
│   ├── detectnet.launch.py        # Object detection pipeline
│   └── depth_processing.launch.py # Depth to point cloud
├── config/
│   ├── cuvslam_params.yaml        # cuVSLAM tuning parameters
│   ├── detectnet_params.yaml      # DetectNet confidence thresholds
│   └── camera_calibration.yaml    # RealSense intrinsics
└── models/
    └── peoplenet_resnet34_int8.onnx  # Pre-trained DetectNet model
```

**Key Files**:
- `cuvslam.launch.py`: Launches cuVSLAM + RealSense camera + odometry publisher
- `detectnet_params.yaml`: Confidence threshold (0.6), IOU threshold (0.4), bbox area min (100px²)

---

### `src/humanoid_control/`
**Purpose**: Navigation, manipulation, and control stacks

**Contents**:
```
humanoid_control/
├── launch/
│   ├── navigation.launch.py       # Nav2 stack for mobile base
│   ├── manipulation.launch.py     # MoveIt2 for arm control
│   └── full_control.launch.py     # Combined navigation + manipulation
├── config/
│   ├── nav2_params.yaml           # Nav2 planner/controller params
│   ├── moveit2_config.yaml        # MoveIt2 OMPL planner config
│   └── joint_trajectory_controller.yaml  # ros2_control configs
└── src/
    ├── waypoint_follower.cpp      # C++ node for waypoint navigation
    └── grasp_planner.py           # Python node for grasp planning
```

**Key Files**:
- `navigation.launch.py`: Launches Nav2 with DWB controller, recoveries, costmap configs
- `manipulation.launch.py`: Launches MoveIt2 with OMPL RRTConnect planner

---

### `src/humanoid_vla/`
**Purpose**: Vision-Language-Action orchestration

**Contents**:
```
humanoid_vla/
├── scripts/
│   ├── llm_orchestrator.py        # GPT-4o/Claude orchestration node
│   ├── prompt_templates.py        # System prompts for LLM
│   └── action_executor.py         # Execute LLM-generated actions
├── config/
│   ├── robot_capabilities.yaml    # Available actions, preconditions
│   └── llm_config.yaml            # API keys, model selection
└── launch/
    └── vla_orchestrator.launch.py  # Launch LLM orchestrator + robot
```

**Key Files**:
- `llm_orchestrator.py`: Parses natural language → generates ROS 2 action goals
- `robot_capabilities.yaml`: Lists available actions (navigate_to_pose, move_to_cartesian_pose, open_gripper, etc.)

---

## CI/CD Workflows

### 1. Build + Unit Tests (`build-test.yml`)
**Trigger**: Every push to `main` or pull request

**Steps**:
1. Build ROS 2 workspace with `colcon build`
2. Run `colcon test` (unit tests for C++/Python nodes)
3. Check code coverage (target: >80%)
4. Upload build artifacts

**Runtime**: ~8 minutes

---

### 2. Integration Tests (`integration-test.yml`)
**Trigger**: Pull request to `main`

**Steps**:
1. Launch Gazebo with apartment world
2. Spawn humanoid robot
3. Run autonomous navigation test (5 waypoints, success = all reached within 5 min)
4. Launch Isaac Sim with Carter robot demo
5. Verify ROS 2 topics: `/cmd_vel`, `/odom`, `/scan`, `/camera/image_raw`, `/joint_states`
6. Shutdown gracefully

**Runtime**: ~15 minutes

**Acceptance Criteria**:
- Gazebo navigation test: 100% waypoint success rate
- Isaac Sim topics: All 5 topics active at 10+ Hz

---

### 3. Deploy Docs (`deploy-docs.yml`)
**Trigger**: Push to `main` branch

**Steps**:
1. Build Docusaurus site: `npm run build`
2. Run link checker: Verify 0 broken links
3. Deploy to GitHub Pages: `gh-pages` branch

**Runtime**: ~5 minutes

---

## Development Workflow

### Clone the Repository
```bash
git clone https://github.com/physical-ai/physical-robotics-ai-book.git
cd physical-robotics-ai-book
```

### Install Dependencies
```bash
# Install ROS 2 Humble (see ubuntu-ros2-setup.md)
sudo apt update
sudo apt install ros-humble-desktop-full

# Install Python dependencies
pip3 install -r requirements.txt

# Install Isaac ROS (see isaac-sim-installation.md)
```

### Build ROS 2 Workspace
```bash
cd src/
colcon build --symlink-install
source install/setup.bash
```

### Run Example: Gazebo Simulation
```bash
# Terminal 1: Launch Gazebo + robot
ros2 launch humanoid_gazebo gazebo.launch.py

# Terminal 2: Launch navigation stack
ros2 launch humanoid_control navigation.launch.py

# Terminal 3: Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}'
```

---

## Docker Development Environment

### Build Docker Image
```bash
cd docker/
docker build -t physical-ai-dev:latest -f Dockerfile.dev .
```

### Run Interactive Shell
```bash
docker run -it --rm \
  --gpus all \
  --network host \
  --env DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace \
  physical-ai-dev:latest \
  bash
```

### Inside Container
```bash
cd /workspace/src/
colcon build
source install/setup.bash
ros2 launch humanoid_gazebo gazebo.launch.py
```

---

## Module-Specific Directories

Each module (02-10) has its own subdirectory in `src/`:

- **Module 02**: `src/module02_first_humanoid/` (URDF creation, Gazebo spawn)
- **Module 03**: `src/module03_navigation/` (Nav2 configs, apartment world)
- **Module 04**: `src/module04_perception/` (Isaac ROS integration)
- **Module 05**: `src/module05_manipulation/` (MoveIt2, grasp planning)
- **Module 06**: `src/module06_sim_to_real/` (Domain randomization, Jetson deployment)
- **Module 07**: `src/module07_vla/` (LLM orchestration)
- **Module 08**: `src/module08_deployment/` (Jetson configs, Docker images)
- **Module 09**: `src/module09_advanced/` (Bipedal locomotion, dexterous hands)
- **Module 10**: `src/module10_capstone/` (Your final project)

---

## Best Practices

### Code Organization
- **One node per file**: `camera_driver.py`, `object_detector.py`, `path_planner.py`
- **Separate config from code**: YAML files in `config/`, not hardcoded in Python/C++
- **Launch files compose nodes**: Avoid monolithic launch files >200 lines

### Version Control
- **Branch naming**: `feature/module-03-navigation`, `fix/gazebo-crash`
- **Commit messages**: Conventional commits (`feat:`, `fix:`, `docs:`, `test:`)
- **Pull requests**: Link to module, include test results, request review

### Testing
- **Unit tests**: Test individual node logic (publishers, subscribers, services)
- **Integration tests**: Test multi-node systems (navigation stack, manipulation pipeline)
- **Simulation tests**: Verify behaviors in Gazebo/Isaac Sim before real hardware

---

## External Links

- [ROS 2 Workspace Structure Best Practices](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) | [Archive](https://archive.is/PLACEHOLDER_ROS2_WORKSPACE)
- [Colcon Build System Docs](https://colcon.readthedocs.io/) | [Archive](https://archive.is/PLACEHOLDER_COLCON_DOCS)
- [GitHub Actions for ROS 2](https://github.com/ros-tooling/action-ros-ci) | [Archive](https://archive.is/PLACEHOLDER_GH_ACTIONS_ROS)

---

**Previous**: [Four Pillars Architecture](./four-pillars-architecture.md) | **Next**: [Ubuntu + ROS 2 Setup](./ubuntu-ros2-setup.md)
