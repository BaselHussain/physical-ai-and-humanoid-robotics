# Implementation Plan: Module 02 — The Robotic Nervous System (ROS 2)

**Branch**: `002-ros2-mastery` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Capstone Repository**: `ros2-humanoid-labs` (separate GitHub repository)

## Summary

Module 02 delivers comprehensive ROS 2 mastery through 12-15 Docusaurus pages and 8 progressive hands-on labs. Learners build from basic publisher/subscriber patterns to a complete 5-DOF humanoid URDF visualized in RViz with working joint states and TF tree. The module includes Humble vs. Jazzy decision matrix, Mermaid sequence diagrams for action servers and navigation stack, complete colcon workspace template (separate capstone repository), and minimum 40 authoritative citations. All labs must pass `colcon build && colcon test` with zero warnings on Ubuntu 22.04.

## Technical Context

**Language/Version**: Python 3.10.12 (Ubuntu 22.04 default), C++ 17 (performance-critical nodes)

**Primary Dependencies**:
- **ROS 2 Humble Hawksbill** (LTS: May 2023 - May 2027) - DECIDED (see research.md)
- colcon-core 0.12.1 (Ubuntu 22.04 apt package)
- ament_cmake 1.3.3, ament_python 1.3.3
- rclpy 3.3.7, rclcpp 16.0.3 (Humble release versions)
- robot_state_publisher 3.0.2, joint_state_publisher_gui 2.3.0
- rviz2 11.2.5, tf2_ros 0.25.2
- xacro 2.0.8 (URDF preprocessing)
- geometry_msgs, sensor_msgs, trajectory_msgs (Humble versions)

**Storage**:
- URDF/xacro files (robot description XML)
- Python launch files (launch system configuration)
- YAML parameter files (node configuration)
- Mesh files (.dae, .stl for visual/collision geometry)
- RViz config files (.rviz for visualization presets)

**Testing**:
- pytest 6.2.5 + pytest-cov (Python node tests)
- gtest 1.11.0 + gmock (C++ node tests)
- launch_testing 1.0.4 (integration tests)
- colcon test (automated test runner)

**Target Platform**: Ubuntu 22.04 LTS (Jammy Jellyfish) - primary ROS 2 Humble platform

**Project Type**:
- Educational documentation (Docusaurus module)
- Capstone repository (separate `ros2-humanoid-labs` GitHub repo with 8 lab packages)

**Performance Goals**:
- Lab build time: <2 minutes for full workspace (first build)
- Lab incremental build: <10 seconds (after code changes)
- URDF loading in RViz: <3 seconds (humanoid with 7 links, 6 joints)
- Topic publish rate: ≥100 Hz for joint states
- Action server response: <100ms for simple goals

**Constraints**:
- Zero build warnings (`COLCON_LOG_LEVEL=warn colcon build`)
- Zero test failures (`colcon test` exit code 0)
- Final humanoid URDF: valid TF tree (no broken transforms, `ros2 run tf2_tools view_frames`)
- All code examples: copy-paste functional (no "..." placeholders)
- Maximum 3 external dependencies per lab package (beyond ROS 2 core)

**Scale/Scope**:
- **Documentation**: 12-15 Docusaurus pages in `docs/module-02-ros2-mastery/`
- **Labs**: 8 hands-on packages in separate `ros2-humanoid-labs` repository
- **Diagrams**: 3 Mermaid diagrams (action sequence, navigation stack, node graph)
- **Humanoid URDF**: 5-DOF minimum (torso base + 2 arms with shoulder/elbow/wrist joints)
- **Citations**: Minimum 40 references (ROS 2 papers, official docs, ICRA/IROS tutorials)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Educational Clarity
- **PASS**: Progressive lab structure clearly defined (8 labs from simple → complex)
- **PASS**: Explicit prerequisites (Module 01 complete, Ubuntu 22.04 + ROS 2 Humble)
- **PASS**: Complex topics scaffolded (topics → services → actions → URDF → TF2 → launch)
- **PASS**: Each lab has clear learning objective and deliverable

### ✅ II. Engineering Accuracy
- **PASS**: ROS 2 Humble is current LTS (verified May 2023 - May 2027 support)
- **PASS**: All patterns follow official ROS 2 design guidelines (docs.ros.org)
- **NEEDS RESEARCH**: Exact dependency versions (Phase 0)
- **NEEDS RESEARCH**: Humble vs. Jazzy trade-off justification (Phase 0)
- **NEEDS RESEARCH**: URDF best practices and humanoid conventions (Phase 0)

### ✅ III. Practical Applicability (NON-NEGOTIABLE)
- **PASS**: All 8 labs are executable with specified dependencies
- **PASS**: Humanoid URDF model is concrete, testable deliverable
- **PASS**: Every concept has hands-on implementation (no theory-only content)
- **PASS**: `colcon build && colcon test` requirement ensures executability

### ✅ IV. Spec-Driven Development
- **PENDING**: spec.md to be created after plan.md (current step)
- **PASS**: This plan.md follows Spec-Kit Plus template structure
- **PASS**: Changes tracked via git (002-ros2-mastery branch)

### ✅ V. Ethical Responsibility
- **PASS**: Safety guidelines for URDF modeling (collision geometry, joint limits)
- **PASS**: Testing protocols required (`colcon test` mandatory)
- **PASS**: Simulation-first approach (RViz before physical hardware)
- **PASS**: Risk mitigation via progressive complexity (simple before complex)

### ✅ VI. Reproducibility & Open Knowledge
- **PASS**: Dependency versions specified (Python 3.10.12, ROS 2 Humble, Ubuntu 22.04)
- **PASS**: Environment setup documented in Module 01
- **PASS**: Separate capstone repository allows independent cloning
- **PASS**: GitHub Pages deployment planned for documentation

### ✅ VII. Zero Broken State
- **PASS**: `colcon build && colcon test` requirement enforces zero failures
- **PASS**: Zero warnings constraint prevents code degradation
- **PASS**: GitHub Actions CI planned for automated validation
- **PASS**: TF tree validation requirement (`view_frames` must succeed)

### Summary
- **Critical Blockers**: None
- **Research Required** (Phase 0):
  1. Humble vs. Jazzy decision matrix
  2. Exact dependency versions
  3. URDF/TF2 best practices
  4. Citation collection (40+ references)
  5. colcon workspace best practices
- **Proceed to Phase 0**: ✅

## Project Structure

### Documentation (Docusaurus module)

```text
physical-robotics-ai-book/
docs/module-02-ros2-mastery/
├── index.md                          # [P1] ROS 2 overview + module roadmap
├── ros2-architecture.md              # [P2] DDS middleware, nodes, graphs
├── workspace-setup.md                # [P3] colcon workspace creation
├── lab01-talker-listener.md          # [P4] Basic pub/sub pattern
├── lab02-custom-messages.md          # [P5] Custom msg definitions
├── lab03-services.md                 # [P6] Request/response pattern
├── lab04-actions.md                  # [P7] Long-running goals with feedback
├── urdf-fundamentals.md              # [P8] URDF XML structure, links, joints
├── lab05-humanoid-urdf.md            # [P9] Building 5-DOF humanoid
├── tf2-coordinate-frames.md          # [P10] TF tree, static/dynamic transforms
├── lab06-tf2-broadcasting.md         # [P11] Publishing transforms
├── lab07-launch-files.md             # [P12] Python launch system
├── lab08-rviz-integration.md         # [P13] Full humanoid in RViz
├── humble-vs-jazzy.md                # [P14] ROS 2 version decision guide
├── debugging-ros2.md                 # [P15] rqt, ros2 cli, topic echo, bags
└── next-steps-references.md          # [P16] 40+ citations + Module 03 preview

static/diagrams/
├── ros2-action-sequence.mmd          # Mermaid: action client/server flow
├── ros2-nav-stack.mmd                # Mermaid: navigation2 architecture
└── ros2-node-graph.mmd               # Mermaid: example node/topic graph
```

### Capstone Repository (separate GitHub repo)

**Repository Name**: `ros2-humanoid-labs`
**Purpose**: Hands-on lab workspace for Module 02
**URL**: `https://github.com/physical-ai/ros2-humanoid-labs`

```text
ros2-humanoid-labs/
├── src/
│   ├── lab01_talker_listener/           # Lab 1: Basic pub/sub
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── resource/lab01_talker_listener
│   │   ├── test/
│   │   │   ├── test_copyright.py
│   │   │   ├── test_flake8.py
│   │   │   └── test_pep257.py
│   │   └── lab01_talker_listener/
│   │       ├── __init__.py
│   │       ├── talker.py                # Publisher node
│   │       └── listener.py              # Subscriber node
│   │
│   ├── lab02_custom_messages/           # Lab 2: Custom msg types
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── msg/
│   │   │   └── RobotState.msg           # Custom message definition
│   │   └── test/
│   │       └── test_msg_build.cpp       # Validate msg generation
│   │
│   ├── lab03_services/                  # Lab 3: Service pattern
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── srv/
│   │   │   └── SetJointAngle.srv        # Service definition
│   │   ├── test/
│   │   │   └── test_service_integration.py
│   │   └── lab03_services/
│   │       ├── __init__.py
│   │       ├── server.py                # Service server
│   │       └── client.py                # Service client
│   │
│   ├── lab04_actions/                   # Lab 4: Action pattern
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── action/
│   │   │   └── MoveArm.action           # Action definition
│   │   ├── src/
│   │   │   ├── action_server.cpp        # C++ action server
│   │   │   └── action_client.cpp        # C++ action client
│   │   └── test/
│   │       └── test_action_lifecycle.cpp
│   │
│   ├── lab05_humanoid_urdf/             # Lab 5: Humanoid URDF
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── urdf/
│   │   │   ├── humanoid.urdf.xacro      # Main xacro file
│   │   │   ├── torso.urdf.xacro         # Torso subassembly
│   │   │   ├── arm.urdf.xacro           # Arm subassembly (reusable)
│   │   │   └── materials.xacro          # Color definitions
│   │   ├── meshes/
│   │   │   ├── visual/
│   │   │   │   ├── torso.dae            # Visual mesh
│   │   │   │   ├── upper_arm.dae
│   │   │   │   ├── forearm.dae
│   │   │   │   └── hand.dae
│   │   │   └── collision/
│   │   │       ├── torso_collision.stl  # Simplified collision
│   │   │       └── arm_collision.stl
│   │   └── test/
│   │       └── test_urdf_valid.py       # Validate URDF with check_urdf
│   │
│   ├── lab06_tf2_transforms/            # Lab 6: TF2 broadcasting
│   │   ├── package.xml
│   │   ├── setup.py
│   │   ├── test/
│   │   │   └── test_tf_tree.py          # Verify TF tree structure
│   │   └── lab06_tf2_transforms/
│   │       ├── __init__.py
│   │       ├── static_broadcaster.py    # Publish static transforms
│   │       └── dynamic_broadcaster.py   # Publish dynamic transforms
│   │
│   ├── lab07_launch_files/              # Lab 7: Launch system
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── humanoid_bringup.launch.py  # Bring up all nodes
│   │   │   └── simulation.launch.py        # Simulation-specific launch
│   │   ├── config/
│   │   │   ├── robot_params.yaml           # Robot parameters
│   │   │   └── controller_config.yaml      # Controller settings
│   │   └── test/
│   │       └── test_launch_integration.py  # Launch test
│   │
│   └── lab08_rviz_integration/          # Lab 8: Full RViz integration
│       ├── package.xml
│       ├── CMakeLists.txt
│       ├── launch/
│       │   └── display.launch.py        # Launch RViz with robot
│       ├── rviz/
│       │   └── humanoid_config.rviz     # RViz configuration
│       ├── config/
│       │   └── joint_states.yaml        # Joint state publisher config
│       └── test/
│           └── test_rviz_integration.py # Headless RViz test
│
├── install/                              # Built packages (gitignored)
├── build/                                # Build artifacts (gitignored)
├── log/                                  # Build logs (gitignored)
│
├── .github/
│   └── workflows/
│       └── ros2-ci.yml                  # GitHub Actions CI
│
├── README.md                            # Workspace setup instructions
├── .gitignore                           # Ignore build/, install/, log/
└── LICENSE                              # MIT License
```

**Structure Decision**:
- **Documentation**: Single Docusaurus module (16 pages) for reading material
- **Labs**: Separate GitHub repository (`ros2-humanoid-labs`) for hands-on practice
- **Rationale**: Separation allows learners to clone the workspace independently, practice labs without documentation site overhead, and contribute lab improvements via separate PRs. Documentation site links to capstone repo in every lab page.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | No violations requiring justification | Constitution Check passed all gates |

---

## Phase 0: Research Plan (Autonomous Dispatch)

**Objective**: Resolve all NEEDS CLARIFICATION items from Technical Context and Constitution Check.

### R1: ROS 2 Version Decision (Humble vs. Jazzy)

**Task**: Create authoritative decision matrix justifying Humble selection

**Research Questions**:
1. What are the LTS timelines? (Humble: May 2023 - May 2027 vs. Jazzy: May 2024 - May 2026)
2. Which Ubuntu versions support each? (Humble: 22.04, Jazzy: 24.04)
3. What is Isaac Sim 2024.1.1 compatibility? (Humble: full support, Jazzy: experimental)
4. What is package ecosystem maturity? (Nav2, MoveIt2, Isaac ROS versions)
5. What are migration paths for learners starting with Humble?

**Sources**:
- REP 2000 (ROS 2 Releases and Target Platforms)
- NVIDIA Isaac Sim 2024.1.1 release notes
- Nav2 and MoveIt2 GitHub repositories (supported ROS 2 versions)

**Output**: Section in research.md with comparison table and decision rationale

### R2: Exact Dependency Versions

**Task**: Lock all dependency versions for reproducibility (Constitution VI requirement)

**Research Questions**:
1. What colcon version ships with Ubuntu 22.04 apt? (colcon-core 0.12.1)
2. What are ament_cmake/ament_python versions in Humble? (1.3.3)
3. What are rclpy/rclcpp API versions? (rclpy 3.3.7, rclcpp 16.0.3)
4. What are testing framework versions? (pytest 6.2.5, gtest 1.11.0, launch_testing 1.0.4)
5. What are robot_state_publisher, tf2_ros, rviz2 versions in Humble?

**Sources**:
- Ubuntu 22.04 package repositories (`apt show colcon-core`)
- ROS 2 Humble release metadata (ros/rosdistro GitHub)
- Official ROS 2 Humble documentation

**Output**: Dependency version table in research.md

### R3: Citation Collection (Minimum 40 References)

**Task**: Collect authoritative ROS 2 academic and technical references

**Research Questions**:
1. What are foundational ROS 2 papers? (DDS middleware, real-time performance, security)
2. What ICRA/IROS tutorials exist for ROS 2? (2020-2025 proceedings)
3. What are the official ROS 2 design REPs? (REP 2005, REP 2007, etc.)
4. What are highly-cited navigation/manipulation papers using ROS 2?
5. What community resources provide best practices? (ROS Discourse, robotics.stackexchange)

**Sources**:
- IEEE Xplore (ICRA, IROS,IROS proceedings)
- arXiv.org (robotics section, cs.RO)
- docs.ros.org/en/humble
- ROS Discourse (discourse.ros.org)
- GitHub (ros2/rclcpp, ros-planning/navigation2, ros-planning/moveit2)

**Output**: Annotated bibliography in research.md (40+ references, categorized by topic)

### R4: URDF and TF2 Best Practices

**Task**: Establish design patterns for humanoid robot modeling

**Research Questions**:
1. What are standard URDF joint naming conventions? (REP 120, humanoid conventions)
2. What are TF2 best practices for static vs. dynamic transforms?
3. What are collision geometry guidelines? (simplified vs. detailed meshes)
4. What are inertia calculation requirements for physics simulation?
5. What are joint limit conventions? (position, velocity, effort ranges)

**Sources**:
- REP 120 (Coordinate Frames for Humanoids)
- ROS 2 robot_state_publisher documentation
- TF2 design documentation (tf2_ros Wiki)
- Example humanoid URDFs (PR2, Valkyrie, Atlas on GitHub)

**Output**: URDF/TF2 design checklist in research.md

### R5: colcon Workspace Best Practices

**Task**: Define optimal workspace structure and build workflows

**Research Questions**:
1. What is recommended src/ organization? (by feature vs. by layer)
2. What are colcon build optimization flags? (--parallel-workers, --symlink-install, --event-handlers)
3. What are testing best practices? (unit vs. integration, launch_testing usage)
4. What are CI/CD patterns for ROS 2? (ros-tooling/setup-ros GitHub Action)
5. What are dependency management patterns? (rosdep usage, package.xml best practices)

**Sources**:
- colcon.readthedocs.io (official documentation)
- ROS 2 testing guide (docs.ros.org/en/humble/Tutorials/Intermediate/Testing)
- ros-tooling/setup-ros GitHub repository
- Example workspaces (moveit2, navigation2)

**Output**: Workspace setup checklist in research.md

---

## Phase 1: Design & Contracts (After research.md Complete)

### D1: Data Model (data-model.md)

**Entities to Define**:

#### 1. RobotState (Custom Message)

**Purpose**: Represent humanoid robot state for monitoring

**Fields**:
- `header` (std_msgs/Header): timestamp, frame_id
- `joint_positions` (float64[]): joint angles [rad]
- `joint_velocities` (float64[]): joint velocities [rad/s]
- `battery_level` (float32): battery % [0.0, 100.0]
- `status` (uint8): IDLE=0, MOVING=1, ERROR=2

**Validation**:
- joint_positions.length == joint_velocities.length
- 0.0 ≤ battery_level ≤ 100.0
- status ∈ {0, 1, 2}

#### 2. SetJointAngle (Service)

**Purpose**: Command single joint to target angle

**Request**:
- `joint_name` (string): joint ID
- `target_angle` (float64): desired angle [rad]
- `max_velocity` (float64): max velocity [rad/s]

**Response**:
- `success` (bool)
- `message` (string): error if !success
- `final_angle` (float64): actual reached angle

**Validation**:
- joint_name must exist in URDF
- target_angle within joint limits
- max_velocity > 0.0

#### 3. MoveArm (Action)

**Purpose**: Execute arm trajectory with feedback

**Goal**:
- `joint_trajectory` (trajectory_msgs/JointTrajectory)
- `tolerance` (float64): acceptable error [rad]

**Feedback**:
- `current_joint_positions` (float64[])
- `time_remaining` (duration)
- `progress` (float32): [0.0, 1.0]

**Result**:
- `success` (bool)
- `final_error` (float64)
- `execution_time` (duration)

**State Machine**:
```
IDLE → EXECUTING (goal received)
EXECUTING → SUCCEEDED (error < tolerance)
EXECUTING → ABORTED (error > tolerance)
EXECUTING → PREEMPTED (new goal)
```

#### 4. HumanoidURDF (Robot Description)

**Purpose**: 5-DOF humanoid model

**Links** (7 total):
- base_link (torso)
- left_shoulder_link, right_shoulder_link
- left_upper_arm, right_upper_arm
- left_forearm, right_forearm
- left_hand, right_hand

**Joints** (6 total):
- left_shoulder_pitch (revolute): [-π/2, π/2]
- left_elbow (revolute): [0, π]
- left_wrist_roll (continuous): [-π, π]
- right_shoulder_pitch, right_elbow, right_wrist_roll (symmetric)

**Sensors**:
- camera (sensor_msgs/Image): 640x480 RGB on head_link

**Collision Geometry**:
- Simplified boxes for torso/arms (performance)
- Detailed meshes for visualization only

### D2: API Contracts (contracts/ directory)

**Files to Generate**:
1. `contracts/messages.yaml` - RobotState message spec
2. `contracts/services.yaml` - SetJointAngle service spec
3. `contracts/actions.yaml` - MoveArm action spec
4. `contracts/urdf-schema.yaml` - Humanoid URDF structure

*(Full YAML schemas will be generated in Phase 1 execution)*

### D3: Quickstart (quickstart.md)

**30-Second Overview**:

> Module 02 teaches ROS 2 fundamentals through 8 progressive hands-on labs. Clone the `ros2-humanoid-labs` workspace, build with `colcon build`, and complete labs 1-8: basic pub/sub → custom messages → services → actions → URDF modeling → TF2 transforms → launch files → full RViz integration. Final deliverable: 5-DOF humanoid robot visualized in RViz with working joint states and valid TF tree. Requires Ubuntu 22.04 + ROS 2 Humble (Module 01 complete).

**Prerequisites**:
- ✅ Module 01 complete
- ✅ Ubuntu 22.04 LTS + ROS 2 Humble
- ✅ `verify-environment.sh` passes

**Learning Outcomes**:
1. Master ROS 2 communication patterns (topics, services, actions)
2. Write Python and C++ ROS 2 nodes
3. Design and validate URDF robot models
4. Publish and visualize TF coordinate frames
5. Create Python launch files for multi-node systems
6. Debug ROS 2 systems with CLI tools and RViz

**Time Estimate**: 10-14 hours (1.5-2 hours per lab)

---

## Next Steps

1. **✅ Phase 0 Complete**: plan.md created with all unknowns marked for research
2. **→ Next**: Generate research.md by executing research tasks R1-R5
3. **→ Then**: Create data-model.md and contracts/ (Phase 1)
4. **→ Then**: Update agent context with new technologies
5. **→ Finally**: Run `/sp.tasks` to break down into atomic implementation tasks

**Branch**: `002-ros2-mastery`
**Plan Path**: `D:/Q4 Hackathon/Project 1/specs/002-ros2-mastery/plan.md`
**Capstone Repo**: `ros2-humanoid-labs` (to be created on GitHub)
**Status**: ✅ Ready for Phase 0 research
