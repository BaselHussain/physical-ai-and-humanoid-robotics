---
title: Reusable Intelligence Infrastructure
sidebar_label: Reusable Intelligence
sidebar_position: 1
description: Overview of Claude Code skills and orchestrator agent for Physical AI & Humanoid Robotics development
---

# Reusable Intelligence Infrastructure

## Overview

This book leverages **Claude Code skills**—specialized, reusable AI agents that accelerate Physical AI and humanoid robotics development. Each skill is an expert in a specific domain (URDF creation, simulation design, perception tuning, LLM orchestration, system validation) and can be invoked on-demand throughout your learning journey.

Think of skills as **intelligent power tools** that remember best practices, catch common errors, and guide you through complex workflows—allowing you to focus on learning concepts rather than debugging syntax errors or configuration issues.

---

## The Five Core Skills

### 1. URDF Builder
**Domain**: Robot Modeling & Description
**Invocation**: `/skill urdf-builder`

Creates and validates humanoid robot URDF/Xacro files with correct physics, joint configurations, and sensor integrations.

**What it handles**:
- 12-32 DoF humanoid kinematics
- Inertia tensor calculation (no more "robot explodes in Gazebo")
- ros2_control integration (position/velocity/effort controllers)
- Sensor integration (cameras, LiDAR, IMU, force/torque sensors)
- Gazebo plugin declarations
- URDF validation and error detection

**Example use cases**:
- Module 02: Create your first humanoid URDF
- Module 05: Add bimanual manipulation arms
- Module 08: Integrate RealSense camera for perception

**Output**: Production-ready URDF/Xacro files, controller configs, launch files

---

### 2. Gazebo World Builder
**Domain**: Simulation Environment Design
**Invocation**: `/skill gazebo-world-builder`

Builds realistic indoor environments (apartments, offices, warehouses) with furniture, lighting, and domain randomization for sim-to-real transfer.

**What it handles**:
- Floor plans and room layouts
- Furniture and obstacle placement
- Lighting configuration (natural + artificial)
- Physics tuning (friction, restitution, contact parameters)
- Domain randomization (object poses, lighting, textures)
- Navigation-compatible worlds (Nav2 costmap integration)

**Example use cases**:
- Module 03: Build apartment for navigation testing
- Module 06: Create manipulation testbed with tables and shelves
- Module 09: Design domain randomization for sim-to-real policy transfer

**Output**: Gazebo SDF world files, launch files, domain randomization scripts

---

### 3. Isaac Perception Tuner
**Domain**: GPU-Accelerated Perception
**Invocation**: `/skill isaac-perception-tuner`

Configures and tunes Isaac ROS perception packages (cuVSLAM, DetectNet, depth processing) for optimal performance on Jetson hardware with RealSense cameras.

**What it handles**:
- cuVSLAM (GPU-accelerated visual SLAM) tuning
- DetectNet object detection confidence thresholds
- Depth image filtering and point cloud generation
- TensorRT inference optimization
- Latency and accuracy trade-offs for real-time robotics
- Perception quality benchmarking

**Example use cases**:
- Module 04: Configure cuVSLAM for indoor localization
- Module 07: Tune DetectNet for object detection in manipulation tasks
- Module 10: Optimize perception pipeline for Jetson Orin Nano deployment

**Output**: Isaac ROS launch files, config YAMLs, benchmarking scripts, validation reports

---

### 4. VLA Orchestrator
**Domain**: Natural Language Robot Control
**Invocation**: `/skill vla-orchestrator`

Builds LLM-powered orchestration layers that translate natural language commands ("go to the kitchen and pick up the red mug") into executable ROS 2 action sequences.

**What it handles**:
- LLM integration (GPT-4, Claude, local LLaMA models)
- Prompt engineering for task decomposition
- ROS 2 action client execution (Nav2, MoveIt2, gripper control)
- Natural language status updates and error explanations
- Failure handling and re-planning
- Safety guardrails and constraint checking

**Example use cases**:
- Module 11: Add voice control to your humanoid robot
- Module 13: Build conversational HRI layer for household tasks
- Module 15: Integrate Vision-Language-Action (VLA) models for embodied AI

**Output**: ROS 2 orchestrator nodes, LLM prompt templates, action execution pipelines, test suites

---

### 5. Robot Validator
**Domain**: System Diagnostics & Testing
**Invocation**: `/skill robot-validator`

Autonomous diagnostic agent that validates robot system health, detects configuration errors, and generates actionable repair reports.

**What it handles**:
- URDF integrity checks (joint limits, inertia, collision meshes)
- TF tree validation (missing transforms, loops, disconnected frames)
- Sensor data quality (publication rates, resolution, noise levels)
- Action server availability and responsiveness
- CI/CD integration (GitHub Actions, GitLab CI)
- Pre-flight validation before deployment

**Example use cases**:
- Every module: Validate your robot configuration before running demos
- Module 14: Set up CI/CD for continuous integration testing
- Debugging: "My camera isn't publishing—run diagnostics"

**Output**: Validation reports (JSON), GitHub Actions workflows, error fix recommendations

---

## The Orchestrator Agent

**Role**: Intelligent skill dispatcher and workflow coordinator

The orchestrator agent sits above the 5 skills and automatically routes your requests to the correct skill(s) based on keyword detection and intent analysis.

### How It Works

**Single-Skill Routing**:
```
You: "Create a URDF for a 20-DoF humanoid with RealSense camera"
Orchestrator: Detects keywords [URDF, humanoid, RealSense] → routes to urdf-builder
urdf-builder: Guides you through URDF creation workflow
```

**Multi-Skill Workflows**:
```
You: "Set up my robot from scratch"
Orchestrator: Detects "complete_robot_setup" workflow → invokes skills sequentially:
  1. urdf-builder (create robot model)
  2. gazebo-world-builder (create simulation environment)
  3. isaac-perception-tuner (configure perception)
  4. robot-validator (validate complete system)
```

### Routing Keywords

| Skill | Triggers |
|-------|----------|
| **urdf-builder** | urdf, xacro, robot description, humanoid model, joint, link, inertia |
| **gazebo-world-builder** | gazebo, world, environment, apartment, office, furniture, domain randomization |
| **isaac-perception-tuner** | isaac ros, perception, cuVSLAM, detectnet, jetson, realsense, confidence threshold |
| **vla-orchestrator** | natural language, llm, gpt, task planning, voice command, robot brain |
| **robot-validator** | validate, diagnostic, health check, debug, test, ci/cd, tf tree |

### Pre-Defined Workflows

**1. Complete Robot Setup** (End-to-End)
- Steps: URDF → Simulation → Perception → Validation
- Trigger: "Set up my robot from scratch"

**2. Sim-to-Real Pipeline**
- Steps: Gazebo world (with domain randomization) → Perception tuning → Validation on real hardware
- Trigger: "Deploy from simulation to real robot"

**3. Intelligent Manipulation**
- Steps: Manipulator URDF → LLM control → Object detection → Validation
- Trigger: "Build manipulation system with natural language control"

---

## Why Reusable Intelligence Matters

### Traditional Approach (Without Skills)
```
Developer workflow:
1. Google "how to create URDF inertia"
2. Copy-paste Stack Overflow code (may be outdated)
3. Robot explodes in Gazebo
4. Debug for 2 hours
5. Find typo in inertia tensor
6. Repeat for every new robot project
```

**Time to first working robot**: 8-12 hours (with debugging)

### Skills-Powered Approach
```
Developer workflow:
1. Invoke: /skill urdf-builder
2. Answer guided questions (DoF count, sensor types)
3. Skill generates validated URDF with correct physics
4. Launch in Gazebo → works immediately
```

**Time to first working robot**: 15-30 minutes

### Key Benefits

1. **Error Prevention**: Skills encode years of best practices and catch common mistakes before they happen
2. **Consistency**: Every URDF follows the same conventions, making code review and maintenance easier
3. **Learning Acceleration**: Spend time understanding concepts, not debugging syntax
4. **Scalability**: Create 10 different humanoids as easily as creating 1
5. **CI/CD Ready**: robot-validator integrates directly into your deployment pipeline

---

## Integration with Book Modules

Each module in this book is designed to leverage skills at specific points:

| Module | Primary Skills Used |
|--------|---------------------|
| **Module 01**: Introduction | *robot-validator* (verify installation) |
| **Module 02**: First Humanoid | *urdf-builder* → *gazebo-world-builder* → *robot-validator* |
| **Module 03**: Navigation | *gazebo-world-builder* (apartment) → *isaac-perception-tuner* (cuVSLAM) |
| **Module 04**: Perception | *isaac-perception-tuner* (DetectNet, depth processing) |
| **Module 05**: Manipulation | *urdf-builder* (add arms) → *gazebo-world-builder* (manipulation testbed) |
| **Module 06**: Grasping | *isaac-perception-tuner* (object detection) → *vla-orchestrator* (pick/place) |
| **Module 07**: Mobile Manipulation | All skills (complete system integration) |
| **Module 08**: Sim-to-Real | *gazebo-world-builder* (domain randomization) → *isaac-perception-tuner* (real hardware) |
| **Module 09**: Natural Language Control | *vla-orchestrator* (LLM brain) |
| **Module 10**: Deployment | *robot-validator* (CI/CD, pre-flight checks) |

---

## Invoking Skills

### Via Claude Code CLI

```bash
# Invoke a single skill
/skill urdf-builder

# The skill will guide you through an interactive workflow:
# 1. Ask clarifying questions
# 2. Generate code/configs
# 3. Validate outputs
# 4. Provide next steps
```

### Via Natural Language (Orchestrator Handles Routing)

```bash
# Let the orchestrator detect the right skill
"Create a URDF for my humanoid robot"
→ Orchestrator routes to /skill urdf-builder

"Build an apartment simulation"
→ Orchestrator routes to /skill gazebo-world-builder

"Tune Isaac ROS perception for Jetson"
→ Orchestrator routes to /skill isaac-perception-tuner
```

### Via Explicit Workflows

```bash
# Invoke a pre-defined multi-skill workflow
"Run complete robot setup workflow"
→ Orchestrator executes: urdf-builder → gazebo-world-builder → isaac-perception-tuner → robot-validator
```

---

## Skill Architecture

Each skill follows a consistent structure:

```markdown
# Skill Name

## When to Use This Skill
- Trigger conditions and use cases

## How It Works
- Step-by-step workflow (typically 6-9 steps)
- User interactions and clarifying questions
- Deliverables at each step

## Output Format
- Exact files generated (with paths)
- Code examples and templates

## Quality Criteria
- Validation checklist
- Success metrics

## Full Example
- Complete end-to-end example with inputs and outputs
```

**Skill Files Location**: `.claude/skills/<skill-name>/SKILL.md`

**Orchestrator Config**: `.claude/agents/orchestrator_agent/description.yaml`

---

## Extending the Intelligence

As you progress through this book, you can:

1. **Customize Skills**: Edit SKILL.md files to match your team's conventions
2. **Add New Skills**: Create skills for custom domains (e.g., legged locomotion, dexterous hands)
3. **Tune Orchestrator**: Adjust routing rules in `description.yaml` for your workflow
4. **Share Skills**: Publish skills to your team's skill library for reuse across projects

---

## Next Steps

- **New to Claude Code Skills?** → Read [Claude Code Documentation](https://docs.anthropic.com/claude-code)
- **Ready to start?** → Continue to [Module 01: Introduction & Physical AI Foundations](../module-01-physical-ai-intro/index.md)
- **Want to dive into a specific skill?** → Explore `.claude/skills/` directory

---

## Skill Summary Table

| Skill | Domain | Primary Output | Typical Invocation |
|-------|--------|----------------|-------------------|
| **urdf-builder** | Robot modeling | URDF/Xacro files, ros2_control configs | "Create humanoid URDF" |
| **gazebo-world-builder** | Simulation environments | Gazebo SDF worlds, domain randomization | "Build apartment world" |
| **isaac-perception-tuner** | GPU perception | Isaac ROS configs, benchmarks | "Tune cuVSLAM for Jetson" |
| **vla-orchestrator** | Natural language control | LLM orchestrator nodes, action pipelines | "Add voice control" |
| **robot-validator** | System diagnostics | Validation reports, CI/CD workflows | "Validate my robot" |

---

**Status**: 5 production-ready skills + 1 orchestrator agent
**Total Lines of Code**: ~8,000 lines (SKILL.md + examples)
**Estimated Time Savings**: 60-80% reduction in setup/debugging time per module
**Maintainability**: Skills are versioned and can be updated independently

Ready to accelerate your Physical AI development? Let's begin! 🚀
