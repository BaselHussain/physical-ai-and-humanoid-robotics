---
id: 001
title: Isaac Robot Brain Spec
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: isaac-robot-brain
branch: 004-isaac-platform
user: user
command: /sp.specify
labels: [isaac,robotics,specification,nvidia]
links:
  spec: specs/004-isaac-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-isaac-platform/spec.md
 - specs/004-isaac-platform/checklists/requirements.md
tests:
 - spec validation completed
---

## Prompt

Module 04 — The AI-Robot Brain (NVIDIA Isaac™ Platform)

Target audience: Students who finished Module 03 (Gazebo sim) and want fast, practical NVIDIA tools for perception and sim-to-real

Focus: Use NVIDIA Isaac Sim 2025 + Isaac ROS for GPU-accelerated perception, VSLAM, navigation, and synthetic data generation on Jetson Orin

Success criteria:
- Reader can launch Isaac Sim + ROS 2 bridge with humanoid at ≥60 FPS in <10 minutes
- Reader can deploy Isaac ROS VSLAM + Nav2 on Jetson Orin Nano with live RealSense D435i (map built in < 30 sec)
- Reader can generate 5,000 labeled synthetic images in < 30 min
- All examples run on Ubuntu 22.04 with RTX 4070 Ti + Jetson

Strict structure — deliver EXACTLY these 4 Markdown files only:
1. docs/module-04/intro.md                         → short intro + learning goals (max 4 pages)
2. docs/module-04/isaac-sim-basics.md           → Isaac Sim 2025 setup, ROS 2 bridge, humanoid loading
3. docs/module-04/perception-and-vslam.md       → Isaac ROS gems, cuVSLAM, DetectNet, people tracking on Jetson
4. docs/module-04/nav2-and-synthetic-data.md    → Nav2 for bipedal, synthetic data generation, sim-to-real tips

Constraints:
- Total length: 40–50 formatted pages maximum
- No extra files, no summary, no appendix
- All assets CC0 and <50 MB total
- Use existing reusable skills (isaac-perception-tuner, urdf-builder) automatically
- Dockerfiles for workstation and Jetson required

Not building:
- Training foundation models from scratch
- Non-NVIDIA alternatives
- Cloud-only setups

Go. Work in 004-isaac-platform branch and inside specs/004-isaac-platform

## Response snapshot

Specification created for Isaac Robot Brain (NVIDIA Isaac™ Platform) feature in the 004-isaac-platform branch. The spec includes user scenarios for launching Isaac Sim with humanoid at ≥60 FPS, deploying VSLAM and Nav2 on Jetson, and generating synthetic training data. Contains 10 functional requirements, key entities, and 7 measurable success criteria. Quality checklist created and all validation items completed.

## Outcome

- ✅ Impact: Created complete specification for Isaac Robot Brain module
- 🧪 Tests: spec validation completed successfully
- 📁 Files: specs/004-isaac-platform/spec.md, specs/004-isaac-platform/checklists/requirements.md
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification addresses all requirements from user description with measurable outcomes

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin planning phase