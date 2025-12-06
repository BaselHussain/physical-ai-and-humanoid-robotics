---
id: 002
title: Isaac Robot Brain Plan
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: isaac-robot-brain
branch: 004-isaac-platform
user: user
command: /sp.plan
labels: [isaac,robotics,plan,nvidia,ros2]
links:
  spec: specs/004-isaac-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-isaac-platform/plan.md
 - specs/004-isaac-platform/research.md
 - specs/004-isaac-platform/data-model.md
 - specs/004-isaac-platform/quickstart.md
 - specs/004-isaac-platform/contracts/ros2_interfaces.yaml
 - CLAUDE.md
tests:
 - constitution gates validated
 - agent context updated
---

## Prompt

Module 04 — The AI-Robot Brain (NVIDIA Isaac™ Platform)

Follow the newly merged spec exactly:

Deliver exactly these 4 Markdown files only:
1. docs/module-04/intro.md                         → short intro + learning goals (max 4 pages)
2. docs/module-04/01-isaac-sim-basics.md           → Isaac Sim 2025 setup, ROS 2 bridge, humanoid loading
3. docs/module-04/02-perception-and-vslam.md       → Isaac ROS gems, cuVSLAM, DetectNet, people tracking on Jetson
4. docs/module-04/03-nav2-and-synthetic-data.md    → Nav2 for bipedal, synthetic data generation, sim-to-real tips

Plan details:
- Total length: 40–50 formatted pages maximum
- Use existing reusable skills automatically (isaac-perception-tuner, urdf-builder)
- All assets CC0, <50 MB total, ≥60 FPS on RTX 4070 Ti
- Include minimal Dockerfiles + one complete synthetic dataset script
- Mermaid diagram for Isaac Sim ↔ ROS 2 ↔ Jetson data flow
- Decision table: Isaac Sim 2025.x vs. 2024.x (choose 2025)
- Testing strategy: VSLAM map build <30 sec on Jetson; synthetic gen benchmark 5,000 images <30 min

Output diagrams in static/diagrams/

Go.

## Response snapshot

Planning completed for Isaac Robot Brain module. Created implementation plan with technical context, constitution check, project structure, research summary, data models, quickstart guide, and ROS 2 interface contracts. Updated agent context with Isaac Sim technologies. All constitution gates passed successfully.

## Outcome

- ✅ Impact: Complete planning phase with all required artifacts
- 🧪 Tests: All constitution gates validated successfully
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts/ros2_interfaces.yaml, CLAUDE.md updated
- 🔁 Next prompts: Ready for task generation with /sp.tasks
- 🧠 Reflection: Planning addresses all requirements with appropriate technical decisions

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks with /sp.tasks