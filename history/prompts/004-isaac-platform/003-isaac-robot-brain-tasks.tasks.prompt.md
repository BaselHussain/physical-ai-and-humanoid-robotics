---
id: 003
title: Isaac Robot Brain Tasks
stage: tasks
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: isaac-robot-brain
branch: 004-isaac-platform
user: user
command: /sp.tasks
labels: [isaac,robotics,tasks,nvidia,ros2]
links:
  spec: specs/004-isaac-platform/spec.md
  plan: specs/004-isaac-platform/plan.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-isaac-platform/tasks.md
tests:
 - task format validation
 - dependency mapping
---

## Prompt

Module 04 — The AI-Robot Brain (NVIDIA Isaac™ Platform)

Follow the newly merged spec and plan exactly.

Break into 4 phases with tiny atomic tasks (15–30 min each).
Total tasks: 14–16 max.
All output goes into exactly these 4 files only:

1. docs/module-04/intro.md                         (max 4 pages)
2. docs/module-04/isaac-sim-basics.md           (10–12 pages)
3. docs/module-04/perception-and-vslam.md       (10–12 pages)
4. docs/module-04/nav2-and-synthetic-data.md    (10–12 pages + final Dockerfiles)

Phase 1 → Intro & Setup (3 tasks)
Phase 2 → Isaac Sim + ROS 2 Bridge (4 tasks)
Phase 3 → Perception Gems + Jetson Deployment (4 tasks)
Phase 4 → Nav2, Synthetic Data & Final Validation (4 tasks)

Every task must:
- Use existing reusable skills automatically (isaac-perception-tuner, urdf-builder)
- End with a working Dockerfile or script <50 MB total
- Be testable in <2 minutes on RTX 4070 Ti + Jetson

Final acceptance criteria for whole module:
- All 4 files exist and total ≤50 formatted pages
- Isaac Sim + ROS 2 bridge ≥60 FPS with humanoid
- VSLAM on Jetson builds map <30 sec
- Synthetic data gen 5,000 images <30 min

Deliver the full task list as specs/003-isaac-platform/tasks.md with exact task name, duration, dependency, acceptance, and output file.

Go.

## Response snapshot

Created comprehensive task list for Isaac Robot Brain module with 15 tasks across 4 phases. Tasks follow strict checklist format with proper IDs, parallelization markers [P], and user story labels [US1/US2/US3]. Phase 1: Intro & Setup (3 tasks). Phase 2: Isaac Sim + ROS 2 Bridge (4 tasks) targeting ≥60 FPS with humanoid. Phase 3: Perception Gems + Jetson Deployment (4 tasks) for <30 sec VSLAM map building. Phase 4: Nav2, Synthetic Data & Final Validation (4 tasks) for 5,000 images in <30 min. Includes Dockerfiles and validation of all acceptance criteria.

## Outcome

- ✅ Impact: Complete task breakdown enabling incremental development
- 🧪 Tests: Task format and dependency validation completed
- 📁 Files: specs/004-isaac-platform/tasks.md created
- 🔁 Next prompts: Ready for implementation with individual tasks
- 🧠 Reflection: Tasks are atomic (15-30 min), testable, and properly sequenced

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin task implementation