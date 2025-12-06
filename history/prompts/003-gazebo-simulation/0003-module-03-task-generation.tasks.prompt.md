---
id: 0003
title: Module 03 Task Generation
stage: tasks
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 003-gazebo-simulation
branch: 003-gazebo-simulation
user: developer
command: /sp.tasks
labels: ["tasks", "gazebo", "simulation", "unity", "documentation"]
links:
  spec: specs/003-gazebo-simulation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-gazebo-simulation/tasks.md
tests:
  - Final validation test (T015) covers FPS, walking, sensors, Unity connection, asset budget, link checking
---

## Prompt

Module 03 — The Digital Twin (Gazebo & Unity)

Follow the newly merged spec and plan exactly.

Break into 4 phases with tiny atomic tasks (15–30 min each).  
Total tasks: 14–16 max.  
All output goes into exactly these 4 files only:

1. docs/module-03/intro.md (max 4 pages)
2. docs/module-03/01-gazebo-fundamentals.md (10–12 pages)
3. docs/module-03/02-sensors-and-humanoid.md (10–12 pages)
4. docs/module-03/03-apartment-worlds-and-unity.md (10–12 pages + final apartment world)

Phase 1 → Intro & Setup (3 tasks)
Phase 2 → Gazebo + URDF→SDF + Launch (4 tasks)
Phase 3 → Sensors + Humanoid in Sim (4 tasks)
Phase 4 → Apartment World + Unity Bridge + Final Validation (4 tasks)

Every task must:
- Use existing reusable skills automatically (urdf-builder, gazebo-world-builder)
- End with a working launch file or asset <50 MB total
- Be testable in <2 minutes on RTX 4070 Ti

Final acceptance criteria for whole module:
- All 4 files exist and total ≤50 formatted pages
- Final apartment scene runs ≥60 FPS
- Humanoid walks 10 m without falling (5/5 runs)
- Unity visualization connects via TCP in <5 seconds

Deliver the full task list as 003-gazebo-simulation/TASKS.md with exact task name, duration, dependency, acceptance, and output file.

Go.

## Response snapshot

Generated 15 atomic tasks (3+4+4+4) organized across 4 phases for Module 03 implementation.

**Phase 1 - Intro & Setup (3 tasks, 45-90 min)**:
- T001: Create intro.md with overview, prerequisites, learning goals (≤4 pages)
- T002: Generate Gazebo↔ROS 2↔Unity dataflow Mermaid diagram
- T003: Generate sensor integration + apartment world structure Mermaid diagrams

**Phase 2 - Gazebo Fundamentals (4 tasks, 1-2 hours)**:
- T004: Gazebo Harmonic installation + decision table (Harmonic vs Classic)
- T005: URDF→SDF workflow (automatic conversion, <gazebo> tags)
- T006: Gazebo plugins (physics DART, sensors, scene broadcaster)
- T007: Launch files (Python example, complete 01-gazebo-fundamentals.md at 10-12 pages)

**Phase 3 - Sensors + Humanoid (4 tasks, 1-2 hours)**:
- T008: Humanoid URDF (urdf-builder skill) + depth camera (RealSense D435)
- T009: LiDAR (16-layer GPU) + IMU (MEMS with bias modeling)
- T010: Contact sensors (feet) + integration/testing guide (complete 02-sensors-and-humanoid.md at 10-12 pages)
- T011: Generate example humanoid URDF with all 4 sensors (urdf-builder skill, optional asset)

**Phase 4 - Apartment + Unity + Validation (4 tasks, 1-2 hours)**:
- T012: Apartment world design + asset sourcing + gazebo-world-builder skill usage
- T013: Unity setup (2022.3 LTS) + bridge configuration (ros_gz_bridge + TCP)
- T014: System integration + generate apartment SDF (gazebo-world-builder, complete 03-apartment-worlds-and-unity.md at 10-12 pages)
- T015: Final validation (all acceptance criteria: page count 40-50, FPS ≥60, walking test, Unity <5s connection, assets <50MB, zero broken links)

**Task Format**: All 15 tasks follow strict checklist format `- [ ] T### [P] [US#] Description with file path`
- 4 parallelizable tasks ([P]): T001, T002, T003, T011
- User story mapping: US1 (11 tasks), US2 (2 tasks), US3 (2 tasks), Validation (1 task)
- Dependencies visualized in Mermaid graph

**MVP Scope**: T001-T011 (User Story 1: Launch Sensor-Equipped Humanoid, 11 tasks, ~4.5-5.5 hours)

**Performance Budget**: Tracked per component (intro ≤4 pages, chapters 10-12 pages each, assets <50MB), validated in T015

## Outcome

- ✅ Impact: Complete task breakdown for Module 03 implementation with 15 atomic, testable tasks
- 🧪 Tests: Comprehensive final validation (T015) covering FPS ≥60, walking 10m × 5 runs, sensor rates, Unity latency <5s, asset budget, link checking
- 📁 Files: Created tasks.md with 15 tasks across 4 phases
- 🔁 Next: Execute tasks sequentially (T001→T015) or run Phase 1 (T001-T003) in parallel
- 🧠 Reflection: All tasks atomic (15-30 min), skills integrated (urdf-builder T011, gazebo-world-builder T014), acceptance criteria specific and testable

## Evaluation

- Failure modes: None
- Graders: Task count = 15 (within 14-16 target), all tasks have checklist format ✓
- Variant: Standard /sp.tasks workflow
- Next experiment: Execute Phase 1 tasks in parallel
