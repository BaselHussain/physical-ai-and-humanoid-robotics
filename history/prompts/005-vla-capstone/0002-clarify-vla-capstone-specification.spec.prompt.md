---
id: 0002
title: Clarify VLA capstone specification
stage: spec
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: 005-vla-capstone
branch: 005-vla-capstone
user: user
command: /sp.clarify
labels: ["clarification", "vla", "ros2", "integration", "benchmark", "vision-msgs", "actions", "error-handling"]
links:
  spec: specs/005-vla-capstone/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/005-vla-capstone/spec.md
tests:
  - N/A (specification clarification)
---

## Prompt

```text
/sp.clarify
```

## Response snapshot

Conducted structured ambiguity scan and resolved 5 high-impact clarifications for Module 04 VLA Capstone specification through interactive questioning. All clarifications integrated incrementally into spec.md.

**Clarifications Resolved (5 of 5)**:

1. **VLM-LLM Communication**: Q: How should the VLM communicate object locations to the LLM planner? → A: ROS 2 messages published to topics (vision_msgs/Detection3D)

2. **Skill Retry Strategy**: Q: What retry strategy should be used when a skill execution fails? → A: 5 retries with fixed 2-second delay between attempts, then fail and replan

3. **LLM-Robot Interface**: Q: How should the LLM planner communicate skill execution commands to the robot? → A: ROS 2 action calls with goal/feedback/result messages (asynchronous with progress monitoring)

4. **Benchmark Composition**: Q: What should the benchmark test suite composition be for measuring end-to-end success rate (SC-001)? → A: 50 commands total with systematic variations (object colors, locations, room combinations)

5. **VLM Message Structure**: Q: What specific ROS 2 message structure should be used for VLM object localization output? → A: vision_msgs/Detection3D (existing ROS 2 vision message with pose, bbox, class, confidence)

**Specification Updates**:
- Created "Clarifications > Session 2025-12-04" section with all 5 Q&A entries
- Updated User Story 1, Acceptance Scenario 4: Changed from "10 varied commands" to "50-command benchmark suite"
- Updated Edge Cases: Grasp failure now specifies "5 retries with fixed 2-second delay"
- Updated FR-005: Skills exposed as "ROS 2 action servers with goal/feedback/result interfaces"
- Updated FR-010: Added explicit retry logic "(5 retries with fixed 2-second delay between attempts) before escalating to LLM"
- Updated FR-013: Evaluation methodology now specifies "50-command benchmark suite with systematic variations"
- Updated SC-001: Success rate measured on "50 commands with systematic variations" instead of "10 varied commands"
- Updated Dependencies: Added "ROS 2 vision_msgs package for VLM object detection output" and "ROS 2 action interfaces for skill execution"

**Coverage Summary**: All 9 taxonomy categories now resolved or already clear. No outstanding or deferred ambiguities.

## Outcome

- ✅ Impact: Resolved 5 high-impact implementation ambiguities in Module 04 VLA Capstone spec, establishing concrete ROS 2 architecture (action servers, vision_msgs), error handling parameters (5×2s retry), and evaluation methodology (50-command benchmark)
- 🧪 Tests: Specification clarification complete - all 5 questions answered and integrated
- 📁 Files: specs/005-vla-capstone/spec.md (updated with Clarifications section and propagated changes)
- 🔁 Next prompts: Run `/sp.plan` to create implementation plan with ROS 2 action server architecture, vision_msgs integration, skill retry logic, and 50-command benchmark harness design
- 🧠 Reflection: Clarifications establish concrete ROS 2 patterns: vision_msgs/Detection3D for VLM output enables standard perception message format, action servers for skills provide async execution with progress monitoring (industry best practice), 5×2s retry balances robustness with time constraints, 50-command benchmark provides statistical validity (vs original 10 commands). All choices align with ROS 2 best practices and enable unambiguous planning phase.

## Evaluation notes (flywheel)

- Failure modes observed: None. All 5 questions answered successfully. User selected options C (vision_msgs/Detection3D) and C (50 commands) over recommendations, demonstrating preference for comprehensive standard messages and robust evaluation over minimal viable options.
- Graders run and results (PASS/FAIL): PASS - All 5 clarifications integrated correctly across 6 spec sections (Clarifications, User Stories, Edge Cases, FRs, Success Criteria, Dependencies). No contradictions or unresolved placeholders.
- Prompt variant (if applicable): Standard /sp.clarify execution with 5 questions asked (quota reached)
- Next experiment (smallest change to try): Monitor whether vision_msgs/Detection3D provides sufficient information for LLM planning without requiring additional custom fields. If LLM struggles with Detection3D format during implementation, consider bridge node that converts Detection3D → natural language summaries for LLM consumption. Also validate 50-command benchmark is achievable within module time budget (may require automated scene generation and parallelized testing).
