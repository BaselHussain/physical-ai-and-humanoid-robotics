# Feature Specification: Module 04 — Vision-Language-Action & The Autonomous Humanoid (Capstone)

**Feature Branch**: `005-vla-capstone`
**Created**: 2025-12-04
**Status**: Draft
**Input**: User description: "Module 04 — Vision-Language-Action & The Autonomous Humanoid (Capstone) - Target audience: Students who completed Modules 1–3. Focus: Close the loop — natural language → planning → physical execution"

## Clarifications

### Session 2025-12-04

- Q: How should the VLM communicate object locations to the LLM planner? → A: ROS 2 messages published to topics (vision_msgs/Detection3D)
- Q: What retry strategy should be used when a skill execution fails (e.g., grasp_object fails)? → A: 5 retries with fixed 2-second delay between attempts, then fail and replan
- Q: How should the LLM planner communicate skill execution commands to the robot? → A: ROS 2 action calls with goal/feedback/result messages (asynchronous with progress monitoring)
- Q: What should the benchmark test suite composition be for measuring end-to-end success rate (SC-001)? → A: 50 commands total with systematic variations (object colors, locations, room combinations)
- Q: What specific ROS 2 message structure should be used for VLM object localization output? → A: vision_msgs/Detection3D (existing ROS 2 vision message with pose, bbox, class, confidence)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Voice-to-Action Pipeline Integration (Priority: P1)

A learner wants to build a complete voice-to-action pipeline where a humanoid robot receives natural language commands (e.g., "Bring me the red cup from the kitchen"), processes them through an LLM for task planning, and executes the physical actions autonomously in Isaac Sim.

**Why this priority**: Voice-to-action integration is the core capability that closes the loop from human intent to robot execution. Without this working end-to-end, learners cannot demonstrate autonomous humanoid capabilities.

**Independent Test**: Learner implements voice recognition (Whisper), LLM task planning (GPT-4o API or LLaVA), and action execution pipeline. Robot receives command "Bring me the red cup from the kitchen," generates a plan, navigates to kitchen, locates cup, grasps it, and returns to user location in Isaac Sim.

**Acceptance Scenarios**:

1. **Given** voice input system configured with Whisper, **When** learner speaks "Bring me the red cup from the kitchen," **Then** system transcribes command accurately (≥95% word accuracy) and passes to LLM planner
2. **Given** LLM planner receives transcribed command, **When** processing request, **Then** planner generates valid action sequence: [navigate_to(kitchen), locate_object(red_cup), grasp_object(red_cup), navigate_to(user), release_object()]
3. **Given** action sequence generated, **When** robot executes plan in Isaac Sim, **Then** each action completes successfully with state updates to LLM for replanning if needed
4. **Given** end-to-end pipeline active, **When** learner runs 50-command benchmark suite with systematic variations (object colors, locations, room combinations), **Then** system achieves ≥80% task success rate in unseen apartment scenes

---

### User Story 2 - Vision-Language Model Integration for Scene Understanding (Priority: P1)

A learner wants to integrate vision-language models (VLMs) for real-time scene understanding, enabling the robot to identify objects, understand spatial relationships, and answer questions about its environment using camera input from Isaac Sim.

**Why this priority**: Scene understanding is critical for grounding natural language commands in the physical world. Without VLM integration, the robot cannot locate "the red cup" or understand "on the kitchen counter."

**Independent Test**: Learner integrates LLaVA or similar open-source VLM with Isaac Sim camera feeds. Robot receives query "Where is the red cup?" and responds with location description. Robot uses VLM output to guide navigation and grasping actions.

**Acceptance Scenarios**:

1. **Given** VLM integrated with Isaac Sim RGB camera, **When** robot captures scene image, **Then** VLM identifies all objects with ≥90% accuracy for common household items
2. **Given** spatial reasoning query "Where is the red cup?", **When** VLM processes image, **Then** model returns location description (e.g., "on kitchen counter, left side") accurate within 0.5m
3. **Given** multiple similar objects in scene, **When** command specifies attributes ("red cup" vs "blue cup"), **Then** VLM correctly discriminates target object with ≥85% accuracy
4. **Given** VLM running on Isaac Sim workstation, **When** processing scene understanding requests, **Then** inference latency is ≤2 seconds per query for interactive performance

---

### User Story 3 - Autonomous Navigation and Manipulation Skills (Priority: P1)

A learner wants to implement a skill library for autonomous navigation (path planning, obstacle avoidance) and manipulation (grasping, placing, releasing) that can be composed by the LLM planner to execute complex tasks.

**Why this priority**: Reusable skills are the building blocks for task execution. Without robust navigation and manipulation primitives, the LLM's plans cannot be reliably executed in the physical simulation.

**Independent Test**: Learner implements skill library with at least 6 core skills: navigate_to(location), locate_object(description), approach_object(object), grasp_object(object), place_object(location), release_object(). Each skill has success/failure detection and can be called by LLM planner.

**Acceptance Scenarios**:

1. **Given** skill library implemented, **When** LLM calls navigate_to("kitchen"), **Then** robot uses Nav2 + Isaac ROS VSLAM to reach kitchen with ≥90% success rate and <5% collision rate
2. **Given** locate_object skill, **When** called with locate_object("red cup"), **Then** robot uses VLM + depth camera to identify object 3D position with ≤10cm error
3. **Given** grasping skill, **When** robot executes grasp_object("red cup"), **Then** grasp succeeds (object secured in gripper) with ≥75% success rate across varied poses
4. **Given** skill execution failure, **When** skill returns failure state, **Then** LLM receives failure message and generates alternative plan (retry with different approach or abort)

---

### User Story 4 - Complete Capstone Project Deployment (Priority: P1)

A learner wants to deploy the complete capstone project as a single public GitHub repository with clear documentation, architecture diagrams, and instructions for running the end-to-end demo in Isaac Sim with optional Jetson deployment guidance.

**Why this priority**: The capstone must be a portfolio-ready project that demonstrates mastery of the entire stack. A well-structured, reproducible repository is essential for learners to showcase their skills and for others to learn from the implementation.

**Independent Test**: Learner follows provided template to organize codebase (<2000 lines original code), creates architecture diagram showing all components (voice, VLM, LLM, skills, Isaac Sim), writes README with setup and demo instructions, and publishes to GitHub. Repository includes Docker setup and works on fresh Isaac Sim installation.

**Acceptance Scenarios**:

1. **Given** complete codebase, **When** learner measures original code (excluding ROS 2 boilerplate, Isaac Sim SDK), **Then** total lines of original application code is <2000 lines
2. **Given** architecture diagram requirement, **When** learner creates system diagram, **Then** diagram shows all major components (voice input, LLM planner, VLM, skill library, Isaac Sim interface, ROS 2 topics) with data flow arrows
3. **Given** GitHub repository with README, **When** new user follows setup instructions on RTX 4070 Ti+ system, **Then** user can run demo "Bring me the red cup" within 30 minutes of initial clone
4. **Given** optional Jetson deployment section, **When** learner includes Jetson Orin instructions, **Then** instructions cover model optimization (quantization, TensorRT), compute distribution (workstation for VLM, Jetson for navigation), and expected performance degradation

---

### Edge Cases

- What happens when voice recognition fails or produces ambiguous transcription? (Include confidence thresholding and clarification dialogue: "Did you mean X or Y?")
- How does system handle LLM hallucinations or impossible plans (e.g., "fly to the ceiling")? (Implement skill precondition checking and graceful failure with user feedback)
- What if object is not found after navigation to expected location? (Include search behavior pattern and replanning logic: expand search radius, ask user for help)
- How does system handle VLM failures or incorrect object identification? (Provide fallback to depth-based heuristics and uncertainty quantification in VLM outputs)
- What happens when grasp fails multiple times? (Implement 5 retries with fixed 2-second delay between attempts and pose variation, then fail to LLM for replanning or ultimate fallback to user notification: "I cannot reach the object")

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Module MUST teach end-to-end VLA (Vision-Language-Action) pipeline architecture connecting natural language input to physical robot actions
- **FR-002**: Module MUST demonstrate voice input integration using Whisper or equivalent open-source speech-to-text model
- **FR-003**: Module MUST teach LLM integration for task planning using GPT-4o API (free tier) or open-source alternatives (LLaVA, Llama 3)
- **FR-004**: Module MUST demonstrate vision-language model (VLM) integration for scene understanding and object localization in Isaac Sim
- **FR-005**: Module MUST teach skill library design with at least 6 reusable skills (navigate_to, locate_object, approach_object, grasp_object, place_object, release_object) exposed as ROS 2 action servers with goal/feedback/result interfaces
- **FR-006**: Module MUST demonstrate integration of all components into single end-to-end demo: voice command → LLM planning → skill execution in Isaac Sim
- **FR-007**: Capstone project MUST be delivered as single public GitHub repository with <2000 lines of original application code (excluding ROS 2/Isaac SDK boilerplate)
- **FR-008**: Module MUST include complete architecture diagram showing all major components (voice, LLM, VLM, skills, Isaac Sim) and data flow
- **FR-009**: Module MUST provide template project structure with Docker setup for reproducibility on RTX 4070 Ti+ systems
- **FR-010**: Module MUST teach error handling and replanning strategies when skills fail or objects are not found, including skill-level retry logic (5 retries with fixed 2-second delay between attempts) before escalating to LLM for replanning
- **FR-011**: Module MUST include optional Jetson Orin deployment instructions with model optimization guidance (quantization, TensorRT, compute distribution)
- **FR-012**: All examples MUST use open-source or free-tier models only (no paid API dependencies beyond free limits)
- **FR-013**: Module MUST teach evaluation methodology for measuring end-to-end success rate using 50-command benchmark suite with systematic variations (object colors, locations, room combinations) across unseen apartment scenes

### Key Entities

- **VLA Pipeline**: Represents complete system integrating voice input, language model planning, vision-language scene understanding, and physical action execution
- **LLM Task Planner**: Represents language model component that converts natural language commands into structured action sequences using skill library
- **Vision-Language Model (VLM)**: Represents multimodal AI component that processes camera images and language queries to identify objects, understand spatial relationships, and answer scene questions
- **Skill Library**: Represents collection of reusable robotic capabilities (navigation, manipulation) with standardized interfaces callable by LLM planner
- **Capstone Repository**: Represents complete GitHub project including source code, documentation, architecture diagrams, Docker setup, and demo instructions

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 80% of learners achieve ≥80% task success rate on benchmark test suite of 50 commands with systematic variations (object colors, locations, room combinations covering "fetch X", "place X on Y", "move to Z" patterns) in unseen Isaac Sim apartment scenes (validated via automated test harness)
- **SC-002**: 90% of learners successfully integrate voice input (Whisper) with ≥95% transcription accuracy on standard robotics command dataset (validated via WER metric)
- **SC-003**: 85% of learners successfully integrate VLM (LLaVA or equivalent) achieving ≥90% object identification accuracy and ≤10cm localization error (validated via labeled test scenes)
- **SC-004**: 100% of learner capstone repositories have <2000 lines of original application code, meeting code budget constraint (validated via automated line counting excluding dependencies)
- **SC-005**: 100% of learner repositories include complete architecture diagram showing all 5+ major components with data flow (validated via rubric checklist)
- **SC-006**: 75% of learners complete full end-to-end demo ("Bring me the red cup from the kitchen") running in Isaac Sim within first attempt after following module instructions (validated via project submission)
- **SC-007**: Module length is between 90-120 formatted pages (validated via Docusaurus page count)
- **SC-008**: 70% of learners successfully implement error handling with at least 3 failure recovery strategies (retry, replan, user notification) (validated via code review and demonstration)
- **SC-009**: 50% of learners who attempt optional Jetson deployment successfully run navigation + manipulation on Jetson Orin with VLM inference on workstation (validated via video demonstration)
- **SC-010**: All learner repositories successfully deploy on fresh RTX 4070 Ti+ system from git clone to running demo in ≤30 minutes (validated via reproducibility testing)

## Assumptions *(optional)*

- **Prerequisites**: Assumes completion of Modules 01 (ROS 2), 02 (Gazebo/Unity), and 03 (NVIDIA Isaac Platform) for full technical foundation
- **Workstation Hardware**: Assumes RTX 4070 Ti, 4080, or 4090 GPU with ≥12GB VRAM for Isaac Sim + VLM inference; 32GB system RAM recommended
- **Jetson Hardware (Optional)**: Assumes Jetson Orin Nano (8GB) or Orin NX (16GB) for optional edge deployment testing
- **Isaac Sim**: Assumes Isaac Sim 2024.x or 2025.x installed with ROS 2 Humble bridge operational
- **API Access**: Assumes learners have OpenAI API key with free-tier GPT-4o access, or are willing to use fully open-source alternatives (LLaVA, Llama 3)
- **Camera Hardware (Optional)**: Assumes Intel RealSense D435/D455 for optional real-world testing beyond Isaac Sim
- **Network**: Assumes stable internet for model downloads (Whisper, LLaVA weights multi-GB), API calls to LLM services if used
- **Python Proficiency**: Assumes intermediate Python skills (async programming, ROS 2 Python, basic ML inference) from previous modules
- **GitHub Account**: Assumes learners have GitHub account for repository publication

## Out of Scope *(optional)*

- Training custom VLMs or LLMs from scratch (inference and fine-tuning only)
- Advanced reinforcement learning for manipulation (skill library uses scripted/heuristic grasping)
- Multi-robot coordination or fleet management (single humanoid focus)
- Real humanoid hardware deployment beyond Isaac Sim (optional Jetson guidance only)
- Custom speech synthesis (TTS) for robot responses (focus on action execution, not dialogue)
- Advanced computer vision pipelines beyond VLM (no custom CNN training, segmentation models)
- Web/mobile app interfaces (command-line or simple GUI only)
- Cloud deployment or distributed systems architecture (single workstation + optional Jetson only)
- Safety certification or regulatory compliance for physical robots
- Advanced motion planning beyond Nav2 defaults (no trajectory optimization, whole-body control)

## Dependencies *(optional)*

- **External**: NVIDIA Isaac Sim 2024.x/2025.x with ROS 2 Humble bridge on workstation
- **External**: OpenAI API key with GPT-4o free-tier access, OR Hugging Face account for open-source model downloads
- **External**: Whisper model weights (openai/whisper-large-v3 or distilled variant)
- **External**: LLaVA or equivalent VLM weights (if using open-source path)
- **External**: Docker Desktop for containerized development environment
- **External**: Jetson Orin with JetPack 6.0+ and Isaac ROS (optional for edge deployment)
- **Internal**: Module 01 completion for ROS 2 workspace management and Python rclpy proficiency
- **Internal**: Module 02 completion for simulation fundamentals, URDF modeling, and environment creation
- **Internal**: Module 03 completion for Isaac Sim proficiency, Isaac ROS perception, Nav2 navigation, and Jetson deployment basics
- **Internal**: Provided apartment scene assets from Module 02 for testing environments
- **Internal**: Provided humanoid robot URDF/USD from Module 02 for Isaac Sim simulation
- **Internal**: Project constitution principles (practical applicability, reproducibility, ethical responsibility)
- **Internal**: ROS 2 vision_msgs package for VLM object detection output (vision_msgs/Detection3D including pose, bounding box, class, confidence)
- **Internal**: ROS 2 action interfaces for skill execution (action_msgs, custom action definitions for each skill with goal/feedback/result structures)

## Constraints *(optional)*

- Module length strictly limited to 90-120 formatted Docusaurus pages
- Capstone codebase must be <2000 lines of original application code (excluding ROS 2 boilerplate, Isaac Sim SDK, third-party libraries)
- All models must be open-source or free-tier only (no paid API dependencies beyond free limits)
- Complete architecture diagram required showing all major components and data flow
- Single public GitHub repository deliverable with reproducible setup
- End-to-end demo must achieve ≥80% success rate in unseen Isaac Sim apartment scenes
- Voice transcription must achieve ≥95% accuracy on robotics command dataset
- VLM object identification must achieve ≥90% accuracy with ≤10cm localization error
- Complete demo setup time must be ≤30 minutes from git clone on target hardware
- All examples must work with ROS 2 Humble and Isaac Sim 2024.x/2025.x
- Optional Jetson deployment must run on Jetson Orin Nano (8GB) minimum with workstation offload for VLM inference
- Skill library must include minimum 6 core skills (navigate, locate, approach, grasp, place, release)
