# Implementation Plan: Module 03 — The Digital Twin (Gazebo & Unity)

**Branch**: `003-gazebo-simulation` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)

## Summary

Module 03 delivers educational content for building realistic digital twins of humanoid robots using Gazebo Harmonic and optional Unity visualization. Targets students who completed Module 01 (ROS 2).

**Primary Requirement**: Create exactly 4 Markdown files (40-50 pages total) teaching students to launch a sensor-equipped humanoid in Gazebo within 10 minutes, build apartment environments, configure sensors, and optionally connect Unity.

**Technical Approach**: Use existing urdf-builder and gazebo-world-builder skills, leverage Gazebo Harmonic with DART physics, implement ros_gz_bridge + Unity ROS TCP Connector, optimize for ≥60 FPS on RTX 4070 Ti, provide CC0-licensed assets <50 MB.

## Technical Context

**Language/Version**: Python 3.10+, Bash 5.x
**Primary Dependencies**: Gazebo Harmonic, ROS 2 Jazzy, ros_gz, Docusaurus v3.x, Unity 2022.3 LTS (optional)
**Storage**: Git repository, static assets (Markdown, COLLADA, YAML), GitHub Pages
**Testing**: Manual validation (FPS, sensor checks), automated link checking
**Target Platform**: Ubuntu 24.04 (Noble), native (not WSL2)
**Project Type**: Documentation project (Docusaurus) + simulation assets
**Performance Goals**: ≥60 FPS, RTF ≥1.0, <10 min startup, Unity latency <30ms depth images
**Constraints**: 40-50 pages, <50 MB assets, CC0/permissive licenses only, no C++ plugins
**Scale/Scope**: 4 Markdown chapters, 1 apartment world, 1 humanoid URDF, 3-5 Mermaid diagrams

## Constitution Check

*All 7 principles: PASS* (See detailed evidence in complete plan document)

## Project Structure

```text
specs/003-gazebo-simulation/
├── spec.md, plan.md, research.md, data-model.md, quickstart.md
└── contracts/ (bridge_config.yaml, unity_ros_settings.json, sensor_specifications.yaml)

docs/module-02/
├── intro.md (4 pages)
├── 01-gazebo-fundamentals.md (~15 pages)
├── 02-sensors-and-humanoid.md (~15 pages)
└── 03-apartment-worlds-and-unity.md (~15 pages)

static/diagrams/ (3 Mermaid diagrams)
```

## Phase 0: Research (✅ COMPLETE)

See `research.md` for:
- Gazebo Harmonic vs Classic (Harmonic chosen)
- URDF→SDF workflow (automatic via ros_gz_sim)
- Sensor specs (RealSense D435, 16-layer LiDAR, MEMS IMU)
- Bridge architecture (ros_gz_bridge + Unity TCP, 17x faster than ROSBridge)
- Performance optimization (mesh decimation, shadows off, DART, GPU LiDAR)
- Asset sources (Free3D, Open3dModel, AWS RoboMaker, Gazebo DB)

## Phase 1: Design & Contracts (✅ COMPLETE)

### Deliverables
1. **data-model.md** - Simulation entities, sensor configs, test scenarios
2. **contracts/bridge_config.yaml** - 10 ros_gz_bridge topic mappings
3. **contracts/unity_ros_settings.json** - Unity ROS TCP Connector config
4. **contracts/sensor_specifications.yaml** - Full sensor specs with noise models
5. **quickstart.md** - <10 min launch guide

### Performance Budget

| Component | Visual Polygons | Collision Polygons | Update Rate | RTF Impact |
|-----------|----------------|-------------------|-------------|------------|
| Humanoid | 30,000 | 3,000 | 1kHz physics | 0.3 |
| Depth Camera | - | - | 30 Hz | 0.1 |
| LiDAR | - | - | 10 Hz | 0.05 |
| IMU | - | - | 100 Hz | 0.01 |
| Contacts | - | - | 100 Hz | 0.02 |
| Apartment | 150,000 | 20,000 | Static | 0.1 |
| Lighting | - | - | - | 0.05 |
| **Total RTF** | | | | **0.63** |
| **Headroom** | | | | **1.6x** |

## Implementation Architecture

### 1. `docs/module-02/intro.md` (4 pages)
- Module overview, prerequisites, learning goals
- Hardware requirements, success criteria

### 2. `docs/module-02/01-gazebo-fundamentals.md` (~15 pages)
- Gazebo Harmonic installation
- Decision table: Harmonic vs Classic
- URDF→SDF workflow
- Plugins and systems
- Launch files

### 3. `docs/module-02/02-sensors-and-humanoid.md` (~15 pages)
- Humanoid URDF (12-32 DoF, urdf-builder skill)
- Depth camera (RealSense D435, 30Hz, σ=0.007)
- LiDAR (16-layer, 10Hz, σ=0.01m)
- IMU (100Hz, angular σ=0.009rad/s)
- Contact sensors (feet, 100Hz)
- Integration and testing

### 4. `docs/module-02/03-apartment-worlds-and-unity.md` (~15 pages)
- Apartment world design (<200k visual, <30k collision)
- Asset sourcing (CC0) and Blender decimation
- World building with gazebo-world-builder skill
- Unity setup (2022.3 LTS, ROS TCP Connector)
- Bridge configuration (ros_gz_bridge + ros_tcp_endpoint)
- Complete system integration and testing

### Mermaid Diagrams
1. Gazebo ↔ ROS 2 ↔ Unity data flow
2. Sensor integration architecture
3. Apartment world component hierarchy

### Testing Strategy
1. **FPS Test**: ≥60 FPS average over 60s
2. **Walking Test**: 10m straight, 5 runs, 0 falls
3. **Sensor Test**: All topics publishing at expected rates
4. **Unity Latency Test**: Joint states <5ms, depth <30ms

## Risks & Mitigations

1. **Performance <60 FPS**: Test early, provide decimation pipeline, document tuning
2. **Asset Licensing**: Verify all licenses (CC0/permissive), legal review
3. **Unity Complexity**: Make optional, provide Gazebo-only workflow first
4. **Page Count >50**: Strict budget (4+15+15+15=49), concise writing
5. **Startup >10 min**: Test on fresh Ubuntu, optimize world loading

## Follow-up TODOs

1. `/sp.tasks` to generate implementation tasks
2. Source CC0 furniture, run Blender decimation
3. Write 4 Markdown chapters (49 pages total)
4. Create configs (launch file, RViz, Unity scripts)
5. Run all tests (FPS, walking, sensors, Unity latency)
6. Performance tuning (gz stats, decimation if needed)
7. Documentation review (page count, clarity, accuracy)
8. Consider ADRs: Gazebo Harmonic, Unity bridge, DART physics
9. Deploy to GitHub Pages, verify zero broken links
10. Community enablement (issues, good-first-issue tags)

---

**Status**: Phase 1 COMPLETE. Ready for `/sp.tasks`.
