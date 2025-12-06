# Data Model: Module 03 — The Digital Twin (Gazebo & Unity)

**Feature**: 003-gazebo-simulation
**Date**: 2025-12-06
**Purpose**: Define simulation entities, configurations, and data structures

## Overview

This module focuses on educational content delivery rather than software system data models. The "data" entities are primarily:
1. Documentation files (Markdown)
2. Configuration files (YAML, XML/SDF)
3. Asset files (COLLADA meshes, textures)
4. Launch files (Python)

## Core Entities

### 1. Documentation Module

**Entity**: Educational Module
**Type**: Markdown documentation structure
**Purpose**: Deliver learning content in Docusaurus-compatible format

**Structure**:
```yaml
Module:
  name: "Module 03 — The Digital Twin (Gazebo & Unity)"
  path: "docs/module-02/"
  files:
    - intro.md
    - 01-gazebo-fundamentals.md
    - 02-sensors-and-humanoid.md
    - 03-apartment-worlds-and-unity.md

  metadata:
    total_pages: 40-50
    learning_goals:
      - Launch sensor-equipped humanoid in <10 minutes
      - Build realistic apartment environment
      - Configure depth camera, LiDAR, IMU, contact sensors
      - Connect Unity visualization (optional)
    prerequisites:
      - Module 01 (ROS 2 basics)
      - Ubuntu 24.04
      - ROS 2 Jazzy installed
```

**Validation Rules**:
- Total page count: 40-50 formatted pages
- Exactly 4 Markdown files
- All code examples must be executable
- All diagrams must be in static/diagrams/

### 2. Humanoid Robot Model

**Entity**: Humanoid Robot URDF/SDF
**Type**: Robot description with sensor suite
**Purpose**: Fully sensor-equipped bipedal robot for simulation

**Attributes**:
```yaml
HumanoidRobot:
  name: "humanoid_bot"
  format: URDF (with <gazebo> extensions)

  physical_properties:
    degrees_of_freedom: 12-32
    total_mass: 40-80 kg
    height: 1.4-1.8 m

  meshes:
    visual_polygon_budget: 20000-50000 triangles
    collision_polygon_budget: 2000-5000 triangles
    format: COLLADA (.dae)

  sensors:
    - depth_camera:
        type: depth
        resolution: 848x480
        fov: 85.2 degrees
        range: 0.3-10 m
        update_rate: 30 Hz
        noise: gaussian (mean=0.0, stddev=0.007)

    - lidar:
        type: gpu_lidar
        horizontal_samples: 1024
        vertical_samples: 16
        range: 0.1-30 m
        update_rate: 10 Hz
        noise: gaussian (mean=0.0, stddev=0.01)

    - imu:
        type: imu
        update_rate: 100 Hz
        angular_velocity_noise:
          stddev: 0.009 rad/s
          bias_mean: 0.00075
          bias_stddev: 0.005
        linear_acceleration_noise:
          stddev: 0.017 m/s²
          bias_mean: 0.1
          bias_stddev: 0.001

    - contact_sensors:
        type: contact
        locations: [left_foot, right_foot]
        update_rate: 100 Hz

  joints:
    type: revolute (most), continuous (some)
    limits:
      position: [-π, π] (varies by joint)
      velocity: 10 rad/s (typical)
      effort: 100 Nm (typical)
    damping: 0.1-1.0 (varies by joint)
    friction: 0.1-0.5 (varies by joint)
```

**Relationships**:
- Uses existing urdf-builder skill for generation
- Spawned in Gazebo world via ros_gz_sim
- Publishes to /robot_description topic

**Validation Rules**:
- All sensors must have realistic noise models
- Collision meshes must be < 500 triangles per link
- Visual meshes must be < 5000 triangles per link
- Total robot collision budget: < 5000 triangles
- Must include inertial properties for all links

### 3. Apartment World Environment

**Entity**: Indoor Simulation Environment
**Type**: Gazebo SDF world file
**Purpose**: Realistic indoor environment for navigation and manipulation

**Attributes**:
```yaml
ApartmentWorld:
  name: "apartment_world"
  format: SDF 1.9

  structure:
    rooms:
      - living_room:
          dimensions: 5m x 4m
          furniture: [sofa, coffee_table, tv_stand, bookshelf]
      - kitchen:
          dimensions: 4m x 3m
          furniture: [counter, sink, refrigerator, table, chairs]
      - bedroom:
          dimensions: 4m x 3m
          furniture: [bed, nightstand, dresser]

  meshes:
    total_visual_polygons: 100000-200000 triangles
    total_collision_polygons: 10000-30000 triangles

  furniture_objects:
    - table:
        visual_polygons: 200-1000
        collision_polygons: 50-100
    - chair:
        visual_polygons: 300-1500
        collision_polygons: 100-300
    - sofa:
        visual_polygons: 500-2000
        collision_polygons: 150-400
    - counter:
        visual_polygons: 500-1500
        collision_polygons: 100-300

  lighting:
    shadows_enabled: false
    directional_lights: 1-2
    point_lights: 0-4 (task lighting only)
    ambient: [0.4, 0.4, 0.4, 1.0]

  physics:
    engine: DART
    max_step_size: 0.001
    real_time_update_rate: 1000
    collision_detector: bullet

  asset_sources:
    - Free3D (COLLADA furniture)
    - Open3dModel (COLLADA furniture)
    - AWS RoboMaker Small House World
    - Gazebo Model Database

  licensing:
    required: CC0, CC-BY, Apache 2.0
    total_size: < 50 MB
```

**Relationships**:
- Uses existing gazebo-world-builder skill for generation
- Contains HumanoidRobot spawn location
- Provides collision environment for navigation

**Validation Rules**:
- Total scene polygon budget: <200k visual, <30k collision
- All assets must be CC0 or permissively licensed
- Total asset size < 50 MB
- Must run ≥60 FPS on RTX 4070 Ti

### 4. Sensor Configuration

**Entity**: Sensor Suite Configuration
**Type**: SDF sensor definitions with noise models
**Purpose**: Realistic sensor simulation for perception algorithms

**Attributes**:
```yaml
SensorConfiguration:
  depth_camera:
    hardware_model: Intel RealSense D435
    resolution: 848x480
    fov_horizontal: 85.2 degrees
    fov_vertical: 58 degrees
    range:
      min: 0.195 m
      max: 10.0 m
    update_rate: 30 Hz
    noise_model:
      type: gaussian
      mean: 0.0
      stddev: 0.007
    output_topic: /depth_camera/image

  lidar:
    type: 3D LiDAR (16 layers)
    horizontal:
      samples: 1024
      fov: 360 degrees
    vertical:
      samples: 16
      fov: 30 degrees (-15° to +15°)
    range:
      min: 0.1 m
      max: 30.0 m
      resolution: 0.01 m
    update_rate: 10 Hz
    noise_model:
      type: gaussian
      mean: 0.0
      stddev: 0.01
    output_topic: /lidar/points

  imu:
    update_rate: 100 Hz
    angular_velocity:
      noise_stddev: 0.009 rad/s
      bias_mean: 0.00075 rad/s
      bias_stddev: 0.005 rad/s
      dynamic_bias_correlation_time: 400.0 s
    linear_acceleration:
      noise_stddev: 0.017 m/s²
      bias_mean: 0.1 m/s²
      bias_stddev: 0.001 m/s²
      dynamic_bias_correlation_time: 300.0 s
    output_topic: /imu/data

  contact_sensors:
    locations: [left_foot, right_foot]
    update_rate: 100 Hz
    collision_targets: [ground, furniture, walls]
    output_topics: [/contact/left_foot, /contact/right_foot]
```

**Validation Rules**:
- All noise parameters must match real-world sensor characteristics
- Update rates must be achievable at ≥60 FPS overall
- Sensor topics must follow ROS 2 naming conventions

### 5. Gazebo-Unity Bridge Configuration

**Entity**: ROS 2 Bridge Configuration
**Type**: YAML configuration for ros_gz_bridge + Unity TCP Connector
**Purpose**: Real-time data streaming from Gazebo to Unity

**Attributes**:
```yaml
BridgeConfiguration:
  ros_gz_bridge:
    config_file: bridge_config.yaml
    topics:
      - name: /robot/joint_states
        ros_type: sensor_msgs/msg/JointState
        gz_type: gz.msgs.Model
        qos_profile: SENSOR_DATA
        throttle_rate: 60 Hz

      - name: /robot/depth_camera/image
        ros_type: sensor_msgs/msg/Image
        gz_type: gz.msgs.Image
        qos_profile: SENSOR_DATA
        throttle_rate: 30 Hz
        compress: true

      - name: /robot/lidar/points
        ros_type: sensor_msgs/msg/PointCloud2
        gz_type: gz.msgs.PointCloudPacked
        qos_profile: SENSOR_DATA
        throttle_rate: 10 Hz
        compress: true

      - name: /robot/odometry
        ros_type: nav_msgs/msg/Odometry
        gz_type: gz.msgs.Odometry
        qos_profile: SENSOR_DATA
        throttle_rate: 60 Hz

  unity_tcp_connector:
    ros_ip: 127.0.0.1
    ros_port: 10000
    subscribed_topics:
      - /robot/joint_states
      - /robot/depth_camera/image
      - /robot/lidar/points
      - /robot/odometry

  performance_expectations:
    joint_states_latency: < 5 ms
    depth_image_latency: 15-30 ms
    point_cloud_latency: 30-50 ms
    total_bandwidth: < 50 Mbps
    cpu_overhead: 5-10%
```

**Relationships**:
- Connects Gazebo Harmonic to ROS 2 topics
- Unity ROS TCP Connector subscribes to ROS 2 topics
- ros_tcp_endpoint runs on ROS 2 side

**Validation Rules**:
- All topic mappings must be valid ROS 2 ↔ Gazebo message types
- QoS profiles must be appropriate for data type
- Throttle rates must not exceed sensor update rates
- Total bandwidth must be < 50 Mbps

### 6. Launch Configuration

**Entity**: ROS 2 Launch File
**Type**: Python launch file
**Purpose**: Single-command startup for complete simulation

**Attributes**:
```yaml
LaunchConfiguration:
  name: apartment_simulation.launch.py
  language: Python

  nodes:
    - robot_state_publisher:
        parameters:
          robot_description: from URDF file
          use_sim_time: true

    - gz_sim:
        world_file: apartment_world.sdf
        arguments: [-r]  # Run on start

    - ros_gz_bridge:
        config_file: bridge_config.yaml

    - rviz2 (optional):
        config_file: simulation.rviz

  parameters:
    use_sim_time: true
    physics_update_rate: 1000 Hz

  environment:
    GZ_SIM_RESOURCE_PATH: model directories

  startup_time_target: < 10 minutes (from installation to running)
```

**Validation Rules**:
- Must launch all required nodes in correct order
- Must set use_sim_time=true for all nodes
- Must provide clear console output for debugging
- Startup time must be < 10 minutes from fresh installation

## Documentation Structure

### 7. Chapter Files

**Entity**: Markdown Chapter
**Type**: Educational content file
**Purpose**: Deliver specific learning content

**Attributes**:
```yaml
Chapter:
  format: Markdown (CommonMark)
  front_matter:
    title: string
    sidebar_position: integer

  sections:
    - title: string
      content: markdown text
      code_blocks:
        - language: bash | python | xml | yaml
          content: executable code
          validation: must be runnable
      diagrams:
        - type: mermaid | image
          source: inline mermaid | path to static/diagrams/

  constraints:
    max_length:
      intro.md: 4 pages
      01-gazebo-fundamentals.md: 15 pages
      02-sensors-and-humanoid.md: 15 pages
      03-apartment-worlds-and-unity.md: 15 pages
    total_module_length: 40-50 pages

  validation:
    - All code blocks must be tested
    - All links must be valid
    - All diagrams must render
    - All examples must use CC0 assets
```

### 8. Mermaid Diagrams

**Entity**: Architecture Diagram
**Type**: Mermaid diagram (embedded in Markdown)
**Purpose**: Visualize Gazebo ↔ ROS 2 ↔ Unity data flow

**Example Structure**:
```yaml
DataFlowDiagram:
  type: mermaid flowchart
  content: |
    graph LR
      A[Gazebo Harmonic<br/>Physics + Sensors] -->|gz topics| B[ros_gz_bridge]
      B -->|ROS 2 topics| C[ROS 2 Middleware]
      C -->|/joint_states<br/>/depth_camera<br/>/lidar| D[ros_tcp_endpoint]
      D -->|TCP port 10000| E[Unity ROS TCP Connector]
      E --> F[Unity 3D<br/>Visualization]

  location: docs/module-02/01-gazebo-fundamentals.md or static/diagrams/
```

### 9. Decision Table

**Entity**: Technical Decision Matrix
**Type**: Markdown table
**Purpose**: Compare Gazebo Harmonic vs Classic

**Structure**:
```yaml
DecisionTable:
  format: Markdown table
  columns:
    - Feature
    - Gazebo Classic
    - Gazebo Harmonic
    - Decision Rationale

  rows:
    - Architecture: [Plugin-based, ECS, "ECS provides flexibility"]
    - Support Status: [EOL Feb 2025, Until Sep 2028, "Active support critical"]
    - ROS 2 Integration: [Limited, Enhanced, "Better Jazzy integration"]
    - Performance (Humanoid): [RTF 0.5-1.5, RTF 1.0-3.5, "2-3x improvement"]
    - Python Bindings: [Partial, Full, "Easier development"]

  location: docs/module-02/01-gazebo-fundamentals.md
```

## Testing Strategy

### 10. Test Scenario

**Entity**: Validation Test
**Type**: Automated test + manual verification
**Purpose**: Ensure simulation meets performance criteria

**Attributes**:
```yaml
TestScenario:
  name: "Apartment Scene Performance Test"

  test_cases:
    - name: Frame Rate Test
      objective: Verify ≥60 FPS
      procedure:
        - Launch apartment_world.sdf with humanoid
        - Run gz stats for 60 seconds
        - Record average FPS
      acceptance: Average FPS ≥ 60

    - name: Humanoid Walking Test
      objective: Verify stable walking without falling
      procedure:
        - Command humanoid to walk 10 meters straight
        - Repeat 5 times
        - Record fall count
      acceptance: 0 falls in 5 runs

    - name: Sensor Data Test
      objective: Verify all sensors publish data
      procedure:
        - Launch simulation
        - Subscribe to all sensor topics
        - Verify data reception within 5 seconds
      acceptance: All sensors publishing at expected rates

    - name: Unity Bridge Latency Test
      objective: Verify Unity visualization latency
      procedure:
        - Connect Unity to Gazebo via bridge
        - Measure joint_states latency
        - Measure depth_image latency
      acceptance:
        - Joint states: < 5 ms
        - Depth images: < 30 ms

  hardware_requirements:
    gpu: RTX 4070 Ti or equivalent
    os: Ubuntu 24.04
    ros: ROS 2 Jazzy

  automation:
    - FPS measurement: gz stats --json
    - Sensor verification: ros2 topic hz <topic>
    - Latency measurement: custom Python script
```

**Validation Rules**:
- All tests must pass before content is considered complete
- Tests must be repeatable on target hardware
- Test results must be documented

## State Transitions

### Simulation Lifecycle

```yaml
SimulationStates:
  UNINITIALIZED:
    description: Before launch file execution
    transitions: [LOADING]

  LOADING:
    description: Launch file starting nodes
    duration: < 30 seconds
    transitions: [RUNNING, FAILED]

  RUNNING:
    description: Simulation active, ≥60 FPS
    monitoring:
      - FPS via gz stats
      - Sensor topics via ros2 topic list
      - CPU/GPU usage
    transitions: [PAUSED, STOPPED, FAILED]

  PAUSED:
    description: Simulation paused (user action)
    transitions: [RUNNING, STOPPED]

  STOPPED:
    description: Clean shutdown
    cleanup:
      - Kill all nodes
      - Release GPU resources
    transitions: [UNINITIALIZED]

  FAILED:
    description: Error state
    error_types:
      - Physics instability
      - Sensor failure
      - Bridge disconnection
      - Performance degradation (< 60 FPS)
    recovery: Manual restart or configuration adjustment
```

## Relationships Summary

```yaml
Relationships:
  HumanoidRobot:
    - spawned_in: ApartmentWorld
    - uses_sensors: SensorConfiguration
    - described_by: URDF file
    - controlled_via: ROS 2 topics

  ApartmentWorld:
    - contains: HumanoidRobot
    - built_with: gazebo-world-builder skill
    - uses_assets: CC0 furniture models
    - configured_in: SDF file

  SensorConfiguration:
    - attached_to: HumanoidRobot
    - publishes_to: ROS 2 topics
    - bridged_via: ros_gz_bridge

  BridgeConfiguration:
    - connects: [Gazebo, ROS 2, Unity]
    - configured_in: YAML file
    - uses: [ros_gz_bridge, ros_tcp_endpoint, Unity ROS TCP Connector]

  LaunchConfiguration:
    - starts: [robot_state_publisher, gz_sim, ros_gz_bridge]
    - configures: use_sim_time parameter
    - documented_in: quickstart.md

  DocumentationModule:
    - contains: 4 Markdown chapters
    - includes: Mermaid diagrams
    - includes: Decision tables
    - validated_by: TestScenario
```

## Notes

This data model focuses on the **specification of educational content and simulation artifacts** rather than traditional application data models. The entities represent configuration files, documentation structure, and validation criteria necessary to deliver the module successfully.
