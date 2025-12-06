# Research: Module 03 — The Digital Twin (Gazebo & Unity)

**Feature**: 003-gazebo-simulation
**Date**: 2025-12-06
**Purpose**: Technical research for Gazebo Harmonic humanoid simulation with Unity visualization

## 1. Gazebo Harmonic vs Classic Decision

### Decision
**Choose Gazebo Harmonic** for this humanoid robotics simulation project.

### Rationale
- **Modern Architecture**: Entity-Component-System (ECS) architecture provides greater flexibility than Classic's plugin system
- **Active Support**: Gazebo Harmonic supported until September 2028; Gazebo Classic reached end-of-life in February 2025
- **Performance Improvements**: Better RTF performance for open kinematic chains (critical for humanoid robots) under Bullet Featherstone plugin
- **Enhanced Features**: Python bindings for all convenience classes, Python system development, automatic inertia/mass calculation for primitive shapes and water-tight meshes
- **ROS 2 Integration**: Enhanced integration with ROS 2 Jazzy, enabling advanced features and functionalities
- **OS Compatibility**: Works best on Ubuntu Jammy (22.04) and Noble (24.04)

### Alternatives Considered
- **Gazebo Classic**: Rejected due to end-of-life status (February 2025) and inferior performance for articulated systems
- **MuJoCo**: Not considered due to licensing restrictions and less mature ROS integration
- **PyBullet**: Not considered due to lack of native ROS 2 integration and limited sensor simulation capabilities

### Technical Specifications
- **Platform**: Ubuntu 24.04 (Noble) recommended
- **ROS Version**: ROS 2 Jazzy
- **Physics Engine**: DART (gz-physics-dartsim-plugin) as primary
- **Architecture**: Entity-Component-System with plugin-based physics/rendering engines

### Performance Benchmarks
- **RTF for humanoids**: Featherstone-based solvers (DART) provide better accuracy for articulated systems
- **GPU Support**: Harmonic includes GPU acceleration for rendering and physics
- **Expected RTF**: 1.0-3.5 on RTX 4070 Ti with optimized models

## 2. URDF to SDF Conversion

### Decision
**Use automatic URDF loading via ros_gz_sim** rather than manual conversion for primary workflow.

### Rationale
- **Automatic Conversion**: Gazebo Harmonic converts URDF to SDF automatically under the hood
- **ROS 2 Integration**: ros_gz_sim create node spawns robots from robot_description topic seamlessly
- **Maintainability**: Single source of truth (URDF/xacro) reduces maintenance burden
- **Gazebo Tags**: URDF supports `<gazebo>` extensions for SDF-specific properties

### Alternatives Considered
- **Manual gz sdf conversion**: Available but adds maintenance overhead
- **Native SDF authoring**: More verbose, harder to maintain, loses ROS ecosystem compatibility

### Technical Specifications

#### Recommended Workflow
1. Author robot in URDF/xacro format with ROS conventions
2. Add `<gazebo>` tags for simulation-specific properties (physics, sensors, plugins)
3. Use robot_state_publisher to publish to robot_description topic
4. Spawn in Gazebo via ros_gz_sim create node

#### Command-Line Conversion (fallback)
```bash
gz sdf -p robot.urdf > robot.sdf
xacro robot.urdf.xacro > robot.urdf  # If using xacro
```

## 3. Sensor Simulation in Gazebo Harmonic

### Decision
**Implement comprehensive sensor suite with realistic noise models** matching real-world sensor characteristics.

### Technical Specifications

#### A. Depth Camera (RealSense D435-like)

**Hardware Specifications (Real D435)**
- **Resolution**: 848x480 (optimal), 1280x720 (max depth), 1920x1080 (RGB)
- **Frame Rate**: Up to 90 FPS (standard), 300 FPS (high-speed mode at reduced resolution)
- **Range**: 0.3m - 3m (optimal), up to 10m (extended)
- **MinZ**: ~19.5cm at 848x480
- **FOV**: 85.2° × 58° (depth)
- **Shutter Type**: Global shutter

**Gazebo SDF Configuration**
```xml
<sensor name="depth_camera" type="depth">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.487</horizontal_fov>  <!-- 85.2° -->
    <image>
      <width>848</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.195</near>  <!-- MinZ -->
      <far>10.0</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  </camera>
</sensor>
```

**Noise Model**
- **Type**: Gaussian
- **Mean**: 0.0
- **Stddev**: 0.007 (typical for RealSense depth cameras)
- **Error Scaling**: Depth error scales as square of distance

#### B. LiDAR (2D/3D)

**Recommended Configuration**
```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15° -->
        <max_angle>0.261799</max_angle>   <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
</sensor>
```

**Noise Parameters**
- **Type**: Gaussian noise added to each beam range
- **Mean**: 0.0
- **Stddev**: 0.01 (1cm typical for indoor LiDAR)
- **Performance**: Use gpu_lidar type for better performance

#### C. IMU (Inertial Measurement Unit)

**Configuration with Realistic Noise**
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.005</bias_stddev>
          <dynamic_bias_correlation_time>400.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- Similar for y, z -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
          <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- Similar for y, z -->
    </linear_acceleration>
  </imu>
</sensor>
```

**Noise Model Components**
- **Angular Rate Noise**: ~0.009 rad/s stddev
- **Linear Acceleration Noise**: ~0.017 m/s² stddev
- **Dynamic Bias Correlation Time**: 300-400 seconds (5-7 minutes)

#### D. Contact Sensors

**Configuration**
```xml
<sensor name="contact_sensor" type="contact">
  <contact>
    <collision>foot_collision</collision>
  </contact>
  <update_rate>100</update_rate>
  <always_on>true</always_on>
</sensor>
```

### Best Practices
- **Update Rates**: Set to 0 in ROS plugins to inherit from sensor update_rate
- **Always On**: Set to true for sensors needed for control (IMU, contact)
- **Visualization**: Enable during development, disable for performance
- **GPU Acceleration**: Use gpu_lidar for performance

## 4. Gazebo-Unity TCP Bridge

### Decision
**Use ROS 2 as intermediary with ros_gz_bridge + Unity ROS TCP Connector** for Gazebo-Unity communication.

### Rationale
- **Performance**: ROS native serialization eliminates JSON encoding/decoding overhead (17x faster than ROSBridge)
- **Proven Architecture**: Industry-standard pattern with active community support
- **Flexibility**: Supports bidirectional data exchange with configurable QoS
- **Ecosystem**: Leverages existing ROS 2 tools and debugging capabilities

### Alternatives Considered
- **ROS# with ROSBridge WebSocket**: Rejected due to poor performance (10s per image vs 0.6s with TCP Connector)
- **Custom TCP Protocol**: Rejected due to development overhead and lack of ecosystem support
- **Direct Gazebo-Unity Bridge**: No mature open-source solution available

### Technical Specifications

#### Architecture Components
1. **Gazebo Harmonic** → 2. **ros_gz_bridge** → 3. **ROS 2 Topics** → 4. **Unity ROS TCP Connector** → 5. **Unity 3D**

#### A. ros_gz_bridge Configuration

```yaml
# bridge_config.yaml
- ros_topic_name: "/robot/joint_states"
  gz_topic_name: "/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"

- ros_topic_name: "/robot/depth_camera/image"
  gz_topic_name: "/depth_camera"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  subscriber_queue: 5
  publisher_queue: 6
  qos_profile: SENSOR_DATA
```

**Launch**
```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config.yaml
```

#### B. Unity ROS TCP Connector

**Configuration**
```csharp
ROSConnection.GetOrCreateInstance().Connect(
    rosIPAddress: "127.0.0.1",
    rosPort: 10000
);

ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
    "/robot/joint_states",
    OnJointStatesReceived
);
```

#### C. ROS TCP Endpoint

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000
```

### Performance Considerations

#### Message Types and Sizes
- **Joint States**: sensor_msgs/msg/JointState (1-5 KB/msg)
- **Images**: sensor_msgs/msg/Image (compressed, ~50-200 KB/frame)
- **Point Clouds**: sensor_msgs/msg/PointCloud2 (100 KB - 5 MB)
- **Odometry**: nav_msgs/msg/Odometry (~200 bytes)

#### Expected Metrics
- **Joint States (60 Hz)**: < 5 ms latency
- **Depth Images (30 Hz)**: 15-30 ms latency
- **Point Clouds (10 Hz)**: 30-50 ms latency
- **Total Network**: < 50 Mbps for full sensor suite
- **CPU Overhead**: 5-10% on modern hardware

## 5. Performance Optimization for 60 FPS

### Decision
**Implement multi-layered optimization strategy** targeting mesh, physics, rendering, and sensor configurations.

### Technical Specifications

#### A. Mesh Optimization

**Collision Mesh Decimation**
- **Tool**: Blender Decimate Modifier (Collapse function)
- **Target**: < 500 triangles per collision mesh for complex parts
- **Target**: < 100 triangles for simple geometries (boxes, cylinders)
- **Visual Mesh**: Can be higher (2,000-5,000 triangles) as rendering is GPU-accelerated
- **Impact**: 2-5x RTF improvement for complex humanoid robots

**Polygon Budget Recommendations**
- **Humanoid Robot (total)**:
  - Visual: 20,000-50,000 triangles
  - Collision: 2,000-5,000 triangles
- **Apartment Environment**:
  - Visual: 100,000-200,000 triangles (entire scene)
  - Collision: 10,000-30,000 triangles

#### B. Physics Engine Optimization

**Physics Engine**: DART (gz-physics-dartsim-plugin)

**Physics Parameters**
```xml
<physics name="dart_physics" type="dart">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>

  <dart>
    <collision_detector>bullet</collision_detector>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
  </dart>
</physics>
```

#### C. Rendering Optimization

**Lighting Configuration**
```xml
<scene>
  <ambient>0.4 0.4 0.4 1</ambient>
  <background>0.7 0.7 0.7 1</background>
  <shadows>false</shadows>  <!-- Major performance impact -->
</scene>

<light type="directional" name="sun">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.1 -0.9</direction>
</light>
```

**Impact**: Disabling shadows alone can improve RTF from 0.5 to 8+ with 50% GPU and 60% CPU reduction

#### D. Sensor Optimization

**Update Rates**
- **Depth Camera**: 30 Hz (not 60, rendering expensive)
- **LiDAR**: 10 Hz (sufficient for navigation)
- **IMU**: 100 Hz (minimum for control)
- **Contact**: 100 Hz

**Configuration**
```xml
<sensor name="depth_camera" type="depth">
  <update_rate>30</update_rate>
  <camera>
    <image>
      <width>640</width>  <!-- Reduced from 848 for performance -->
      <height>480</height>
    </image>
  </camera>
  <visualize>false</visualize>
</sensor>
```

#### E. Performance Budget

| Component | Visual Polygons | Collision Polygons | Update Rate | RTF Impact |
|-----------|-----------------|-------------------|-------------|------------|
| Humanoid Robot | 30,000 | 3,000 | Physics: 1kHz | 0.3 |
| Depth Camera | - | - | 30 Hz | 0.1 |
| LiDAR | - | - | 10 Hz | 0.05 |
| IMU | - | - | 100 Hz | 0.01 |
| Contact Sensors | - | - | 100 Hz | 0.02 |
| Apartment (Scene) | 150,000 | 20,000 | Static | 0.1 |
| Lighting (no shadows) | - | - | - | 0.05 |
| **Total Expected RTF** | | | | **0.63** |
| **Headroom for 60 FPS** | | | | **1.6x** |

### Optimization Checklist
- Decimate collision meshes to <500 triangles per component
- Separate visual and collision meshes
- Disable shadows in scene configuration
- Use directional light instead of multiple point lights
- Set physics to DART with Bullet collision detector
- Limit sensor update rates
- Use gpu_lidar instead of standard lidar
- Disable sensor visualization in GUI
- Monitor with gz stats and adjust bottlenecks iteratively

## 6. Apartment World Building

### Decision
**Build modular, lightweight apartment world using low-poly assets from open-source repositories** with Blender-based mesh optimization pipeline.

### Rationale
- **Performance**: Low-poly meshes critical for maintaining 60 FPS
- **Realism**: Sufficient visual fidelity for robot navigation and task learning
- **Modularity**: Reusable room/furniture components for varied environments
- **Licensing**: Open-source/CC0 assets avoid legal complications

### Asset Sources (Free/Open)

1. **Free3D** - 83 Free Furniture 3D models in COLLADA (.dae)
   - URL: https://free3d.com/3d-models/collada-furniture

2. **Open3dModel** - 25 Free 3D Furniture Models in COLLADA
   - URL: https://open3dmodel.com/3d-models/collada-furniture

3. **TurboSquid Free** - COLLADA furniture models
   - URL: https://www.turbosquid.com/Search/3D-Models/free/furniture/dae

4. **AWS RoboMaker Small House World**
   - URL: https://github.com/aws-robotics/aws-robomaker-small-house-world
   - License: Apache 2.0 (permissive)

5. **Gazebo Model Database**
   - Built-in: Cafe, kitchen, Willow Garage models
   - Access: Via Gazebo GUI Insert tab

### Polygon Budget Guidelines

| Object Type | Visual Polygons | Collision Polygons |
|-------------|----------------|-------------------|
| Wall | 4-12 | 4-12 |
| Floor | 2-4 | 2 |
| Door | 50-200 | 12-24 |
| Table | 200-1,000 | 50-100 |
| Chair | 300-1,500 | 100-300 |
| Sofa | 500-2,000 | 150-400 |
| Kitchen Counter | 500-1,500 | 100-300 |

**Total Scene Budget**: <200,000 visual, <30,000 collision for 60 FPS on RTX 4070 Ti

### Lighting Techniques

**Recommended Approach**: Baked Lighting + Simple Real-time

1. **Ambient + Directional** (Real-time) - See rendering optimization above
2. **Baked Textures** (Offline) - Blender Cycles/Eevee for light baking to diffuse textures
3. **Point Lights** (Minimal Use) - Only for critical task lighting

**Lighting Budget**
- **Directional Lights**: 1-2 (sun + optional fill)
- **Point Lights**: 0-4 (only for specific task areas)
- **Shadows**: DISABLED globally

### Mesh Optimization Pipeline

**Step 1**: Acquisition - Download models from asset sources
**Step 2**: Import to Blender
**Step 3**: Decimate visual mesh (30% ratio)
**Step 4**: Create collision mesh (10% ratio, aggressive decimation)
**Step 5**: Export to COLLADA (.dae)
**Step 6**: Integration into Gazebo model structure

### Best Practices
- Prioritize models with <2,000 polygons originally
- Check for manifold geometry
- Limit texture resolution: 1024x1024 for large objects, 512x512 for small
- Use primitive shapes for collision where possible
- Share textures across similar objects

## Summary: Key Implementation Decisions

| Domain | Decision | Primary Rationale | Expected Metric |
|--------|----------|-------------------|-----------------|
| **Simulator** | Gazebo Harmonic | Active support, ROS 2 integration, ECS architecture | RTF 1.0-3.5 |
| **Physics Engine** | DART | Featherstone algorithm optimal for humanoid articulation | Contact stability |
| **URDF/SDF** | Automatic URDF→SDF via ros_gz_sim | Maintainability, ROS ecosystem compatibility | N/A |
| **Depth Camera** | RealSense D435 specs (848×480 @ 30Hz) | Industry standard, proven sim-to-real transfer | 15-30ms latency |
| **LiDAR** | GPU LiDAR (360-1024 samples @ 10Hz) | GPU acceleration, sufficient for indoor navigation | <50ms latency |
| **IMU** | Realistic noise (0.009 rad/s, 0.017 m/s²) | Matches commercial MEMS IMU characteristics | 100Hz update |
| **Bridge** | ros_gz_bridge + Unity ROS TCP Connector | 17x faster than JSON-based alternatives | 0.6s per image |
| **Rendering** | Shadows disabled, 1-2 directional lights | 8x RTF improvement, minimal visual degradation | 60 FPS |
| **Meshes** | <500 tri collision, <5000 tri visual per object | 2-5x RTF improvement for complex robots | <30k tri scene |
| **Assets** | Free3D, Open3dModel, AWS RoboMaker | Free, COLLADA-compatible, community-tested | N/A |
