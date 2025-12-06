# Apartment Worlds and Unity Integration

## Section 1: Apartment World Design and Asset Sourcing

Creating realistic apartment environments for humanoid robot simulation requires careful attention to polygon budgets, lighting strategies, and asset optimization. This section covers the design of a three-room apartment (living room, kitchen, bedroom) using the gazebo-world-builder skill.

### Apartment World Architecture

A well-designed apartment world for humanoid robotics should include:

- **Living Room** (70k visual polygons max, 10k collision polygons max)
- **Kitchen** (60k visual polygons max, 5k collision polygons max)
- **Bedroom** (70k visual polygons max, 5k collision polygons max)
- Total visual polygon budget: < 200k
- Total collision polygon budget: < 20k

The apartment should provide realistic navigation challenges for humanoid robots while maintaining high simulation performance.

### Polygon Budget Management

To stay within polygon budgets:

```xml
<!-- Example world file with optimized furniture -->
<sdf version="1.7">
  <world name="apartment_world">
    <!-- Physics configuration -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting setup -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Living room furniture -->
    <model name="sofa">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://sofa/meshes/sofa.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>2.0 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Kitchen counter -->
    <model name="kitchen_counter">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://kitchen_counter/meshes/counter.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1.5 0.6 0.9</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Bedroom furniture -->
    <model name="bed">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://bed/meshes/bed.dae</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>2.0 1.5 0.5</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### Lighting Strategy

For optimal performance, use directional lights without shadows:

```xml
<light name="apartment_light" type="directional">
  <cast_shadows>false</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.3 0.3 -1</direction>
</light>
```

### Physics Configuration

Configure physics for optimal humanoid simulation:

```xml
<physics type="dart">
  <max_step_size>0.001</max_step_size>  <!-- 1kHz update -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <dart>
    <solver>
      <type>PGS</type>
      <iters>100</iters>
      <sor>1.5</sor>
    </solver>
  </dart>
</physics>
```

### Asset Sourcing Strategy

Use CC0 licensed assets from reliable sources:

1. **Free3D** (free3d.com) - High-quality 3D models
2. **Open3dModel** - Robotics-specific models
3. **AWS RoboMaker Sample Assets** - Optimized for robotics simulation

### Blender Decimation Pipeline

For asset optimization, use Blender's decimate modifier:

1. Import high-poly model
2. Apply decimate modifier with "Unsubdivide" or "Collapse" mode
3. Target vertex count for visual meshes: < 10k per object
4. Target vertex count for collision meshes: < 1k per object
5. Export as DAE or STL for Gazebo

### Visual vs Collision Mesh Separation

Always separate visual and collision meshes:

- **Visual meshes**: High detail for rendering (max 10k triangles per object)
- **Collision meshes**: Simplified for physics (max 1k triangles per object)
- Use primitive shapes (boxes, cylinders) for collision when possible

## Section 2: World Building with gazebo-world-builder Skill

The gazebo-world-builder skill provides automated tools for creating realistic apartment environments with proper physics and optimization.

### Using the gazebo-world-builder Skill

The gazebo-world-builder skill can automatically generate apartment worlds with:

- Optimized furniture placement
- Polygon budget management
- Physics configuration
- Lighting setup
- Domain randomization for robust training

### Example World Generation Command

```bash
# Generate an apartment world with gazebo-world-builder
gazebo-world-builder --type apartment \
                     --rooms living_room,kitchen,bedroom \
                     --furniture sofa,chair,table,bed \
                     --polygon-budget 200000 \
                     --output-path worlds/apartment.sdf
```

### Generated World Structure

The generated world will include:

1. **Room Layout**: Properly sized rooms with doorways
2. **Furniture Placement**: Realistic positioning of furniture
3. **Physics Properties**: Optimized for humanoid simulation
4. **Lighting**: Directional lights without shadows
5. **Materials**: Realistic textures within polygon budgets

### Customization Options

Customize the generated world with:

```bash
# Add custom furniture or modify placement
gazebo-world-builder --config worlds/apartment_config.yaml \
                     --modify \
                     --add-furniture "custom_robot_table"
```

## Section 3: Unity Visualization Setup

Unity integration provides real-time visualization capabilities for Gazebo simulation, allowing for enhanced debugging and human-robot interaction.

### Unity Setup (2022.3 LTS)

Install Unity 2022.3 LTS with the following components:

1. **Unity Hub**: For version management
2. **Unity Editor**: 2022.3.x LTS version
3. **Visual Studio** or **Rider**: For scripting
4. **Unity Robotics Package**: For ROS integration

### ROS TCP Connector Installation

Install the Unity ROS TCP Connector:

1. Download the Unity package from the Unity Asset Store
2. Import into your Unity project
3. Add the ROSConnection prefab to your scene
4. Configure the connection settings

### ROSConnection Configuration

```csharp
using Unity.Robotics.ROSTCPConnector;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        // Connect to ROS on localhost:10000
        ros = ROSConnection.instance;
        ros.Initialize("127.0.0.1", 10000);
    }

    void Update()
    {
        // Subscribe to joint states
        ros.Subscribe<sensor_msgs.JointState>("/joint_states", OnJointState);
    }

    void OnJointState(sensor_msgs.JointState jointState)
    {
        // Update robot model in Unity based on joint states
        for (int i = 0; i < jointState.name.Count; i++)
        {
            string jointName = jointState.name[i];
            float jointPosition = jointState.position[i];

            // Update corresponding joint in Unity
            Transform jointTransform = GameObject.Find(jointName).transform;
            jointTransform.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
        }
    }
}
```

### Subscriber Scripts for Sensor Data

Create subscriber scripts for different sensor types:

**Depth Camera Subscriber:**
```csharp
using Unity.Robotics.ROS_TCPConnector;
using UnityEngine;

public class DepthCameraSubscriber : MonoBehaviour
{
    Texture2D depthTexture;

    void Start()
    {
        ROSConnection.instance.Subscribe<sensor_msgs.Image>("/depth_camera/image_raw", OnDepthImage);
    }

    void OnDepthImage(sensor_msgs.Image image)
    {
        // Convert ROS image to Unity texture
        depthTexture = new Texture2D(image.width, image.height, TextureFormat.RGB24, false);

        // Process image data and update UI texture
        // (Implementation depends on image encoding format)
    }
}
```

### Unity Scene Setup

Create a Unity scene with:

1. **Robot Model**: Imported URDF as FBX with proper joint hierarchy
2. **Camera**: Main camera for visualization
3. **Lighting**: Match Gazebo lighting conditions
4. **Environment**: Simple representation of apartment layout

## Section 4: Gazebo-Unity Bridge Configuration

The bridge between Gazebo and Unity requires proper configuration of both the ROS-Gazebo bridge and the ROS TCP endpoint.

### Bridge YAML Configuration

Create a bridge configuration file (`config/bridge_config.yaml`):

```yaml
# Bridge configuration for Gazebo-Unity integration
- ros_topic_name: "/joint_states"
  gz_topic_name: "/world/apartment/model/robot/joint_state"
  ros_type_name: "sensor_msgs/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "BIDIRECTIONAL"

- ros_topic_name: "/depth_camera/image_raw"
  gz_topic_name: "/world/apartment/model/robot/link/head_link/sensor/depth_camera/depth_camera/image"
  ros_type_name: "sensor_msgs/Image"
  gz_type_name: "gz.msgs.Image"
  direction: "ROS_TO_GZ"

- ros_topic_name: "/lidar/points"
  gz_topic_name: "/world/apartment/model/robot/link/lidar_link/sensor/gpu_lidar/scan"
  ros_type_name: "sensor_msgs/PointCloud2"
  gz_type_name: "gz.msgs.PointCloudPacked"
  direction: "ROS_TO_GZ"

- ros_topic_name: "/imu/data"
  gz_topic_name: "/world/apartment/model/robot/link/imu_link/sensor/imu_sensor/imu"
  ros_type_name: "sensor_msgs/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: "ROS_TO_GZ"

- ros_topic_name: "/contact/left_foot"
  gz_topic_name: "/world/apartment/model/robot/link/left_foot_link/sensor/left_foot_contact/contacts"
  ros_type_name: "gazebo_msgs/ContactsState"
  gz_type_name: "gz.msgs.Contacts"
  direction: "ROS_TO_GZ"
```

### ros_tcp_endpoint Configuration

Launch the ROS TCP endpoint to connect Unity:

```bash
# Launch the ROS TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

### Topic Mapping and QoS Tuning

Configure Quality of Service (QoS) settings for optimal performance:

```python
# Example Python script for QoS configuration
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def create_qos_profile(depth=10):
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,  # For sensor data
        history=QoSHistoryPolicy.KEEP_LAST
    )

# For critical control topics, use RELIABLE
def create_control_qos_profile(depth=1):
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,  # For control commands
        history=QoSHistoryPolicy.KEEP_LAST
    )
```

### Performance Monitoring

Monitor bridge performance with:

```bash
# Monitor topic rates
ros2 topic hz /joint_states
ros2 topic hz /depth_camera/image_raw

# Monitor network usage
iftop -i lo  # or the appropriate network interface

# Monitor latency
ros2 run topic_tools relay --ros-args -p topic_from:=/original_topic -p topic_to:=/monitored_topic
```

## Section 5: Complete System Integration

This section covers the complete workflow for integrating all components of the simulation system.

### Multi-Terminal Launch Workflow

Set up the complete system with multiple terminals:

**Terminal 1 - Gazebo Simulation:**
```bash
# Launch Gazebo with apartment world
gz sim -r apartment.sdf
```

**Terminal 2 - ROS Bridge:**
```bash
# Launch ROS-Gazebo bridge
ros2 launch ros_gz_bridge parameter_bridge --ros-args --params-file config/bridge_config.yaml
```

**Terminal 3 - TCP Endpoint:**
```bash
# Launch ROS TCP endpoint for Unity
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1 -p ROS_TCP_PORT:=10000
```

**Terminal 4 - Robot Controller:**
```bash
# Launch robot control nodes
ros2 launch my_robot_bringup robot.launch.py
```

### Performance Benchmarks

Verify performance targets:

**FPS Test:**
```bash
# Monitor Gazebo performance
gz stats

# Expected: ≥60 FPS average over 60 seconds
# Expected: RTF ≥1.0 (Real Time Factor)
```

**Walking Test:**
```bash
# Test humanoid walking capability
# 1. Launch robot in apartment world
# 2. Send navigation commands for 10m straight path
# 3. Repeat 5 times
# 4. Verify 0 falls during testing
```

**Sensor Test:**
```bash
# Verify all sensors publish at expected rates:
ros2 topic hz /depth_camera/image_raw     # Expected: ~30Hz
ros2 topic hz /lidar/points               # Expected: ~10Hz
ros2 topic hz /imu/data                   # Expected: ~100Hz
ros2 topic hz /contact/left_foot          # Expected: ~100Hz
ros2 topic hz /contact/right_foot         # Expected: ~100Hz
```

### Troubleshooting Guide

Common integration issues and solutions:

1. **Unity Connection Timeout**:
   - Verify ros_tcp_endpoint is running
   - Check firewall settings
   - Ensure Unity and ROS use same IP/network

2. **Low Simulation FPS**:
   - Reduce sensor update rates
   - Simplify collision meshes
   - Check GPU utilization

3. **Sensor Data Not Synchronizing**:
   - Verify use_sim_time is set to true
   - Check bridge configuration
   - Confirm topic names match

4. **Robot Model Not Updating in Unity**:
   - Verify joint names match between URDF and Unity model
   - Check ROS connection status
   - Ensure proper message types

### Unity Connection Test

Verify Unity connection performance:

```bash
# Measure connection time
time telnet 127.0.0.1 10000

# Expected: Connection should establish in <5 seconds
```

Monitor joint state latency:

```bash
# In Unity, log time difference between ROS message and Unity update
# Expected: <5ms latency for joint states
```

## Section 6: Apartment World Generation with gazebo-world-builder

Generate the final apartment world using the gazebo-world-builder skill:

### World Generation Command

```bash
# Generate optimized apartment world
gazebo-world-builder --type apartment \
                     --rooms living_room,kitchen,bedroom \
                     --polygon-budget-visual 200000 \
                     --polygon-budget-collision 20000 \
                     --file-size-limit 10MB \
                     --output worlds/apartment_world.sdf \
                     --validate-performance
```

### Generated World Specifications

The generated apartment world will meet these specifications:

- **Kitchen**: < 60k visual polys, < 5k collision polys
- **Living Room**: < 70k visual polys, < 10k collision polys
- **Bedroom**: < 70k visual polys, < 5k collision polys
- **Total file size**: < 10MB
- **Load time**: < 30 seconds in Gazebo
- **Performance**: ≥60 FPS with humanoid robot

### Validation Process

Validate the generated world with:

```bash
# Test world loading time
time gz sim -s apartment_world.sdf

# Test simulation performance
gz stats -e 60  # Monitor for 60 seconds

# Verify polygon counts
# (Use mesh analysis tools to check visual and collision mesh counts)
```

This complete system provides a high-performance simulation environment for humanoid robots with realistic apartment environments and Unity visualization capabilities.