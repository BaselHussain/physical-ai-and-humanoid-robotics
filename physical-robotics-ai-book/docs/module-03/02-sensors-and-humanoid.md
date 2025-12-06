# Sensors and Humanoid Integration

## Section 1: Humanoid Robot URDF Design with Sensor Integration

Designing a humanoid robot for Gazebo Harmonic simulation requires careful attention to both kinematic structure and sensor placement. This section covers the creation of a humanoid URDF with integrated sensors using the urdf-builder skill approach and proper depth camera configuration.

### Humanoid Kinematic Structure (12-32 DoF)

A humanoid robot for physical AI applications typically requires 12-32 degrees of freedom (DoF) to achieve human-like mobility and manipulation capabilities. The kinematic structure follows the standard humanoid configuration:

- **Trunk**: 1-3 DoF (torso pitch, roll, yaw)
- **Head**: 2-3 DoF (neck pitch, yaw, roll)
- **Arms**: 6-8 DoF each (shoulder 3 DoF, elbow 1 DoF, wrist 2-4 DoF)
- **Legs**: 6-7 DoF each (hip 3 DoF, knee 1 DoF, ankle 2 DoF)

Here's an example of a basic humanoid URDF structure with optimized mesh configuration:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://humanoid_description/meshes/base_link.dae" scale="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso_link">
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://humanoid_description/meshes/torso.dae" scale="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head with depth camera -->
  <joint name="head_joint" type="revolute">
    <parent link="torso_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>

  <link name="head_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://humanoid_description/meshes/head.dae" scale="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

### Mesh Optimization for Performance

For optimal simulation performance, mesh optimization is critical:

- **Visual meshes**: Should have < 30k triangles total for the entire robot
- **Collision meshes**: Should have < 3k triangles total for efficient physics computation
- **Texture optimization**: Use compressed textures (< 2MB total for the robot)
- **LOD (Level of Detail)**: Implement multiple LOD levels for different viewing distances

### Depth Camera Configuration (RealSense D435)

The depth camera is a crucial sensor for humanoid navigation and perception tasks. We configure it based on the RealSense D435 specifications:

```xml
<!-- Depth camera mounted on the head -->
<joint name="camera_joint" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<link name="camera_link">
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<!-- Depth camera sensor definition -->
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>848</width>
        <height>480</height>
        <format>RGB8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- 7mm depth accuracy -->
      </noise>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
    <topic>depth_camera/image_raw</topic>
  </sensor>
</gazebo>
```

### ROS 2 Topic Output and RViz2 Visualization

The depth camera publishes to several ROS 2 topics:

- `/depth_camera/image_raw` - Raw RGB image (sensor_msgs/Image)
- `/depth_camera/depth/image_raw` - Depth image (sensor_msgs/Image)
- `/depth_camera/points` - Point cloud data (sensor_msgs/PointCloud2)

For RViz2 visualization, you'll need to subscribe to these topics and configure appropriate displays.

## Section 2: LiDAR and IMU Configuration

This section covers the configuration of LiDAR and IMU sensors for the humanoid robot, including GPU-accelerated LiDAR and MEMS-based IMU with proper noise modeling.

### GPU-Accelerated LiDAR (16-Layer Configuration)

GPU-accelerated LiDAR provides high-performance point cloud generation essential for navigation and mapping:

```xml
<!-- LiDAR joint and link -->
<joint name="lidar_joint" type="fixed">
  <parent link="head_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.0 0 0.1" rpy="0 0 0"/>
</joint>

<link name="lidar_link">
  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
  </inertial>
</link>

<!-- GPU LiDAR sensor -->
<gazebo reference="lidar_link">
  <sensor name="gpu_lidar" type="gpu_lidar">
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>1024</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -π -->
          <max_angle>3.14159</max_angle>   <!-- π -->
        </horizontal>
        <vertical>
          <samples>16</samples>
          <resolution>1</resolution>
          <min_angle>-0.349066</min_angle>  <!-- -20 degrees -->
          <max_angle>0.349066</max_angle>   <!-- 20 degrees -->
        </vertical>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <always_on>true</always_on>
    <visualize>false</visualize>
    <topic>lidar/points</topic>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1cm noise -->
    </noise>
  </sensor>
</gazebo>
```

### Point Cloud Visualization and Processing

The LiDAR sensor publishes to the `/lidar/points` topic as a sensor_msgs/PointCloud2 message. For visualization in RViz2:

1. Add a PointCloud2 display
2. Set the topic to `/lidar/points`
3. Configure appropriate color transforms and size scaling

### IMU Configuration (MEMS-based)

The IMU provides crucial orientation and acceleration data for humanoid balance and navigation:

```xml
<!-- IMU joint and link -->
<joint name="imu_joint" type="fixed">
  <parent link="torso_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<!-- IMU sensor -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <update_rate>100</update_rate>
    <topic>imu/data</topic>
    <always_on>true</always_on>
    <visualize>false</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>  <!-- 0.5 deg/s -->
            <bias_mean>0.005</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
            <bias_mean>0.005</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
            <bias_mean>0.005</bias_mean>
            <bias_stddev>0.01</bias_stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- 170 μg -->
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.02</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.02</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.01</bias_mean>
            <bias_stddev>0.02</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### IMU Data Interpretation

The IMU sensor publishes to `/imu/data` as sensor_msgs/Imu messages containing:
- Orientation (quaternion)
- Angular velocity (vector3)
- Linear acceleration (vector3)

These values are crucial for humanoid balance control algorithms and state estimation.

## Section 3: Contact Sensors and Sensor Integration Testing

Contact sensors are essential for humanoid walking and balance control, providing ground contact information for each foot.

### Contact Sensor Configuration

Contact sensors are configured for both feet to detect ground contact:

```xml
<!-- Left foot contact sensor -->
<gazebo reference="left_foot_link">
  <sensor name="left_foot_contact" type="contact">
    <update_rate>100</update_rate>
    <contact>
      <collision>left_foot_collision</collision>
    </contact>
    <always_on>true</always_on>
    <visualize>false</visualize>
    <topic>contact/left_foot</topic>
  </sensor>
</gazebo>

<!-- Right foot contact sensor -->
<gazebo reference="right_foot_link">
  <sensor name="right_foot_contact" type="contact">
    <update_rate>100</update_rate>
    <contact>
      <collision>right_foot_collision</collision>
    </contact>
    <always_on>true</always_on>
    <visualize>false</visualize>
    <topic>contact/right_foot</topic>
  </sensor>
</gazebo>
```

### Integration with Walking Controllers

Contact sensors are used in walking controllers to:
- Detect ground contact for stable foot placement
- Trigger foot lift during walking phases
- Provide feedback for balance control algorithms
- Detect falls or unstable states

### Sensor Integration and Testing

To verify all sensors are working correctly, use the following ROS 2 commands:

```bash
# Check available topics
ros2 topic list | grep -E "(depth_camera|lidar|imu|contact)"

# Monitor depth camera data rate
ros2 topic hz /depth_camera/image_raw

# Monitor LiDAR data rate
ros2 topic hz /lidar/points

# Monitor IMU data rate
ros2 topic hz /imu/data

# Monitor contact sensor data
ros2 topic hz /contact/left_foot
ros2 topic hz /contact/right_foot
```

Expected data rates:
- Depth camera: ~30 Hz
- LiDAR: ~10 Hz
- IMU: ~100 Hz
- Contact sensors: ~100 Hz

### Performance Validation

Validate sensor performance with:

```bash
# Monitor sensor processing time
ros2 run topic_tools relay /depth_camera/image_raw /monitor/depth_camera

# Check for dropped messages
ros2 topic echo /depth_camera/image_raw --field header.seq | head -n 100
```

### Troubleshooting Guide

Common sensor issues and solutions:

1. **Sensors not publishing data**:
   - Check that the robot is properly spawned in Gazebo
   - Verify sensor definitions in URDF
   - Ensure ros_gz_bridge is running

2. **Low data rates**:
   - Check GPU performance for depth camera and LiDAR
   - Reduce sensor resolution if needed
   - Verify physics update rates

3. **High latency**:
   - Check network configuration
   - Verify use_sim_time parameter
   - Reduce topic publishing frequency if needed

## Section 4: Complete Humanoid Sensor Integration Example

Here's a complete example of a humanoid robot with all four sensor types integrated:

```xml
<?xml version="1.0"?>
<robot name="sensor_equipped_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base and trunk -->
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Head with depth camera -->
  <joint name="head_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>

  <link name="head_link">
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Depth camera -->
  <joint name="camera_joint" type="fixed">
    <parent link="head_link"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_link"/>

  <gazebo reference="camera_link">
    <sensor name="depth_camera" type="depth">
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>848</width>
          <height>480</height>
          <format>RGB8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <always_on>true</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>

  <!-- LiDAR -->
  <joint name="lidar_joint" type="fixed">
    <parent link="head_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link"/>

  <gazebo reference="lidar_link">
    <sensor name="gpu_lidar" type="gpu_lidar">
      <update_rate>10</update_rate>
      <ray>
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
            <min_angle>-0.349066</min_angle>
            <max_angle>0.349066</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <always_on>true</always_on>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <!-- IMU -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <update_rate>100</update_rate>
      <topic>imu/data</topic>
      <always_on>true</always_on>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <!-- Foot links for contact sensors -->
  <joint name="left_foot_joint" type="fixed">
    <parent link="base_link"/>
    <child link="left_foot_link"/>
    <origin xyz="-0.1 -0.05 0" rpy="0 0 0"/>
  </joint>

  <link name="left_foot_link">
    <collision name="left_foot_collision">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.02"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_foot_joint" type="fixed">
    <parent link="base_link"/>
    <child link="right_foot_link"/>
    <origin xyz="-0.1 0.05 0" rpy="0 0 0"/>
  </joint>

  <link name="right_foot_link">
    <collision name="right_foot_collision">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.02"/>
      </geometry>
    </collision>
  </link>

  <!-- Contact sensors -->
  <gazebo reference="left_foot_link">
    <sensor name="left_foot_contact" type="contact">
      <update_rate>100</update_rate>
      <contact>
        <collision>left_foot_collision</collision>
      </contact>
      <always_on>true</always_on>
      <visualize>false</visualize>
    </sensor>
  </gazebo>

  <gazebo reference="right_foot_link">
    <sensor name="right_foot_contact" type="contact">
      <update_rate>100</update_rate>
      <contact>
        <collision>right_foot_collision</collision>
      </contact>
      <always_on>true</always_on>
      <visualize>false</visualize>
    </sensor>
  </gazebo>
</robot>
```

This complete example demonstrates the integration of all four sensor types (depth camera, LiDAR, IMU, and contact sensors) in a humanoid robot configuration suitable for Gazebo Harmonic simulation.