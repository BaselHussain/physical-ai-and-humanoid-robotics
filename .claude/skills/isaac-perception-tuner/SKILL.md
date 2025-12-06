# Isaac Perception Tuner Skill

```yaml
name: isaac-perception-tuner
version: 1.0.0
description: Tune Isaac ROS perception packages (cuVSLAM, DetectNet, depth processing) on Jetson hardware with RealSense cameras for optimal performance
keywords: [isaac-ros, perception, cuVSLAM, DetectNet, jetson, realsense, visual-slam, object-detection, depth-processing]
category: robotics-perception
author: Physical AI Engineering Team
created: 2025-12-05
updated: 2025-12-05
```

---

## When to Use This Skill

Invoke this skill when the user needs to:

1. **Configure Isaac ROS Perception Packages**:
   - Set up cuVSLAM (GPU-accelerated visual SLAM) with RealSense D435/D455
   - Configure Isaac ROS DetectNet for object/person detection
   - Tune depth processing parameters for navigation
   - Integrate Isaac ROS FoundationPose for 6-DoF object pose estimation

2. **Optimize Performance on Jetson Hardware**:
   - Tune perception pipelines for Jetson Orin Nano/NX (8GB-16GB)
   - Balance latency vs. accuracy for real-time robotics applications
   - Configure TensorRT engine settings for GPU inference
   - Monitor and optimize memory usage and thermal throttling

3. **Tune Detection Confidence Thresholds**:
   - Adjust DetectNet confidence thresholds to reduce false positives
   - Configure NMS (Non-Maximum Suppression) parameters
   - Calibrate depth filtering for noisy environments
   - Tune SLAM loop closure thresholds

4. **Validate Perception Quality**:
   - Measure localization accuracy (ATE/RPE metrics)
   - Benchmark detection precision/recall on test datasets
   - Profile inference latency and throughput
   - Generate perception quality reports with visualization

**Example User Requests**:
- "Tune Isaac ROS cuVSLAM for my RealSense D435 on Jetson Orin Nano"
- "Configure DetectNet to detect people with <10% false positive rate"
- "Optimize depth processing for obstacle avoidance in indoor navigation"
- "Benchmark Isaac ROS perception pipeline latency on Jetson Orin NX"

---

## How It Works

This skill follows an 8-step workflow to configure, tune, and validate Isaac ROS perception packages:

### Step 1: Hardware and Software Context Gathering

**What the skill does**:
- Identifies Jetson hardware model (Orin Nano 8GB, Orin NX 16GB, etc.)
- Determines camera model (RealSense D435, D435i, D455)
- Checks Isaac ROS version and installed packages
- Reviews robotics application requirements (navigation, manipulation, etc.)

**User interaction**:
- Ask: "What Jetson hardware are you using? (e.g., Orin Nano 8GB, Orin NX 16GB)"
- Ask: "Which camera? (e.g., RealSense D435, D435i, D455)"
- Ask: "Which Isaac ROS packages? (cuVSLAM, DetectNet, Depth, FoundationPose, all)"
- Ask: "What's the primary use case? (navigation, manipulation, person following, custom)"

**Deliverable**:
```yaml
hardware_context.yaml:
  jetson:
    model: "Jetson Orin Nano 8GB"
    cuda_version: "11.4"
    tensorrt_version: "8.5.2"
    max_gpu_memory: "7.5 GB"
  camera:
    model: "RealSense D435"
    depth_resolution: "640x480"
    rgb_resolution: "640x480"
    fps: 30
  isaac_ros_version: "2.1.0"
  packages:
    - cuVSLAM
    - DetectNet
    - Depth
  use_case: "indoor_navigation"
  latency_budget_ms: 100
  accuracy_priority: "balanced"  # low | balanced | high
```

---

### Step 2: Create Base Perception Launch Configuration

**What the skill does**:
- Generates ROS 2 launch file for Isaac ROS perception pipeline
- Configures camera driver with optimal resolution/FPS
- Sets up image transport and rectification
- Adds visualization nodes (RViz2 markers for debugging)

**Deliverable**:
```python
# launch/isaac_perception.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_namespace = LaunchConfiguration('camera_namespace', default='camera')

    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace=camera_namespace,
        parameters=[{
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'enable_sync': True,
            'align_depth.enable': True,
            'enable_color': True,
            'enable_depth': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'depth_module.emitter_enabled': 1,
            'decimation_filter.enable': False,
            'spatial_filter.enable': True,
            'temporal_filter.enable': True,
            'hole_filling_filter.enable': True,
        }],
        remappings=[
            ('/camera/depth/image_rect_raw', '/depth'),
            ('/camera/color/image_raw', '/rgb'),
            ('/camera/infra1/image_rect_raw', '/infra1'),
            ('/camera/infra2/image_rect_raw', '/infra2'),
        ]
    )

    # Isaac ROS container for GPU-accelerated perception
    isaac_ros_container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # Image format converter
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_converter',
                parameters=[{
                    'encoding_desired': 'rgb8',
                    'backend': 'CUDA'
                }],
                remappings=[
                    ('image_raw', '/camera/color/image_raw'),
                    ('image', '/image_rect')
                ]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        realsense_node,
        isaac_ros_container,
    ])
```

---

### Step 3: Configure cuVSLAM (Visual SLAM)

**What the skill does**:
- Adds Isaac ROS cuVSLAM node to container
- Configures camera intrinsics and rectification
- Sets map resolution and feature tracking parameters
- Tunes loop closure detection thresholds
- Configures odometry covariance for sensor fusion

**Key tuning parameters**:
- `rectified_images`: `true` (use rectified stereo images for accuracy)
- `enable_image_denoising`: `true` (reduce noise on Jetson, +5ms latency)
- `enable_localization_n_mapping`: `true` (full SLAM vs. odometry-only)
- `map_frame`: `"map"`, `odom_frame`: `"odom"`, `base_frame`: `"base_link"`
- `num_cameras`: `2` (stereo), `1` (monocular with IMU)
- `min_num_images`: `4` (minimum images before publishing pose, higher = more stable)
- `img_jitter_threshold_ms`: `33.33` (30 FPS = 33ms tolerance)

**Deliverable**:
```python
# Add to isaac_ros_container composable_node_descriptions:
ComposableNode(
    package='isaac_ros_visual_slam',
    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
    name='visual_slam',
    parameters=[{
        'use_sim_time': use_sim_time,
        'denoise_input_images': True,
        'rectified_images': True,
        'enable_image_denoising': True,
        'enable_localization_n_mapping': True,
        'num_cameras': 2,  # Stereo mode
        'min_num_images': 4,
        'img_jitter_threshold_ms': 33.33,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'enable_slam_visualization': True,
        'enable_landmarks_view': True,
        'enable_observations_view': True,
        'path_max_size': 1024,
        # Loop closure tuning
        'enable_loop_closure': True,
        'loop_closure_frequency': 10,  # Hz
        'loop_closure_max_correspondences': 50,
        # Performance tuning for Jetson
        'max_feature_count': 300,  # Reduce for Orin Nano (default 500)
        'enable_imu_fusion': False,  # Set true if IMU available
    }],
    remappings=[
        ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
        ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
        ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
        ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
        ('visual_slam/tracking/odometry', '/visual_slam/tracking/odometry'),
        ('visual_slam/tracking/vo_pose', '/visual_slam/tracking/vo_pose'),
        ('visual_slam/tracking/vo_pose_covariance', '/visual_slam/tracking/vo_pose_covariance'),
    ]
),
```

**Performance targets** (Jetson Orin Nano 8GB):
- Latency: 40-60ms per frame (stereo processing + feature tracking)
- Memory: ~2.5GB GPU + 1.5GB CPU
- Localization accuracy: <2% trajectory error (indoor, textured environment)

---

### Step 4: Configure DetectNet (Object Detection)

**What the skill does**:
- Adds Isaac ROS DNN Image Encoder + Tensor RT node
- Configures PeopleNet or custom DetectNet model
- Tunes confidence thresholds and NMS (Non-Maximum Suppression)
- Sets up bounding box filtering and tracking

**Key tuning parameters**:
- `model_name`: `"peoplenet"` (person detection), `"trafficcamnet"` (vehicles), `"dashcamnet"` (cars/bikes/people)
- `model_repository_paths`: Path to TensorRT engine
- `confidence_threshold`: `0.5` (default), increase to 0.7 to reduce false positives
- `iou_threshold`: `0.4` (NMS threshold, lower = fewer duplicate boxes)
- `max_batch_size`: `1` (Jetson memory constraint)

**Deliverable**:
```python
# Add to isaac_ros_container composable_node_descriptions:

# DNN Image Encoder (preprocess image for TensorRT)
ComposableNode(
    package='isaac_ros_dnn_image_encoder',
    plugin='nvidia::isaac_ros::dnn_inference::DnnImageEncoderNode',
    name='dnn_image_encoder',
    parameters=[{
        'network_image_width': 960,
        'network_image_height': 544,
        'image_mean': [0.5, 0.5, 0.5],
        'image_stddev': [0.5, 0.5, 0.5],
    }],
    remappings=[
        ('image', '/camera/color/image_raw'),
        ('camera_info', '/camera/color/camera_info'),
        ('tensor', '/tensor_pub'),
    ]
),

# TensorRT inference node
ComposableNode(
    package='isaac_ros_tensor_rt',
    plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
    name='tensor_rt',
    parameters=[{
        'model_file_path': '/workspaces/isaac_ros-dev/models/peoplenet/resnet34_peoplenet_int8.onnx',
        'engine_file_path': '/workspaces/isaac_ros-dev/models/peoplenet/resnet34_peoplenet_int8.plan',
        'input_tensor_names': ['input_tensor'],
        'input_binding_names': ['input_1'],
        'output_tensor_names': ['output_cov', 'output_bbox'],
        'output_binding_names': ['output_cov/Sigmoid', 'output_bbox/BiasAdd'],
        'verbose': False,
        'force_engine_update': False,
    }]
),

# DetectNet decoder (convert tensor to bounding boxes)
ComposableNode(
    package='isaac_ros_detectnet',
    plugin='nvidia::isaac_ros::detectnet::DetectNetDecoderNode',
    name='detectnet_decoder',
    parameters=[{
        'confidence_threshold': 0.6,  # TUNE: 0.5 (default), 0.7 (fewer false positives)
        'iou_threshold': 0.4,         # TUNE: NMS threshold
        'enable_confidence_threshold': True,
        'enable_bbox_area_threshold': True,
        'bbox_area_threshold': 100,   # TUNE: Minimum bbox area (pixels²)
    }],
    remappings=[
        ('tensor_pub', '/tensor_pub'),
        ('detectnet/detections', '/detections'),
    ]
),
```

**Confidence Threshold Tuning Guide**:
- `0.3-0.5`: High recall, more false positives (good for exploration/mapping)
- `0.5-0.6`: Balanced (default for most applications)
- `0.6-0.8`: High precision, fewer false positives (good for critical safety applications)
- `0.8+`: Very conservative (may miss true positives)

**Performance targets** (Jetson Orin Nano 8GB + PeopleNet):
- Latency: 25-35ms per frame (640x480 input)
- Memory: ~1.2GB GPU
- Throughput: 25-30 FPS
- Precision: 85-90% (indoor, well-lit)

---

### Step 5: Configure Depth Processing for Navigation

**What the skill does**:
- Adds Isaac ROS Depth Image Proc for point cloud generation
- Configures depth filtering (temporal, spatial, decimation)
- Tunes obstacle detection parameters
- Integrates with Nav2 costmap (optional)

**Key tuning parameters**:
- `depth_scale`: `0.001` (RealSense default, mm to m)
- `min_depth`: `0.3` (meters, filter out noise < 30cm)
- `max_depth`: `10.0` (meters, max range for indoor navigation)
- `decimation_factor`: `2` (reduce point cloud density, 2x faster)

**Deliverable**:
```python
# Add to isaac_ros_container composable_node_descriptions:
ComposableNode(
    package='isaac_ros_depth_image_proc',
    plugin='nvidia::isaac_ros::depth_image_proc::PointCloudXyzNode',
    name='point_cloud_xyz',
    parameters=[{
        'use_sim_time': use_sim_time,
        'queue_size': 10,
    }],
    remappings=[
        ('image_rect', '/camera/depth/image_rect_raw'),
        ('camera_info', '/camera/depth/camera_info'),
        ('points', '/points'),
    ]
),
```

**Optional: Depth filtering node** (reduce noise):
```python
Node(
    package='isaac_ros_depth_image_proc',
    executable='depth_image_filter_node',
    name='depth_filter',
    parameters=[{
        'min_depth': 0.3,
        'max_depth': 10.0,
        'filter_nan': True,
        'filter_inf': True,
        'decimation_factor': 2,
    }],
    remappings=[
        ('depth', '/camera/depth/image_rect_raw'),
        ('depth_filtered', '/camera/depth/image_rect_filtered'),
    ]
)
```

---

### Step 6: Create Tuning Configuration Files

**What the skill does**:
- Generates YAML config files for each perception package
- Documents all tunable parameters with comments
- Creates tuning presets (low-latency, balanced, high-accuracy)

**Deliverable**:
```yaml
# config/isaac_perception_tuning.yaml
cuVSLAM:
  performance_profile: "balanced"  # low_latency | balanced | high_accuracy

  low_latency:
    max_feature_count: 200
    min_num_images: 2
    enable_image_denoising: false
    loop_closure_frequency: 5
    target_latency_ms: 30

  balanced:
    max_feature_count: 300
    min_num_images: 4
    enable_image_denoising: true
    loop_closure_frequency: 10
    target_latency_ms: 50

  high_accuracy:
    max_feature_count: 500
    min_num_images: 6
    enable_image_denoising: true
    loop_closure_frequency: 20
    target_latency_ms: 80

DetectNet:
  model: "peoplenet"  # peoplenet | trafficcamnet | custom

  # Confidence threshold tuning (adjust for false positive rate)
  confidence_threshold: 0.6  # Range: 0.3-0.9
  iou_threshold: 0.4         # NMS, range: 0.3-0.5
  bbox_area_threshold: 100   # Min bbox area (pixels²)

  # TensorRT optimization
  tensorrt_precision: "INT8"  # FP32 | FP16 | INT8
  max_batch_size: 1
  workspace_size_mb: 512

  # Performance targets
  target_fps: 25
  max_latency_ms: 40

DepthProcessing:
  # Depth range (meters)
  min_depth: 0.3
  max_depth: 10.0

  # Filtering
  decimation_factor: 2  # 1 (no decimation) | 2 | 4
  spatial_filter_enable: true
  temporal_filter_enable: true
  hole_filling_enable: true

  # Point cloud generation
  point_cloud_downsample_voxel_size: 0.05  # meters

HardwareLimits:
  jetson_model: "Orin Nano 8GB"
  max_gpu_memory_gb: 7.5
  max_cpu_memory_gb: 6.0
  thermal_throttle_temp_celsius: 80
  target_power_watts: 15  # 7W (low), 15W (default), 25W (max for Orin Nano)
```

---

### Step 7: Create Validation and Benchmarking Scripts

**What the skill does**:
- Generates Python script to measure latency, FPS, memory
- Creates ROS 2 test node to log perception metrics
- Adds plotting tools for visualization

**Deliverable**:
```python
# scripts/benchmark_perception.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
import time
import psutil
import subprocess

class PerceptionBenchmark(Node):
    def __init__(self):
        super().__init__('perception_benchmark')

        # Subscribers
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.odom_callback, 10)

        # Metrics
        self.image_count = 0
        self.detection_count = 0
        self.odom_count = 0
        self.last_image_time = None
        self.last_detection_time = None
        self.last_odom_time = None

        # Timer for logging
        self.create_timer(5.0, self.log_metrics)

        self.get_logger().info("Perception benchmark started")

    def image_callback(self, msg):
        self.image_count += 1
        now = time.time()
        if self.last_image_time:
            latency = now - self.last_image_time
            self.get_logger().info(f"Image latency: {latency*1000:.1f} ms")
        self.last_image_time = now

    def detection_callback(self, msg):
        self.detection_count += 1
        num_detections = len(msg.detections)
        now = time.time()
        if self.last_detection_time:
            latency = now - self.last_detection_time
            self.get_logger().info(f"Detection latency: {latency*1000:.1f} ms, {num_detections} objects detected")
        self.last_detection_time = now

    def odom_callback(self, msg):
        self.odom_count += 1
        now = time.time()
        if self.last_odom_time:
            latency = now - self.last_odom_time
            self.get_logger().info(f"Odometry latency: {latency*1000:.1f} ms")
        self.last_odom_time = now

    def log_metrics(self):
        # GPU memory (NVIDIA Jetson)
        try:
            result = subprocess.run(['tegrastats', '--interval', '1000', '--logfile', '/tmp/tegra.log'],
                                    capture_output=True, text=True, timeout=1.5)
        except:
            pass

        # CPU/RAM
        cpu_percent = psutil.cpu_percent(interval=1)
        ram = psutil.virtual_memory()

        self.get_logger().info(f"""
        ===== Perception Metrics =====
        Images received: {self.image_count}
        Detections received: {self.detection_count}
        Odometry received: {self.odom_count}
        CPU usage: {cpu_percent:.1f}%
        RAM usage: {ram.percent:.1f}% ({ram.used / 1e9:.1f} GB / {ram.total / 1e9:.1f} GB)
        ==============================
        """)

def main():
    rclpy.init()
    node = PerceptionBenchmark()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage**:
```bash
# Terminal 1: Launch perception pipeline
ros2 launch my_robot isaac_perception.launch.py

# Terminal 2: Run benchmark
ros2 run my_robot benchmark_perception.py

# Expected output:
# Image latency: 33.5 ms
# Detection latency: 28.3 ms, 2 objects detected
# Odometry latency: 45.2 ms
# CPU usage: 42.3%
# RAM usage: 65.1% (5.2 GB / 8.0 GB)
```

---

### Step 8: Generate Validation Report

**What the skill does**:
- Documents perception performance metrics
- Provides tuning recommendations based on hardware
- Suggests next steps (sim-to-real transfer, dataset collection)

**Deliverable**:
```markdown
# Isaac ROS Perception Tuning Report

**Date**: 2025-12-05
**Hardware**: Jetson Orin Nano 8GB
**Camera**: RealSense D435 (640x480 @ 30 FPS)
**Isaac ROS Version**: 2.1.0

---

## Configuration Summary

### cuVSLAM
- Profile: Balanced
- Max features: 300
- Min images: 4
- Denoising: Enabled
- Loop closure: 10 Hz

### DetectNet (PeopleNet)
- Model: PeopleNet (ResNet34 INT8)
- Confidence threshold: 0.6
- IOU threshold: 0.4
- Precision: INT8

### Depth Processing
- Range: 0.3-10.0 m
- Decimation: 2x
- Filtering: Spatial + Temporal + Hole Filling

---

## Performance Results

| Metric | Target | Measured | Status |
|--------|--------|----------|--------|
| cuVSLAM latency | <60 ms | 48.3 ms | ✅ PASS |
| DetectNet latency | <40 ms | 31.2 ms | ✅ PASS |
| Total pipeline latency | <100 ms | 79.5 ms | ✅ PASS |
| Detection FPS | >20 FPS | 28.1 FPS | ✅ PASS |
| GPU memory usage | <6 GB | 4.8 GB | ✅ PASS |
| CPU usage | <60% | 43.2% | ✅ PASS |

---

## Detection Quality (Indoor Office, 10 min run)

- **True Positives**: 187 detections
- **False Positives**: 12 detections (6.4% FP rate)
- **False Negatives**: 5 missed detections (2.6% FN rate)
- **Precision**: 94.0%
- **Recall**: 97.4%

---

## Localization Quality (50m trajectory)

- **Absolute Trajectory Error (ATE)**: 0.83 m (1.66% of path length)
- **Relative Pose Error (RPE)**: 0.021 m/m (2.1% drift)
- **Loop Closures Detected**: 4
- **Map Landmarks**: 1,247 features

---

## Recommendations

### 1. Confidence Threshold Tuning
- **Current**: 0.6 (94% precision, 6.4% FP rate)
- **Recommendation**: Acceptable for navigation. If false positives are problematic, increase to 0.65-0.70.

### 2. Performance Optimization
- cuVSLAM is running well within latency budget (48ms < 60ms target)
- Consider enabling IMU fusion for better odometry in texture-poor environments
- Thermal throttling not observed (max temp 68°C < 80°C threshold)

### 3. Next Steps
- Collect indoor navigation dataset for sim-to-real validation
- Test in low-light conditions (consider IR emitter duty cycle tuning)
- Integrate with Nav2 for autonomous navigation

---

## Configuration Files

All tuned configuration files available at:
- `launch/isaac_perception.launch.py`
- `config/isaac_perception_tuning.yaml`
- `scripts/benchmark_perception.py`

Run validation:
```bash
ros2 launch my_robot isaac_perception.launch.py
ros2 run my_robot benchmark_perception.py
```

---

**Status**: ✅ Perception pipeline ready for deployment
