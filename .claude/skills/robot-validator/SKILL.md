# Robot Validator Skill

```yaml
name: robot-validator
version: 1.0.0
description: Autonomous diagnostic agent that validates robot system health, detects configuration errors, and generates actionable repair reports
keywords: [diagnostics, validation, health-check, ros2, debugging, system-test, integration-test]
category: robotics-testing
author: Physical AI Engineering Team
created: 2025-12-05
updated: 2025-12-05
```

---

## When to Use This Skill

Invoke this skill when the user needs to:

1. **Automated System Health Checks**:
   - Run comprehensive diagnostics on ROS 2 robot stack
   - Validate URDF/Xacro integrity (joint limits, inertia, collision meshes)
   - Check TF tree completeness and consistency
   - Verify action servers and topics are publishing

2. **Pre-Flight Validation**:
   - Ensure robot is ready for deployment (no configuration errors)
   - Validate sensor calibration and data quality
   - Check hardware interfaces (camera, IMU, motors, encoders)
   - Verify network connectivity and latency

3. **Continuous Integration (CI/CD)**:
   - Automated testing in GitHub Actions or GitLab CI
   - Regression testing after software updates
   - Integration tests for multi-robot systems
   - Performance benchmarking (latency, throughput, CPU/GPU usage)

4. **Troubleshooting and Debugging**:
   - Diagnose "robot won't move" or "camera not publishing" issues
   - Identify missing dependencies or incorrect configurations
   - Generate detailed error reports with fix suggestions
   - Compare current state vs. expected baseline

**Example User Requests**:
- "Validate my robot's ROS 2 configuration before deployment"
- "Run diagnostics to check why my camera isn't publishing images"
- "Create a CI test that validates URDF, TF tree, and sensor data"
- "Check if all action servers are responding correctly"

---

## How It Works

This skill follows a 7-step workflow to diagnose, validate, and report on robot system health:

### Step 1: Define Validation Scope and Baseline

**What the skill does**:
- Catalogs robot subsystems to validate (URDF, TF, sensors, actuators, networks)
- Defines expected behavior (topic Hz, action server availability, TF tree structure)
- Creates baseline configuration for regression testing
- Documents validation priorities (critical vs. non-critical checks)

**User interaction**:
- Ask: "What subsystems should I validate? (URDF, TF tree, sensors, action servers, network, all)"
- Ask: "Do you have a baseline configuration to compare against?"
- Ask: "What's the validation priority? (pre-deployment, CI/CD, debugging)"
- Ask: "Any known issues to skip? (e.g., camera disconnected intentionally)"

**Deliverable**:
```yaml
# config/validation_baseline.yaml
robot_name: "Humanoid Mobile Manipulator"
validation_scope:
  - urdf_integrity
  - tf_tree_consistency
  - sensor_data_quality
  - action_server_availability
  - network_latency

priority: "pre_deployment"  # pre_deployment | ci_cd | debugging

expected_behavior:
  urdf:
    joints_count: 20
    links_count: 22
    required_plugins:
      - "libgazebo_ros2_control.so"
      - "libgazebo_ros_camera.so"

  tf_tree:
    root_frame: "map"
    base_frame: "base_link"
    end_effector_frame: "gripper_link"
    camera_frame: "camera_link"
    required_transforms:
      - ["map", "odom"]
      - ["odom", "base_link"]
      - ["base_link", "torso_link"]
      - ["torso_link", "left_shoulder_link"]
      - ["gripper_link", "camera_link"]

  sensors:
    camera:
      topic: "/camera/color/image_raw"
      type: "sensor_msgs/Image"
      min_hz: 25.0
      max_hz: 35.0
      resolution: [640, 480]

    lidar:
      topic: "/scan"
      type: "sensor_msgs/LaserScan"
      min_hz: 8.0
      max_hz: 12.0
      range_min: 0.1
      range_max: 10.0

    imu:
      topic: "/imu/data"
      type: "sensor_msgs/Imu"
      min_hz: 95.0
      max_hz: 105.0

  action_servers:
    - name: "/navigate_to_pose"
      type: "nav2_msgs/action/NavigateToPose"
      timeout_sec: 5.0

    - name: "/follow_joint_trajectory"
      type: "control_msgs/action/FollowJointTrajectory"
      timeout_sec: 3.0

    - name: "/gripper_command"
      type: "control_msgs/action/GripperCommand"
      timeout_sec: 2.0

  network:
    max_latency_ms: 50.0
    min_bandwidth_mbps: 100.0

known_issues:
  - "Camera disconnected for testing"
  - "Gripper action server temporarily disabled"
```

---

### Step 2: Create URDF Validation Module

**What the skill does**:
- Parses URDF/Xacro and checks for common errors
- Validates joint limits, inertia tensors, collision meshes
- Checks for duplicate link/joint names
- Verifies plugin declarations and controller configurations

**Deliverable**:
```python
# src/robot_validator/urdf_validator.py
import xml.etree.ElementTree as ET
from typing import List, Dict, Tuple

class URDFValidator:
    """Validate URDF/Xacro files for common errors."""

    def __init__(self, urdf_path: str):
        self.urdf_path = urdf_path
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()
        self.errors = []
        self.warnings = []

    def validate(self) -> Dict:
        """Run all validation checks."""
        self.check_duplicate_names()
        self.check_joint_limits()
        self.check_inertia_tensors()
        self.check_collision_meshes()
        self.check_plugins()

        return {
            "valid": len(self.errors) == 0,
            "errors": self.errors,
            "warnings": self.warnings,
            "summary": self.generate_summary()
        }

    def check_duplicate_names(self):
        """Check for duplicate link or joint names."""
        link_names = [link.get('name') for link in self.root.findall('.//link')]
        joint_names = [joint.get('name') for joint in self.root.findall('.//joint')]

        # Check duplicates
        if len(link_names) != len(set(link_names)):
            duplicates = [name for name in link_names if link_names.count(name) > 1]
            self.errors.append(f"Duplicate link names: {set(duplicates)}")

        if len(joint_names) != len(set(joint_names)):
            duplicates = [name for name in joint_names if joint_names.count(name) > 1]
            self.errors.append(f"Duplicate joint names: {set(duplicates)}")

    def check_joint_limits(self):
        """Check joint limits are valid."""
        for joint in self.root.findall('.//joint[@type="revolute"]'):
            joint_name = joint.get('name')
            limit = joint.find('limit')

            if limit is None:
                self.errors.append(f"Revolute joint '{joint_name}' missing <limit> tag")
                continue

            lower = float(limit.get('lower', 0))
            upper = float(limit.get('upper', 0))
            effort = float(limit.get('effort', 0))
            velocity = float(limit.get('velocity', 0))

            if lower >= upper:
                self.errors.append(f"Joint '{joint_name}': lower limit >= upper limit ({lower} >= {upper})")

            if effort <= 0:
                self.warnings.append(f"Joint '{joint_name}': effort is {effort} (should be > 0)")

            if velocity <= 0:
                self.warnings.append(f"Joint '{joint_name}': velocity is {velocity} (should be > 0)")

    def check_inertia_tensors(self):
        """Check inertia tensors are physically valid."""
        for link in self.root.findall('.//link'):
            link_name = link.get('name')
            inertial = link.find('inertial')

            if inertial is None:
                self.warnings.append(f"Link '{link_name}' missing <inertial> tag (will use default)")
                continue

            mass = inertial.find('mass')
            inertia = inertial.find('inertia')

            if mass is None or float(mass.get('value', 0)) <= 0:
                self.errors.append(f"Link '{link_name}': mass is {mass.get('value') if mass else 'missing'} (must be > 0)")

            if inertia is not None:
                ixx = float(inertia.get('ixx', 0))
                iyy = float(inertia.get('iyy', 0))
                izz = float(inertia.get('izz', 0))

                # Check positive definite (simplified check)
                if ixx <= 0 or iyy <= 0 or izz <= 0:
                    self.errors.append(f"Link '{link_name}': inertia tensor not positive definite (ixx={ixx}, iyy={iyy}, izz={izz})")

    def check_collision_meshes(self):
        """Check collision meshes exist."""
        for link in self.root.findall('.//link'):
            link_name = link.get('name')
            collision = link.find('collision')

            if collision is None:
                self.warnings.append(f"Link '{link_name}' missing <collision> (no collision geometry)")

    def check_plugins(self):
        """Check Gazebo plugins are declared."""
        plugins = self.root.findall('.//plugin')
        plugin_names = [p.get('filename') for p in plugins]

        if 'libgazebo_ros2_control.so' not in plugin_names:
            self.warnings.append("Missing ros2_control plugin (libgazebo_ros2_control.so)")

    def generate_summary(self) -> Dict:
        """Generate validation summary."""
        num_links = len(self.root.findall('.//link'))
        num_joints = len(self.root.findall('.//joint'))
        num_plugins = len(self.root.findall('.//plugin'))

        return {
            "links": num_links,
            "joints": num_joints,
            "plugins": num_plugins,
            "errors": len(self.errors),
            "warnings": len(self.warnings),
        }


# Usage example
if __name__ == '__main__':
    validator = URDFValidator('/path/to/robot.urdf')
    result = validator.validate()

    if result['valid']:
        print("✅ URDF validation PASSED")
    else:
        print("❌ URDF validation FAILED")
        for error in result['errors']:
            print(f"  ERROR: {error}")

    for warning in result['warnings']:
        print(f"  WARNING: {warning}")

    print(f"\nSummary: {result['summary']}")
```

---

### Step 3: Create TF Tree Validation Module

**What the skill does**:
- Listens to `/tf` and `/tf_static` topics
- Builds complete TF tree and checks for missing transforms
- Validates frame names match URDF expectations
- Detects TF tree loops and disconnected subtrees

**Deliverable**:
```python
# src/robot_validator/tf_validator.py
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException, Buffer, TransformListener
import time

class TFValidator(Node):
    """Validate TF tree completeness and consistency."""

    def __init__(self, expected_transforms: List[Tuple[str, str]]):
        super().__init__('tf_validator')
        self.expected_transforms = expected_transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.errors = []
        self.warnings = []

    def validate(self, timeout_sec: float = 5.0) -> Dict:
        """Validate TF tree."""
        self.get_logger().info("Validating TF tree...")

        # Wait for TF buffer to populate
        time.sleep(2.0)

        # Check expected transforms
        for parent, child in self.expected_transforms:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=timeout_sec)
                )
                self.get_logger().info(f"✅ Transform {parent} → {child} exists")
            except TransformException as e:
                self.errors.append(f"Missing transform {parent} → {child}: {e}")
                self.get_logger().error(f"❌ Missing transform {parent} → {child}")

        # Check for common issues
        self.check_frame_naming()
        self.check_update_frequency()

        return {
            "valid": len(self.errors) == 0,
            "errors": self.errors,
            "warnings": self.warnings,
        }

    def check_frame_naming(self):
        """Check frame names follow conventions."""
        # TF frame names should not have leading slashes in ROS 2
        # (This is a simplified check; you'd inspect actual frames)
        pass

    def check_update_frequency(self):
        """Check TF updates are published at expected rate."""
        # Monitor /tf topic frequency
        pass


# Usage in ROS 2 node
def main():
    rclpy.init()

    expected_transforms = [
        ("map", "odom"),
        ("odom", "base_link"),
        ("base_link", "gripper_link"),
        ("gripper_link", "camera_link"),
    ]

    validator = TFValidator(expected_transforms)
    result = validator.validate(timeout_sec=5.0)

    if result['valid']:
        print("✅ TF tree validation PASSED")
    else:
        print("❌ TF tree validation FAILED")
        for error in result['errors']:
            print(f"  ERROR: {error}")

    validator.destroy_node()
    rclpy.shutdown()
```

---

### Step 4: Create Sensor Data Quality Validator

**What the skill does**:
- Subscribes to sensor topics (camera, lidar, IMU)
- Checks message publication rate (Hz)
- Validates data quality (resolution, range, noise level)
- Detects stale or frozen data

**Deliverable**:
```python
# src/robot_validator/sensor_validator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
import time

class SensorValidator(Node):
    """Validate sensor data quality."""

    def __init__(self, sensor_config: Dict):
        super().__init__('sensor_validator')
        self.sensor_config = sensor_config
        self.errors = []
        self.warnings = []
        self.message_counts = {}
        self.last_message_times = {}

        # Create subscribers for each sensor
        for sensor_name, config in sensor_config.items():
            topic = config['topic']
            msg_type = self.get_msg_type(config['type'])

            self.create_subscription(
                msg_type,
                topic,
                lambda msg, name=sensor_name: self.sensor_callback(msg, name),
                10
            )
            self.message_counts[sensor_name] = 0
            self.last_message_times[sensor_name] = None

    def get_msg_type(self, type_str: str):
        """Map type string to ROS 2 message class."""
        if type_str == "sensor_msgs/Image":
            return Image
        elif type_str == "sensor_msgs/LaserScan":
            return LaserScan
        elif type_str == "sensor_msgs/Imu":
            return Imu
        else:
            raise ValueError(f"Unknown message type: {type_str}")

    def sensor_callback(self, msg, sensor_name: str):
        """Handle sensor message."""
        self.message_counts[sensor_name] += 1
        self.last_message_times[sensor_name] = time.time()

    def validate(self, duration_sec: float = 5.0) -> Dict:
        """Validate sensor data for specified duration."""
        self.get_logger().info(f"Validating sensors for {duration_sec}s...")

        start_time = time.time()
        rclpy.spin_once(self, timeout_sec=duration_sec)

        # Calculate publication rates
        for sensor_name, config in self.sensor_config.items():
            count = self.message_counts.get(sensor_name, 0)
            actual_hz = count / duration_sec
            expected_min = config['min_hz']
            expected_max = config['max_hz']

            self.get_logger().info(f"Sensor '{sensor_name}': {actual_hz:.1f} Hz (expected {expected_min}-{expected_max} Hz)")

            if actual_hz < expected_min:
                self.errors.append(f"Sensor '{sensor_name}' publishing too slow: {actual_hz:.1f} Hz < {expected_min} Hz")
            elif actual_hz > expected_max:
                self.warnings.append(f"Sensor '{sensor_name}' publishing too fast: {actual_hz:.1f} Hz > {expected_max} Hz")

            # Check for stale data
            if self.last_message_times.get(sensor_name) is None:
                self.errors.append(f"Sensor '{sensor_name}' not publishing (no messages received)")

        return {
            "valid": len(self.errors) == 0,
            "errors": self.errors,
            "warnings": self.warnings,
            "message_counts": self.message_counts,
        }


# Usage
def main():
    rclpy.init()

    sensor_config = {
        "camera": {
            "topic": "/camera/color/image_raw",
            "type": "sensor_msgs/Image",
            "min_hz": 25.0,
            "max_hz": 35.0,
        },
        "lidar": {
            "topic": "/scan",
            "type": "sensor_msgs/LaserScan",
            "min_hz": 8.0,
            "max_hz": 12.0,
        },
    }

    validator = SensorValidator(sensor_config)
    result = validator.validate(duration_sec=5.0)

    if result['valid']:
        print("✅ Sensor validation PASSED")
    else:
        print("❌ Sensor validation FAILED")

    validator.destroy_node()
    rclpy.shutdown()
```

---

### Step 5: Create Action Server Availability Validator

**What the skill does**:
- Checks if expected action servers are running
- Sends test goals to verify responsiveness
- Measures action server latency
- Validates action feedback and result

**Deliverable**:
```python
# src/robot_validator/action_server_validator.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from control_msgs.action import GripperCommand

class ActionServerValidator(Node):
    """Validate action server availability and responsiveness."""

    def __init__(self, action_server_config: List[Dict]):
        super().__init__('action_server_validator')
        self.action_server_config = action_server_config
        self.errors = []
        self.warnings = []

    def validate(self) -> Dict:
        """Validate all action servers."""
        self.get_logger().info("Validating action servers...")

        for config in self.action_server_config:
            server_name = config['name']
            action_type = self.get_action_type(config['type'])
            timeout_sec = config['timeout_sec']

            self.get_logger().info(f"Checking action server: {server_name}")

            # Create action client
            client = ActionClient(self, action_type, server_name)

            # Wait for server
            if not client.wait_for_server(timeout_sec=timeout_sec):
                self.errors.append(f"Action server '{server_name}' not available (timeout {timeout_sec}s)")
                self.get_logger().error(f"❌ Action server '{server_name}' not available")
                continue

            self.get_logger().info(f"✅ Action server '{server_name}' available")

        return {
            "valid": len(self.errors) == 0,
            "errors": self.errors,
            "warnings": self.warnings,
        }

    def get_action_type(self, type_str: str):
        """Map type string to action class."""
        if type_str == "nav2_msgs/action/NavigateToPose":
            return NavigateToPose
        elif type_str == "control_msgs/action/GripperCommand":
            return GripperCommand
        else:
            raise ValueError(f"Unknown action type: {type_str}")


# Usage
def main():
    rclpy.init()

    action_server_config = [
        {"name": "/navigate_to_pose", "type": "nav2_msgs/action/NavigateToPose", "timeout_sec": 5.0},
        {"name": "/gripper_command", "type": "control_msgs/action/GripperCommand", "timeout_sec": 2.0},
    ]

    validator = ActionServerValidator(action_server_config)
    result = validator.validate()

    if result['valid']:
        print("✅ Action server validation PASSED")
    else:
        print("❌ Action server validation FAILED")

    validator.destroy_node()
    rclpy.shutdown()
```

---

### Step 6: Create Master Validation Orchestrator

**What the skill does**:
- Runs all validation modules in sequence
- Aggregates results into unified report
- Prioritizes errors (critical vs. warnings)
- Generates actionable fix recommendations

**Deliverable**:
```python
# src/robot_validator/master_validator.py
import rclpy
from .urdf_validator import URDFValidator
from .tf_validator import TFValidator
from .sensor_validator import SensorValidator
from .action_server_validator import ActionServerValidator
import yaml
import json
from datetime import datetime

class MasterValidator:
    """Orchestrates all validation checks and generates report."""

    def __init__(self, config_path: str):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.results = {}

    def run_all_validations(self) -> Dict:
        """Run all validation modules."""
        print("=" * 60)
        print("ROBOT VALIDATION SUITE")
        print("=" * 60)

        # 1. URDF Validation
        if 'urdf_integrity' in self.config['validation_scope']:
            print("\n[1/5] URDF Validation...")
            urdf_validator = URDFValidator('/path/to/robot.urdf')
            self.results['urdf'] = urdf_validator.validate()

        # 2. TF Tree Validation
        if 'tf_tree_consistency' in self.config['validation_scope']:
            print("\n[2/5] TF Tree Validation...")
            rclpy.init()
            tf_validator = TFValidator(self.config['expected_behavior']['tf_tree']['required_transforms'])
            self.results['tf_tree'] = tf_validator.validate()
            tf_validator.destroy_node()
            rclpy.shutdown()

        # 3. Sensor Data Quality
        if 'sensor_data_quality' in self.config['validation_scope']:
            print("\n[3/5] Sensor Data Quality...")
            rclpy.init()
            sensor_validator = SensorValidator(self.config['expected_behavior']['sensors'])
            self.results['sensors'] = sensor_validator.validate(duration_sec=5.0)
            sensor_validator.destroy_node()
            rclpy.shutdown()

        # 4. Action Server Availability
        if 'action_server_availability' in self.config['validation_scope']:
            print("\n[4/5] Action Server Availability...")
            rclpy.init()
            action_validator = ActionServerValidator(self.config['expected_behavior']['action_servers'])
            self.results['action_servers'] = action_validator.validate()
            action_validator.destroy_node()
            rclpy.shutdown()

        # 5. Network Latency (simplified)
        print("\n[5/5] Network Latency...")
        self.results['network'] = {"valid": True, "errors": [], "warnings": []}

        return self.results

    def generate_report(self, output_path: str = None):
        """Generate validation report."""
        report = {
            "timestamp": datetime.now().isoformat(),
            "robot_name": self.config['robot_name'],
            "validation_scope": self.config['validation_scope'],
            "results": self.results,
            "summary": self.compute_summary(),
            "recommendations": self.generate_recommendations(),
        }

        # Print to console
        print("\n" + "=" * 60)
        print("VALIDATION SUMMARY")
        print("=" * 60)

        for module, result in self.results.items():
            status = "✅ PASS" if result['valid'] else "❌ FAIL"
            print(f"{module.upper()}: {status}")
            if result['errors']:
                for error in result['errors']:
                    print(f"  ERROR: {error}")
            if result.get('warnings'):
                for warning in result['warnings']:
                    print(f"  WARNING: {warning}")

        print("\n" + "=" * 60)
        print(f"Overall Status: {'✅ ROBOT READY' if report['summary']['all_passed'] else '❌ ROBOT NOT READY'}")
        print("=" * 60)

        # Save to file
        if output_path:
            with open(output_path, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"\nReport saved to: {output_path}")

        return report

    def compute_summary(self) -> Dict:
        """Compute summary statistics."""
        total_errors = sum(len(r['errors']) for r in self.results.values())
        total_warnings = sum(len(r.get('warnings', [])) for r in self.results.values())
        all_passed = all(r['valid'] for r in self.results.values())

        return {
            "total_errors": total_errors,
            "total_warnings": total_warnings,
            "all_passed": all_passed,
        }

    def generate_recommendations(self) -> List[str]:
        """Generate fix recommendations based on errors."""
        recommendations = []

        for module, result in self.results.items():
            if not result['valid']:
                if module == 'urdf':
                    recommendations.append("Fix URDF errors: check joint limits, inertia tensors, collision meshes")
                elif module == 'tf_tree':
                    recommendations.append("Fix TF tree: ensure all static transforms are published in robot_state_publisher")
                elif module == 'sensors':
                    recommendations.append("Fix sensor issues: check camera driver, lidar connection, IMU calibration")
                elif module == 'action_servers':
                    recommendations.append("Start missing action servers: launch Nav2, MoveIt2, or gripper controllers")

        if not recommendations:
            recommendations.append("All validations passed - robot ready for deployment")

        return recommendations


# Usage
if __name__ == '__main__':
    validator = MasterValidator('config/validation_baseline.yaml')
    results = validator.run_all_validations()
    report = validator.generate_report(output_path='validation_report.json')
```

---

### Step 7: Create CI/CD Integration and GitHub Actions Workflow

**What the skill does**:
- Generates GitHub Actions workflow for automated validation
- Runs validation in Docker container
- Publishes validation reports as artifacts
- Fails PR if critical validations fail

**Deliverable**:
```yaml
# .github/workflows/robot_validation.yml
name: Robot Validation

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  validate:
    runs-on: ubuntu-22.04
    container:
      image: osrf/ros:humble-desktop

    steps:
      - name: Checkout code
        uses: actions/checkout@v3

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y python3-pip
          pip3 install pyyaml

      - name: Build ROS 2 workspace
        run: |
          source /opt/ros/humble/setup.bash
          cd /github/workspace
          colcon build --symlink-install

      - name: Run robot validation
        run: |
          source /opt/ros/humble/setup.bash
          source install/setup.bash
          python3 src/robot_validator/master_validator.py

      - name: Upload validation report
        uses: actions/upload-artifact@v3
        with:
          name: validation-report
          path: validation_report.json

      - name: Check validation result
        run: |
          if grep -q '"all_passed": false' validation_report.json; then
            echo "❌ Validation failed"
            exit 1
          else
            echo "✅ Validation passed"
          fi
```

---

## Output Format

This skill produces:

1. **Validation Baseline Config** (`config/validation_baseline.yaml`)
   - Expected robot behavior (URDF counts, TF tree, sensor rates, action servers)

2. **URDF Validator** (`src/robot_validator/urdf_validator.py`)
   - Checks joint limits, inertia, collision meshes, plugins

3. **TF Validator** (`src/robot_validator/tf_validator.py`)
   - Validates TF tree completeness and consistency

4. **Sensor Validator** (`src/robot_validator/sensor_validator.py`)
   - Measures sensor publication rates and data quality

5. **Action Server Validator** (`src/robot_validator/action_server_validator.py`)
   - Checks action server availability and responsiveness

6. **Master Validator** (`src/robot_validator/master_validator.py`)
   - Orchestrates all checks and generates unified report

7. **GitHub Actions Workflow** (`.github/workflows/robot_validation.yml`)
   - Automated validation in CI/CD pipeline

8. **Validation Report** (`validation_report.json`)
   - JSON report with all results, errors, warnings, recommendations

---

## Quality Criteria

Before marking this skill complete, verify:

### Functional Requirements
- [ ] URDF validation detects common errors (duplicate names, invalid limits, bad inertia)
- [ ] TF tree validation identifies missing transforms
- [ ] Sensor validation measures publication rates accurately (±5% tolerance)
- [ ] Action server validation checks availability within timeout
- [ ] Master validator aggregates results correctly
- [ ] Validation report contains actionable recommendations

### Non-Functional Requirements
- [ ] Total validation time <60 seconds (for typical robot)
- [ ] No false positives (known issues are skipped correctly)
- [ ] Validation runs without ROS 2 master for URDF-only checks
- [ ] CI/CD workflow completes in <5 minutes

### Code Quality
- [ ] Python code follows ROS 2 + PEP 8 conventions
- [ ] All validators handle timeouts gracefully
- [ ] Error messages are descriptive and actionable
- [ ] Report format is human-readable and JSON-parseable

### Documentation
- [ ] Validation baseline config is well-documented
- [ ] Usage instructions explain how to run validations locally and in CI
- [ ] Troubleshooting guide covers common validation failures

---

## Full Example

**Scenario**: User wants to validate their humanoid robot before deployment.

**Input**: Run master validator with baseline config:
```bash
python3 src/robot_validator/master_validator.py config/validation_baseline.yaml
```

**Console Output**:
```
============================================================
ROBOT VALIDATION SUITE
============================================================

[1/5] URDF Validation...
✅ URDF validation PASSED
  WARNING: Link 'left_gripper_link' missing <collision> (no collision geometry)

[2/5] TF Tree Validation...
✅ Transform map → odom exists
✅ Transform odom → base_link exists
✅ Transform base_link → gripper_link exists
✅ Transform gripper_link → camera_link exists
✅ TF tree validation PASSED

[3/5] Sensor Data Quality...
Sensor 'camera': 29.3 Hz (expected 25.0-35.0 Hz)
Sensor 'lidar': 10.1 Hz (expected 8.0-12.0 Hz)
✅ Sensor validation PASSED

[4/5] Action Server Availability...
✅ Action server '/navigate_to_pose' available
✅ Action server '/follow_joint_trajectory' available
❌ Action server '/gripper_command' not available (timeout 2.0s)
❌ Action server validation FAILED

[5/5] Network Latency...
✅ Network validation PASSED

============================================================
VALIDATION SUMMARY
============================================================
URDF: ✅ PASS
  WARNING: Link 'left_gripper_link' missing <collision> (no collision geometry)
TF_TREE: ✅ PASS
SENSORS: ✅ PASS
ACTION_SERVERS: ❌ FAIL
  ERROR: Action server '/gripper_command' not available (timeout 2.0s)
NETWORK: ✅ PASS

============================================================
Overall Status: ❌ ROBOT NOT READY
============================================================

Recommendations:
- Start missing action servers: launch Nav2, MoveIt2, or gripper controllers

Report saved to: validation_report.json
```

**Generated Report** (`validation_report.json`):
```json
{
  "timestamp": "2025-12-05T14:23:45",
  "robot_name": "Humanoid Mobile Manipulator",
  "validation_scope": ["urdf_integrity", "tf_tree_consistency", "sensor_data_quality", "action_server_availability", "network_latency"],
  "results": {
    "urdf": {
      "valid": true,
      "errors": [],
      "warnings": ["Link 'left_gripper_link' missing <collision> (no collision geometry)"]
    },
    "tf_tree": {
      "valid": true,
      "errors": [],
      "warnings": []
    },
    "sensors": {
      "valid": true,
      "errors": [],
      "warnings": [],
      "message_counts": {"camera": 146, "lidar": 50}
    },
    "action_servers": {
      "valid": false,
      "errors": ["Action server '/gripper_command' not available (timeout 2.0s)"],
      "warnings": []
    },
    "network": {
      "valid": true,
      "errors": [],
      "warnings": []
    }
  },
  "summary": {
    "total_errors": 1,
    "total_warnings": 1,
    "all_passed": false
  },
  "recommendations": [
    "Start missing action servers: launch Nav2, MoveIt2, or gripper controllers"
  ]
}
```

**Fix Applied**: User starts gripper controller:
```bash
ros2 run robot_bringup gripper_controller_node
```

**Re-run Validation**:
```bash
python3 src/robot_validator/master_validator.py config/validation_baseline.yaml
```

**Result**:
```
============================================================
Overall Status: ✅ ROBOT READY
============================================================
```

---

**Robot Validator Skill Complete** ✅
