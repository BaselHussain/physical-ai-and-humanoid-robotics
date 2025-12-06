---
title: Lab 3 - Services
sidebar_label: Lab 3 - Services
sidebar_position: 6
description: Implement request-response communication patterns using ROS 2 services, create SetJointAngle.srv with request/response fields, implement service server and client nodes
---

# Lab 3: Services

## Overview

In this lab, you'll learn to implement request-response communication patterns using ROS 2 services. You'll create a `SetJointAngle.srv` with request and response fields, implement service server and client nodes, and understand when to use services versus topics. Services are essential for synchronous operations where you need a guaranteed response to a request.

**Duration**: 1.5 hours

**Learning Objectives**:
- ✅ Define custom service types using `.srv` syntax
- ✅ Implement service servers that handle requests and return responses
- ✅ Create service clients that make requests and handle responses
- ✅ Validate service communication with `ros2 service` commands
- ✅ Implement SetJointAngle service with joint_name, target_angle, max_velocity request fields

---

## Prerequisites

Before starting this lab, ensure you have:

✅ **Completed Lab 1** - Talker/Listener basics
✅ **Completed Lab 2** - Custom messages
✅ **ROS 2 Humble installed** with all standard packages
✅ **Basic understanding** of ROS 2 nodes, topics, and custom messages
✅ **Python programming skills** for implementing service usage

---

## Step 1: Understanding ROS 2 Services

### What are Services?

**ROS 2 services** provide a request-response communication pattern where:
- A **service client** sends a request to a **service server**
- The server processes the request and returns a response
- This is synchronous communication (the client waits for the response)

### Service Definition Syntax

ROS 2 uses a simple syntax for service definitions:

```
# Request fields (above the --- separator)
string name                    # Request field
int32 age                      # Request field

---
# Response fields (below the --- separator)
bool success                   # Response field
string message                 # Response field
```

### SetJointAngle Service Requirements

Based on our spec, we need a `SetJointAngle.srv` with:
- **Request**: `joint_name` (string), `target_angle` (float64), `max_velocity` (float64)
- **Response**: `success` (bool), `message` (string), `final_angle` (float64)

---

## Step 2: Create Services Package

First, let's create a package for our services:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python lab03_services --dependencies rclpy std_msgs
```

---

## Step 3: Define the SetJointAngle Service

Create the services directory:
```bash
mkdir -p ~/ros2_ws/src/lab03_services/srv
```

Create the SetJointAngle service definition:
```bash
touch ~/ros2_ws/src/lab03_services/srv/SetJointAngle.srv
```

Add the following content to `~/ros2_ws/src/lab03_services/srv/SetJointAngle.srv`:

```
# SetJointAngle.srv: Request to set a joint to a specific angle
# Request: joint name, target angle, and maximum velocity
# Response: success flag, message, and final achieved angle

# Request fields
string joint_name            # Name of the joint to control
float64 target_angle         # Target angle in radians
float64 max_velocity         # Maximum velocity for movement

---
# Response fields
bool success                 # True if the operation was successful
string message               # Status message (e.g., "Joint moved successfully" or error reason)
float64 final_angle          # The final angle achieved (may differ from target if limits reached)
```

---

## Step 4: Update Package Configuration for Services

Edit the `setup.py` file to include service generation:

Edit `~/ros2_ws/src/lab03_services/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'lab03_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/SetJointAngle.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 services implementation with SetJointAngle service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server = lab03_services.service_server:main',
            'service_client = lab03_services.service_client:main',
        ],
    },
)
```

Wait, I need to update the package.xml to include service generation properly. Let me fix that:

Edit `~/ros2_ws/src/lab03_services/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lab03_services</name>
  <version>0.0.0</version>
  <description>ROS 2 services implementation with SetJointAngle service</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Actually, for services, we should use ament_cmake. Let me recreate the package properly:

```bash
cd ~/ros2_ws/src
rm -rf lab03_services
ros2 pkg create --build-type ament_cmake lab03_services
```

Now let's recreate the service files with proper CMake configuration:

Create the service definition again:
```bash
mkdir -p ~/ros2_ws/src/lab03_services/srv
touch ~/ros2_ws/src/lab03_services/srv/SetJointAngle.srv
```

Add the content to `~/ros2_ws/src/lab03_services/srv/SetJointAngle.srv`:

```
# SetJointAngle.srv: Request to set a joint to a specific angle
# Request: joint name, target angle, and maximum velocity
# Response: success flag, message, and final achieved angle

# Request fields
string joint_name            # Name of the joint to control
float64 target_angle         # Target angle in radians
float64 max_velocity         # Maximum velocity for movement

---
# Response fields
bool success                 # True if the operation was successful
string message               # Status message (e.g., "Joint moved successfully" or error reason)
float64 final_angle          # The final angle achieved (may differ from target if limits reached)
```

Update the CMakeLists.txt:

Edit `~/ros2_ws/src/lab03_services/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(lab03_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Define service files
set(srv_files
  "srv/SetJointAngle.srv"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_package()
```

Update the package.xml:

Edit `~/ros2_ws/src/lab03_services/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lab03_services</name>
  <version>0.0.0</version>
  <description>ROS 2 services implementation with SetJointAngle service</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <build_depend>rosidl_default_generators</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>rclpy</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## Step 5: Build the Services Package

```bash
cd ~/ros2_ws
colcon build --packages-select lab03_services
source ~/ros2_ws/install/setup.bash
```

---

## Step 6: Verify Service Generation

Check that the SetJointAngle service was generated correctly:

```bash
ros2 interface show lab03_services/srv/SetJointAngle
```

**Expected Output**:
```
string joint_name
float64 target_angle
float64 max_velocity
---
bool success
string message
float64 final_angle
```

You can also list all available services:
```bash
ros2 interface list | grep SetJointAngle
```

---

## Step 7: Create Service Server

Create a new directory for the Python nodes:
```bash
mkdir -p ~/ros2_ws/src/lab03_services/lab03_services
```

Create the service server file:
```bash
touch ~/ros2_ws/src/lab03_services/lab03_services/service_server.py
```

Add the following code to `~/ros2_ws/src/lab03_services/lab03_services/service_server.py`:

```python
#!/usr/bin/env python3
"""
Service server - implements SetJointAngle service to control robot joints
"""

import rclpy
from rclpy.node import Node
import math

# Import our custom service
from lab03_services.srv import SetJointAngle


class JointControlService(Node):

    def __init__(self):
        super().__init__('joint_control_service')

        # Create the service server
        self.srv = self.create_service(
            SetJointAngle,
            'set_joint_angle',
            self.set_joint_angle_callback
        )

        # Simulate joint limits and current positions
        self.joint_limits = {
            'shoulder_pitch': (-math.pi/2, math.pi/2),      # -90° to 90°
            'elbow': (0, math.pi),                          # 0° to 180°
            'wrist_roll': (-math.pi, math.pi),              # -180° to 180°
            'hip_pitch': (-math.pi/3, math.pi/3),           # -60° to 60°
            'knee': (0, math.pi/2),                         # 0° to 90°
        }

        # Current joint positions (starting position)
        self.current_positions = {
            'shoulder_pitch': 0.0,
            'elbow': 0.0,
            'wrist_roll': 0.0,
            'hip_pitch': 0.0,
            'knee': 0.0,
        }

        self.get_logger().info('Joint control service server started')

    def set_joint_angle_callback(self, request, response):
        """
        Callback function for the SetJointAngle service
        """
        joint_name = request.joint_name
        target_angle = request.target_angle
        max_velocity = request.max_velocity

        self.get_logger().info(
            f'Received request: set {joint_name} to {target_angle:.3f} rad '
            f'with max velocity {max_velocity:.3f}'
        )

        # Validate joint name
        if joint_name not in self.joint_limits:
            response.success = False
            response.message = f'Unknown joint: {joint_name}'
            response.final_angle = self.current_positions.get(joint_name, 0.0)
            return response

        # Validate max_velocity is positive
        if max_velocity <= 0:
            response.success = False
            response.message = f'Max velocity must be positive, got: {max_velocity}'
            response.final_angle = self.current_positions[joint_name]
            return response

        # Check joint limits
        min_limit, max_limit = self.joint_limits[joint_name]
        if target_angle < min_limit or target_angle > max_limit:
            response.success = False
            response.message = (
                f'Joint {joint_name} angle {target_angle:.3f} rad out of limits '
                f'[{min_limit:.3f}, {max_limit:.3f}] rad'
            )
            response.final_angle = self.current_positions[joint_name]
            return response

        # Simulate the movement (in a real robot, this would send commands to hardware)
        # For simulation, we'll just update the position
        self.current_positions[joint_name] = target_angle

        # In a real implementation, you might need to consider velocity limits
        # and simulate the time it takes to move

        response.success = True
        response.message = f'Joint {joint_name} successfully moved to {target_angle:.3f} rad'
        response.final_angle = target_angle

        self.get_logger().info(
            f'Service response: {response.message}, final angle: {response.final_angle:.3f}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    service_node = JointControlService()

    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        service_node.get_logger().info('Service server stopped')
    finally:
        service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 8: Create Service Client

Create the service client file:
```bash
touch ~/ros2_ws/src/lab03_services/lab03_services/service_client.py
```

Add the following code to `~/ros2_ws/src/lab03_services/lab03_services/service_client.py`:

```python
#!/usr/bin/env python3
"""
Service client - calls SetJointAngle service to control robot joints
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import time

# Import our custom service
from lab03_services.srv import SetJointAngle


class JointControlClient(Node):

    def __init__(self):
        super().__init__('joint_control_client')

        # Create the service client
        self.cli = self.create_client(SetJointAngle, 'set_joint_angle')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client connected')

    def send_request(self, joint_name, target_angle, max_velocity):
        """
        Send a request to the SetJointAngle service
        """
        request = SetJointAngle.Request()
        request.joint_name = joint_name
        request.target_angle = target_angle
        request.max_velocity = max_velocity

        self.get_logger().info(
            f'Sending request: set {joint_name} to {target_angle:.3f} rad '
            f'with max velocity {max_velocity:.3f}'
        )

        # Call the service asynchronously
        future = self.cli.call_async(request)

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)

        try:
            response = future.result()
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)

    client = JointControlClient()

    # Test cases for different joints
    test_cases = [
        ('shoulder_pitch', 0.5, 0.1),   # Move shoulder to 0.5 rad
        ('elbow', 1.0, 0.2),            # Move elbow to 1.0 rad
        ('wrist_roll', -0.3, 0.15),     # Move wrist to -0.3 rad
        ('hip_pitch', 0.2, 0.1),        # Move hip to 0.2 rad
        ('knee', 0.8, 0.25),            # Move knee to 0.8 rad
    ]

    for joint_name, target_angle, max_velocity in test_cases:
        response = client.send_request(joint_name, target_angle, max_velocity)

        if response is not None:
            if response.success:
                client.get_logger().info(
                    f'SUCCESS: {response.message}, final angle: {response.final_angle:.3f} rad'
                )
            else:
                client.get_logger().warn(
                    f'FAILED: {response.message}, final angle: {response.final_angle:.3f} rad'
                )
        else:
            client.get_logger().error(f'Failed to get response for {joint_name}')

        # Add a small delay between requests
        time.sleep(0.5)

    # Test error case: invalid joint
    client.get_logger().info('Testing invalid joint...')
    response = client.send_request('invalid_joint', 0.5, 0.1)
    if response is not None:
        client.get_logger().info(f'Response: success={response.success}, message="{response.message}"')

    # Test error case: out of limits
    client.get_logger().info('Testing out-of-limits angle...')
    response = client.send_request('shoulder_pitch', 2.0, 0.1)  # Should be out of limits
    if response is not None:
        client.get_logger().info(f'Response: success={response.success}, message="{response.message}"')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Step 9: Update Package Configuration for Executables

Add the Python files to the setup.py:

Edit `~/ros2_ws/src/lab03_services/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'lab03_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 services implementation with SetJointAngle service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server = lab03_services.service_server:main',
            'service_client = lab03_services.service_client:main',
        ],
    },
)
```

---

## Step 10: Build the Updated Package

```bash
cd ~/ros2_ws
colcon build --packages-select lab03_services
source ~/ros2_ws/install/setup.bash
```

---

## Step 11: Test the Service

Run the service server in one terminal:

**Terminal 1 - Service Server**:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run lab03_services service_server
```

In another terminal, run the service client:

**Terminal 2 - Service Client** (open a new terminal):
```bash
source ~/ros2_ws/install/setup.bash
ros2 run lab03_services service_client
```

You should see the client sending requests and the server responding with success or failure messages.

---

## Step 12: Use Command Line Tools with Services

ROS 2 command line tools work with services too:

```bash
# List all available services
ros2 service list
```

```bash
# Get info about a specific service
ros2 service info /set_joint_angle
```

```bash
# Call a service directly from command line
ros2 service call /set_joint_angle lab03_services/srv/SetJointAngle "{joint_name: 'shoulder_pitch', target_angle: 0.5, max_velocity: 0.1}"
```

```bash
# Check the service type
ros2 service type /set_joint_angle
```

---

## Step 13: Advanced Service Patterns

### Service with Complex Data Types

Services can use complex data types including arrays and nested messages:

```
# ComplexService.srv
geometry_msgs/Pose target_pose
float64[] joint_trajectory
string[] joint_names
---
bool[] success_array
geometry_msgs/Transform result_transform
```

### Service with Validation

In real applications, you should add comprehensive validation:

```python
def set_joint_angle_callback(self, request, response):
    # Validate all inputs
    if not self.is_valid_joint_name(request.joint_name):
        response.success = False
        response.message = f'Invalid joint name: {request.joint_name}'
        return response

    # Check for safety constraints
    if self.would_cause_collision(request.joint_name, request.target_angle):
        response.success = False
        response.message = f'Motion would cause collision'
        return response

    # Perform the action
    try:
        result = self.move_joint_safely(request.joint_name, request.target_angle, request.max_velocity)
        response.success = True
        response.message = 'Success'
        response.final_angle = result
    except Exception as e:
        response.success = False
        response.message = f'Error during movement: {str(e)}'
        response.final_angle = self.current_positions[request.joint_name]

    return response
```

---

## Step 14: Services vs Topics Comparison

| Aspect | Services | Topics |
|--------|----------|---------|
| **Communication Pattern** | Request-Response (synchronous) | Publish-Subscribe (asynchronous) |
| **Use Case** | Actions that require a response | Continuous data streams |
| **Examples** | Set joint angle, get robot pose, save map | Sensor data, robot state, commands |
| **Performance** | Higher latency, guaranteed delivery | Lower latency, best-effort delivery |
| **Blocking** | Client blocks until response | Non-blocking, continuous flow |

### When to Use Services

Use services when you need:
- **Synchronous communication** with guaranteed response
- **Action confirmation** (e.g., "move succeeded/failed")
- **Computational requests** (e.g., "plan a path from A to B")
- **Configuration changes** (e.g., "set parameter X to value Y")

### When to Use Topics

Use topics when you need:
- **Continuous data streams** (sensors, state)
- **Real-time performance** (low latency)
- **Many-to-many communication** (broadcast)
- **Fire-and-forget commands** (velocity commands)

---

## Step 15: Service Integration Test

Let's create a simple integration test to validate our service. Create a test file:

```bash
mkdir -p ~/ros2_ws/src/lab03_services/test
touch ~/ros2_ws/src/lab03_services/test/test_service_integration.py
```

Add the following code to `~/ros2_ws/src/lab03_services/test/test_service_integration.py`:

```python
#!/usr/bin/env python3
"""
Integration test for SetJointAngle service
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
import time

# Import our custom service
from lab03_services.srv import SetJointAngle


class ServiceIntegrationTest(Node):

    def __init__(self):
        super().__init__('service_integration_test')

        # Create the service client
        self.cli = self.create_client(SetJointAngle, 'set_joint_angle')

        self.get_logger().info('Service integration test node created')

    def test_valid_request(self):
        """Test a valid service request"""
        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # Create request
        request = SetJointAngle.Request()
        request.joint_name = 'shoulder_pitch'
        request.target_angle = 0.5
        request.max_velocity = 0.1

        # Call service
        future = self.cli.call_async(request)

        # Wait for response
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=5.0)
        finally:
            executor.shutdown()

        response = future.result()

        if response is not None:
            success = response.success and abs(response.final_angle - 0.5) < 0.001
            self.get_logger().info(f'Valid request test: {"PASSED" if success else "FAILED"}')
            return success
        else:
            self.get_logger().error('Valid request test: FAILED - No response')
            return False

    def test_invalid_joint(self):
        """Test request with invalid joint name"""
        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        # Create request
        request = SetJointAngle.Request()
        request.joint_name = 'invalid_joint'
        request.target_angle = 0.5
        request.max_velocity = 0.1

        # Call service
        future = self.cli.call_async(request)

        # Wait for response
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            executor.spin_until_future_complete(future, timeout_sec=5.0)
        finally:
            executor.shutdown()

        response = future.result()

        if response is not None:
            success = not response.success  # Should fail
            self.get_logger().info(f'Invalid joint test: {"PASSED" if success else "FAILED"}')
            return success
        else:
            self.get_logger().error('Invalid joint test: FAILED - No response')
            return False


def main(args=None):
    rclpy.init(args=args)

    test_node = ServiceIntegrationTest()

    # Run tests
    test1_passed = test_node.test_valid_request()
    time.sleep(1)  # Small delay between tests
    test2_passed = test_node.test_invalid_joint()

    all_passed = test1_passed and test2_passed
    test_node.get_logger().info(f'All tests: {"PASSED" if all_passed else "FAILED"}')

    test_node.destroy_node()
    rclpy.shutdown()

    return 0 if all_passed else 1


if __name__ == '__main__':
    exit(main())
```


Add the test to the setup.py entry points:

Edit `~/ros2_ws/src/lab03_services/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'lab03_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 services implementation with SetJointAngle service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server = lab03_services.service_server:main',
            'service_client = lab03_services.service_client:main',
            'test_service = lab03_services.test_service_integration:main',
        ],
    },
)
```

Wait, I made an error. The test file is in the test directory, not in the main package. Let me fix the setup.py:

```python
from setuptools import find_packages, setup

package_name = 'lab03_services'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 services implementation with SetJointAngle service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server = lab03_services.service_server:main',
            'service_client = lab03_services.service_client:main',
        ],
    },
)
```

---

## Step 16: Build and Test the Service Package

```bash
cd ~/ros2_ws
colcon build --packages-select lab03_services
source ~/ros2_ws/install/setup.bash
```

---

## Step 17: Best Practices for Services

### Service Design Guidelines

1. **Keep Requests Small**: Services should not transfer large amounts of data
2. **Define Clear Contracts**: Document request/response behavior clearly
3. **Handle Errors Gracefully**: Always return meaningful error messages
4. **Validate Inputs**: Check all request parameters before processing
5. **Use Descriptive Names**: Choose clear, consistent service names

### Performance Considerations

- **Response Time**: Services should respond quickly (under 1 second for most use cases)
- **Blocking Nature**: Be aware that service calls block the client
- **Timeout Handling**: Always implement proper timeout handling in clients
- **Concurrency**: Service servers should handle multiple requests efficiently

### Common Service Patterns

**Configuration Service**:
```
# Request: parameter name and value
string param_name
string param_value
---
# Response: success and error message
bool success
string error_msg
```

**Status Check Service**:
```
# Request: (empty)
---
# Response: system status
bool system_ok
string status_msg
float64 battery_level
```

**Calibration Service**:
```
# Request: calibration type
string calibration_type
---
# Response: success and results
bool success
string result_msg
float64[] calibration_values
```

---

## Step 18: Troubleshooting Common Service Issues

### Issue 1: Service Not Found

**Symptoms**: `Service not available, waiting again...`

**Solutions**:
```bash
# Check if service is running
ros2 service list

# Check service status
ros2 service info /set_joint_angle

# Make sure both server and client are using the same service name
```

### Issue 2: Client Times Out

**Symptoms**: Client waits indefinitely or times out

**Solutions**:
```python
# Add timeout to service calls
future = self.cli.call_async(request)
rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

if future.done():
    response = future.result()
else:
    self.get_logger().error('Service call timed out')
```

### Issue 3: Import Errors

**Symptoms**: `ImportError` when trying to import service definition

**Solutions**:
- Make sure the service package is built: `colcon build --packages-select lab03_services`
- Ensure the package is sourced: `source ~/ros2_ws/install/setup.bash`
- Check that the package.xml includes proper dependencies

---

## Lab Summary

In this lab, you've successfully:

✅ **Defined a custom SetJointAngle service** with request/response fields
✅ **Implemented a service server** that handles joint control requests
✅ **Created a service client** that makes requests and handles responses
✅ **Validated service communication** with `ros2 service` commands
✅ **Understood service vs topic** communication patterns

### Key Takeaways

- **Services** provide synchronous request-response communication
- **Service definitions** use `.srv` files with request/response separation
- **Service servers** implement callbacks to process requests
- **Service clients** make requests and wait for responses
- **Services vs Topics** have different use cases based on communication needs

---

## Next Steps

Now that you understand services, you're ready to explore:

- **Lab 4**: Actions for long-running tasks with feedback
- **Lab 5**: URDF fundamentals for robot modeling
- **Lab 6**: TF2 coordinate frames and transforms

**Continue to [Lab 4: Actions](./lab04-actions.md)**

---
**Previous**: [Lab 2: Custom Messages](./lab02-custom-messages.md) | **Next**: [Lab 4: Actions](./lab04-actions.md)