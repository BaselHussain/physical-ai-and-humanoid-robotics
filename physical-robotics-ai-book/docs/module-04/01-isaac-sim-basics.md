# Isaac Sim 2025 Basics: Setup, ROS 2 Bridge, and Humanoid Loading

This section will guide you through setting up NVIDIA Isaac Sim 2025 with optimal performance settings to achieve ≥60 FPS with humanoid robots, configuring the ROS 2 bridge for seamless communication, and loading humanoid robot models.

## Prerequisites

Before starting with Isaac Sim 2025, ensure you have:

- NVIDIA RTX 4070 Ti or equivalent/higher GPU
- Ubuntu 22.04 LTS
- CUDA 12.x installed
- Isaac Sim 2025.1 or later
- ROS 2 Humble Hawksbill
- Isaac ROS core packages

## Installing Isaac Sim 2025

### Option 1: Omniverse Launcher (Recommended)

1. Download the NVIDIA Omniverse Launcher from the [NVIDIA Developer website](https://developer.nvidia.com/isaac-sim)
2. Create a free NVIDIA Developer account if you don't have one
3. Install the Omniverse Launcher
4. Through the launcher, search for and install Isaac Sim 2025
5. Launch Isaac Sim and follow the initial setup wizard

### Option 2: Docker (Alternative)

```bash
# Pull the Isaac Sim 2025 Docker image
docker pull nvcr.io/nvidia/isaac-sim:2025.1.0-hotfix1

# Run Isaac Sim with GPU support
docker run --gpus all -it --rm \
  --network=host \
  --env "ACCEPT_EULA=Y" \
  --env "ISAACSIM_LICENSE_FILE=/isaac-sim/ov/pkg/isaac_sim/python/omni.isaac.sim.python.gpl-2025.1.0.r1.eula.txt" \
  --volume $HOME/.nvidia-omniverse:/root/.nvidia-omniverse \
  --volume $HOME/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  --volume $HOME/isaac-sim/cache/ov:/root/.cache/ov:rw \
  --volume $HOME/isaac-sim/cache/pip:/root/.cache/pip:rw \
  --volume $HOME/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  --volume $HOME/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
  --volume $HOME/isaac-sim/data:/isaac-sim/ov/data:rw \
  --volume $HOME/isaac-sim/extensions:/isaac-sim/ov/pkg/isaac_sim-2025.1.0-hotfix1/extensions:rw \
  nvcr.io/nvidia/isaac-sim:2025.1.0-hotfix1
```

## Performance Optimization for ≥60 FPS

### Graphics Settings

To achieve ≥60 FPS with humanoid robots in Isaac Sim 2025:

1. Open Isaac Sim and go to **Window → Stage → Renderer Settings**
2. Set the following performance parameters:

```
# Renderer Configuration for ≥60 FPS
Renderer Settings:
- Renderer: Kit Renderer (default)
- Render Mode: Render to Prims
- Render Quality: High Performance
- Level of Detail: 0.5 (adjust based on scene complexity)
- Dynamic Batching: Enabled
- GPU Instancing: Enabled

Viewport Settings:
- Resolution: 1280x720 (for development) or 1920x1080 (for final testing)
- Refresh Rate: 60Hz or higher
- Multi-Sample: 4x (balance quality and performance)

Physics Settings:
- Time Step: 1/60 second (0.016667)
- Substeps: 1 (minimum for performance)
- Solver Iterations: 4 (balance stability and performance)
```

### Scene Optimization

For optimal humanoid robot performance:

```python
# Example Python script to optimize scene settings for humanoid performance
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Configure physics for humanoid performance
world = World(stage_units_in_meters=1.0)
world.physics_sim_params.set_physics_dt(1.0/60.0, substeps=1)
world.set_max_deviations(0.01, 0.01)

# Enable GPU dynamics if available
world.set_gpu_dynamic_flag(True)
```

### GPU Optimization

1. Ensure your NVIDIA GPU drivers are up to date (550.x or higher recommended)
2. In Isaac Sim, go to **Window → Compute → GPU Settings**
3. Enable all GPU acceleration features:
   - GPU dynamics
   - GPU particles
   - GPU Caching

## Configuring the ROS 2 Bridge

### Installing Isaac ROS Bridge

```bash
# Add NVIDIA's package repository
curl -sSL https://repos.lgsvl.com/setup | sudo bash -

# Install Isaac ROS common packages
sudo apt update
sudo apt install ros-humble-isaac-ros-common

# Install Isaac Sim ROS bridge
sudo apt install ros-humble-isaac-ros-isaac-sim-bridge
```

### Setting up ROS 2 Environment

```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source Isaac ROS workspace if installed
source /opt/ros/humble/isaac_ros-dev_ws/install/setup.bash

# Set ROS domain ID (optional, for network isolation)
export ROS_DOMAIN_ID=10
```

### Configuring Isaac Sim ROS Bridge

1. In Isaac Sim, go to **Window → Extensions → Isaac ROS Bridge**
2. Enable the ROS Bridge extension
3. Configure the bridge settings:

```
# ROS Bridge Configuration
ROS Bridge Settings:
- Transport: TCP (default)
- Port: 10500 (default)
- Message Buffer Size: 1000000 bytes
- Enable TF Publishing: True
- TF Publish Rate: 100 Hz
- Enable Clock Publishing: True
- Clock Publish Rate: 100 Hz
```

4. Alternatively, create a configuration file `isaac_ros_bridge_config.yaml`:

```yaml
isaac_ros_bridge:
  ros_bridge:
    ros_domain_id: 10
    transport: "tcp"
    port: 10500
    message_buffer_size: 1000000
    enable_tf_publishing: true
    tf_publish_rate: 100.0
    enable_clock_publishing: true
    clock_publish_rate: 100.0
```

## Loading Humanoid Robot Models

### Using Built-in Humanoid Models

Isaac Sim 2025 includes several humanoid models optimized for performance:

1. Go to **Window → Isaac Examples → Robotics → Humanoid**
2. Select a humanoid model (Atlas, ATRIAS, or custom humanoid)
3. Drag the model into your stage

### Custom Humanoid Model Loading

To load custom humanoid models:

```bash
# If using custom URDF models, place them in Isaac Sim's assets folder
mkdir -p $HOME/.nvidia-omniverse/Assets/humanoid_models
cp your_humanoid.urdf $HOME/.nvidia-omniverse/Assets/humanoid_models/
```

### Optimizing Humanoid for Performance

For ≥60 FPS performance with humanoid robots:

1. Limit the number of rigid bodies in the humanoid model
2. Use simplified collision meshes for physics calculations
3. Set appropriate joint limits and damping parameters
4. Use GPU instancing for multiple humanoid instances

```python
# Example code to load and optimize a humanoid robot
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage

# Load humanoid with optimized settings
world = World(stage_units_in_meters=1.0)
humanoid = world.scene.add(
    Robot(
        prim_path="/World/Humanoid",
        name="my_humanoid",
        usd_path="path/to/humanoid.usd",  # Optimized USD file
        position=[0, 0, 1.0],
        orientation=[0, 0, 0, 1]
    )
)

# Optimize physics for humanoid
humanoid.set_solver_position_iteration_count(4)
humanoid.set_solver_velocity_iteration_count(4)
```

## Performance Validation

To validate that you're achieving ≥60 FPS:

1. In Isaac Sim, open the **Profiler** window (`Window → Profiler → Profiler`)
2. Monitor the frame rate in the status bar
3. Use the following Python snippet to check FPS programmatically:

```python
import carb
from omni.isaac.core.utils.viewports import get_current_viewport_height, get_current_viewport_width

# Get current FPS
current_fps = carb.app.get_app().get_framework().get_average_fps()
print(f"Current FPS: {current_fps}")

# Validate performance target
if current_fps >= 60:
    print("✅ Performance target achieved: ≥60 FPS")
else:
    print(f"⚠️  Performance target not met: {current_fps} FPS < 60 FPS")
```

## Troubleshooting Performance Issues

If you're not achieving ≥60 FPS:

1. **Reduce scene complexity**: Remove unnecessary objects from the scene
2. **Lower rendering quality**: Temporarily reduce render quality settings
3. **Check GPU usage**: Monitor GPU utilization with `nvidia-smi`
4. **Optimize physics**: Reduce physics substeps and solver iterations
5. **Update drivers**: Ensure latest NVIDIA GPU drivers are installed

## Next Steps

Once you've successfully set up Isaac Sim 2025 with ≥60 FPS performance, proceed to:

1. Configure the ROS 2 bridge with proper topic mappings
2. Load and calibrate your humanoid robot model
3. Test basic joint control and sensor data publishing
4. Validate the Isaac Sim ↔ ROS 2 communication pipeline

The next section will cover configuring the ROS 2 bridge with proper topic mappings for Isaac Sim ↔ ROS 2 communication.