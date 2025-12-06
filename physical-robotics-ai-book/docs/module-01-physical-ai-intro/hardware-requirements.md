---
title: Hardware Requirements & Budget Planning
sidebar_label: Hardware
sidebar_position: 7
description: Complete hardware guide with 3-tier workstation options, Jetson development kits, and optional robot hardware for Physical AI development
---

# Hardware Requirements & Budget Planning

## Overview

This guide helps you select the right hardware for Physical AI development based on your learning goals and budget. All prices are current as of **December 2025**.

---

## Development Workstation (Required)

### 3-Tier Workstation Matrix

| Component | **Entry Tier** | **Mid Tier** | **High-End Tier** |
|-----------|----------------|--------------|-------------------|
| **GPU** | NVIDIA RTX 4060 Ti (16GB) | NVIDIA RTX 4070 Ti (12GB) | NVIDIA RTX 4080 Super (16GB) |
| **CPU** | Intel i5-13400F (10 cores) | AMD Ryzen 7 7700X (8 cores) | AMD Ryzen 9 7950X (16 cores) |
| **RAM** | 32 GB DDR5-5200 | 32 GB DDR5-6000 | 64 GB DDR5-6400 |
| **Storage** | 1 TB NVMe Gen4 | 2 TB NVMe Gen4 | 2 TB NVMe Gen5 + 4 TB SATA SSD |
| **Motherboard** | B760 / B650 | Z790 / X670 | Z790 / X670E |
| **PSU** | 750W 80+ Gold | 850W 80+ Gold | 1000W 80+ Platinum |
| **Cooling** | Tower air cooler | 240mm AIO | 360mm AIO |
| **Case** | Mid-tower ATX | Mid-tower ATX | Full-tower ATX |
| **Total Price** | **$1,499** (2025-12) | **$2,799** (2025-12) | **$5,499** (2025-12) |

### Use Cases by Tier

**Entry Tier ($1,499)**:
- ✅ ROS 2 development (Nav2, MoveIt2)
- ✅ Gazebo simulation (1-2 robots)
- ✅ Isaac Sim (basic scenes, < 10 objects)
- ✅ Module 01-06 (all core modules)
- ❌ Large-scale Isaac Sim scenes (>20 objects)
- ❌ Multi-robot swarms (>3 robots)

**Best For**: Students, hobbyists, learning Physical AI fundamentals

---

**Mid Tier ($2,799)**:
- ✅ All Entry Tier capabilities
- ✅ Isaac Sim (complex scenes, 20+ objects)
- ✅ Domain randomization training (100+ variations)
- ✅ Multi-robot sim (3-5 robots)
- ✅ Module 01-09 (including advanced topics)
- ✅ LLM inference (local LLaMA-2 13B)

**Best For**: Professional robotics engineers, research labs, small companies

---

**High-End Tier ($5,499)**:
- ✅ All Mid Tier capabilities
- ✅ Large-scale Isaac Sim (50+ robots, warehouse sims)
- ✅ Real-time photorealistic rendering (RTX ray tracing)
- ✅ Multi-modal VLA training (RT-2, PaLM-E fine-tuning)
- ✅ Production deployment (Docker swarms, CI/CD)
- ✅ Module 01-10 + capstone projects

**Best For**: Advanced researchers, robotics startups, production teams

---

## Jetson Development Kits (Optional, for Edge Deployment)

Deploy trained models to edge hardware for real-world robots.

| Model | **Jetson Orin Nano 8GB** | **Jetson Orin NX 16GB** |
|-------|--------------------------|-------------------------|
| **AI Performance** | 40 TOPS (INT8) | 100 TOPS (INT8) |
| **GPU** | 1024-core NVIDIA Ampere | 1024-core NVIDIA Ampere |
| **CPU** | 6-core Arm Cortex-A78AE | 8-core Arm Cortex-A78AE |
| **Memory** | 8 GB LPDDR5 | 16 GB LPDDR5 |
| **Storage** | microSD (64 GB included) | NVMe SSD slot (not included) |
| **Power** | 7W - 15W | 10W - 25W |
| **Camera Support** | Up to 4x cameras | Up to 6x cameras |
| **Price** | **$499** (2025-12) | **$899** (2025-12) |

### Use Cases by Jetson Model

**Jetson Orin Nano 8GB ($499)**:
- ✅ Isaac ROS cuVSLAM (stereo SLAM at 20 FPS)
- ✅ DetectNet (single-class detection at 25 FPS)
- ✅ RealSense D435 integration
- ✅ Mobile robots (TurtleBot 4, wheeled platforms)
- ❌ Multi-camera perception (3+ cameras)
- ❌ High-res detection (>1080p)

**Best For**: Entry-level edge deployment, wheeled mobile robots, single-camera applications

---

**Jetson Orin NX 16GB ($899)**:
- ✅ All Orin Nano capabilities
- ✅ Multi-camera SLAM (3-4 cameras at 30 FPS)
- ✅ Multi-object detection (10+ classes at 30 FPS)
- ✅ VLA model inference (quantized RT-2)
- ✅ Humanoid robots (Figure 01, Unitree H1)
- ✅ Dexterous manipulation

**Best For**: Professional robotics, humanoids, multi-camera systems, production deployment

---

### When to Buy Jetson Hardware

**Buy Jetson if**:
- You plan to deploy to real robots (TurtleBot 4, Unitree, custom hardware)
- You're working on Module 06+ (sim-to-real transfer)
- You need to test edge performance (latency, power consumption)
- Your project requires autonomous operation (no tethered workstation)

**Skip Jetson if**:
- You're only doing Modules 01-05 (simulation-focused)
- Budget is tight (< $2,000 total)
- No real robot hardware available yet

---

## Optional Robot Hardware

Pre-built robots compatible with ROS 2 Humble for hands-on learning.

| Robot | **TurtleBot 4** | **Unitree Go2** |
|-------|-----------------|-----------------|
| **Type** | Wheeled Mobile Robot | Quadruped Robot |
| **Locomotion** | Differential drive | Legged (dynamic walking) |
| **Sensors** | RealSense D435, 2D lidar, IMU, encoders | 5x cameras, IMU, foot force sensors |
| **Payload** | 2 kg (add arm optional) | 5 kg (not designed for manipulation) |
| **Battery Life** | 4-6 hours | 1-2 hours (active locomotion) |
| **ROS 2 Support** | ✅ Native ROS 2 Humble | ✅ ROS 2 Humble (community support) |
| **Use Cases** | Navigation, mapping, delivery | Locomotion research, uneven terrain |
| **Price** | **$1,695** (2025-12) | **$1,600** (2025-12) |
| **Vendor** | [Clearpath Robotics](https://clearpathrobotics.com/turtlebot-4/) \| [Archive](https://archive.is/PLACEHOLDER_TB4_VENDOR) | [Unitree Robotics](https://www.unitree.com/go2) \| [Archive](https://archive.is/PLACEHOLDER_UNITREE_VENDOR) |

### Use Cases by Robot

**TurtleBot 4 ($1,695)**:
- ✅ Nav2 stack testing (real-world navigation)
- ✅ SLAM algorithm validation (cuVSLAM, cartographer)
- ✅ Object detection + manipulation (with arm upgrade)
- ✅ Multi-robot coordination (buy 2-3 units)
- ✅ Sim-to-real transfer validation

**Best For**: Navigation research, wheeled robot development, budget-conscious teams

---

**Unitree Go2 ($1,600)**:
- ✅ Bipedal/quadruped locomotion research
- ✅ Uneven terrain navigation
- ✅ Dynamic balance algorithms
- ✅ High-speed locomotion (up to 3.7 m/s)
- ❌ Manipulation (no arm, not designed for grasping)

**Best For**: Locomotion research, outdoor robotics, legged robot development

---

### When to Buy Robot Hardware

**Buy robot hardware if**:
- Your learning goal includes sim-to-real transfer (Module 06+)
- You have budget for both workstation + robot ($3,000+ total)
- You're working in a research lab or robotics company
- You want to publish papers (real-world validation required)

**Skip robot hardware if**:
- You're learning fundamentals (Modules 01-05)
- Budget is limited (< $3,000 total)
- Focus is on software/algorithm development (sim is sufficient)
- No physical space for robot testing

---

## Complete Budget Scenarios

### Scenario 1: Budget Learner ($1,499)
**Goal**: Learn Physical AI fundamentals, complete Modules 01-06

**Hardware**:
- Entry Tier Workstation: $1,499

**Capabilities**:
- ROS 2 development
- Gazebo + Isaac Sim (basic scenes)
- cuVSLAM, DetectNet perception
- LLM orchestration (cloud APIs)

**Limitations**:
- Cannot deploy to edge (no Jetson)
- Cannot validate on real robot
- Limited multi-robot simulation

---

### Scenario 2: Serious Student ($2,298)
**Goal**: Complete all 10 modules, edge deployment

**Hardware**:
- Entry Tier Workstation: $1,499
- Jetson Orin Nano 8GB: $499
- RealSense D435 camera: $300

**Capabilities**:
- All Scenario 1 capabilities
- Edge deployment (Jetson)
- Sim-to-real transfer testing
- Custom robot integration

**Limitations**:
- No real robot hardware (sim-only validation)
- Limited Jetson performance (single camera)

---

### Scenario 3: Professional Engineer ($4,493)
**Goal**: Production robotics development, real-world deployment

**Hardware**:
- Mid Tier Workstation: $2,799
- TurtleBot 4: $1,695

**Capabilities**:
- Complex Isaac Sim scenes (20+ objects)
- Real-world navigation validation
- Multi-robot simulation (3-5 robots)
- Production-ready deployment

**Limitations**:
- No high-performance edge compute (use TurtleBot's onboard Raspberry Pi)
- No legged locomotion (wheeled robot only)

---

### Scenario 4: Research Lab / Startup ($7,897)
**Goal**: Advanced research, multi-robot systems, production deployment

**Hardware**:
- High-End Tier Workstation: $5,499
- Jetson Orin NX 16GB: $899
- TurtleBot 4: $1,695
- Additional sensors (cameras, lidars): ~$800

**Capabilities**:
- Large-scale simulations (50+ robots)
- Multi-camera perception (4-6 cameras)
- VLA model fine-tuning
- Production deployment pipeline
- Real-world validation

**No Limitations**: Can complete all 10 modules + advanced research

---

## Vendor Recommendations & Links

### Workstation Components

**GPUs**:
- NVIDIA Store: [nvidia.com/shop](https://www.nvidia.com/shop/) | [Archive](https://archive.is/PLACEHOLDER_NVIDIA_SHOP)
- Newegg: [newegg.com](https://www.newegg.com/) | [Archive](https://archive.is/PLACEHOLDER_NEWEGG)
- Amazon: [amazon.com](https://www.amazon.com/) | [Archive](https://archive.is/PLACEHOLDER_AMAZON)

**Pre-Built Workstations** (if DIY is not preferred):
- NVIDIA Certified Systems: [nvidia.com/certified-systems](https://www.nvidia.com/en-us/data-center/products/certified-systems/) | [Archive](https://archive.is/PLACEHOLDER_NVIDIA_CERTIFIED)
- System76 (Linux pre-installed): [system76.com](https://system76.com/) | [Archive](https://archive.is/PLACEHOLDER_SYSTEM76)

### Jetson Development Kits

- NVIDIA Developer Store: [developer.nvidia.com/embedded/buy/jetson-orin-nano-devkit](https://developer.nvidia.com/embedded/buy/jetson-orin-nano-devkit) | [Archive](https://archive.is/PLACEHOLDER_JETSON_BUY)
- Authorized Distributors: Arrow, Digi-Key, Mouser

### Robot Hardware

- TurtleBot 4: [clearpathrobotics.com/turtlebot-4](https://clearpathrobotics.com/turtlebot-4/) | [Archive](https://archive.is/PLACEHOLDER_TB4_VENDOR)
- Unitree Go2: [unitree.com/go2](https://www.unitree.com/go2) | [Archive](https://archive.is/PLACEHOLDER_UNITREE_VENDOR)

---

## Additional Hardware Considerations

### Cameras (for Jetson Integration)

| Camera | Resolution | Frame Rate | Price | Use Case |
|--------|------------|------------|-------|----------|
| RealSense D435 | 1280x720 depth | 30 FPS | $300 | SLAM, manipulation |
| RealSense D455 | 1280x720 depth | 30 FPS (longer range) | $380 | Outdoor navigation |
| ZED 2i | 2208x1242 stereo | 60 FPS | $450 | High-res SLAM |

### Network Hardware

- **Router**: WiFi 6 (802.11ax) for low-latency ROS 2 communication
- **Switch**: Gigabit Ethernet (1 Gbps minimum) for multi-robot setups

---

## Summary: Which Tier is Right for You?

| Your Goal | Recommended Hardware | Total Budget |
|-----------|---------------------|--------------|
| Learn basics (Modules 01-04) | Entry Workstation | $1,499 |
| Complete all modules | Entry Workstation + Jetson Nano | $2,298 |
| Professional development | Mid Workstation + TurtleBot 4 | $4,493 |
| Research / Production | High-End Workstation + Jetson NX + Robot | $7,897 |

---

**Recommendation**: Start with **Entry Tier Workstation** ($1,499) for Modules 01-04. Upgrade to Jetson + robot hardware when you reach Module 06 (sim-to-real transfer).

---

## Next Steps

✅ Reviewed 3 workstation tiers
✅ Compared Jetson development kits
✅ Evaluated optional robot hardware
✅ Selected budget scenario

**Next**: [Next Steps & References](./next-steps-references.md) - Complete Module 01 and preview Module 02

---

## External Resources

- [NVIDIA Jetson Comparison](https://developer.nvidia.com/embedded/jetson-modules) | [Archive](https://archive.is/PLACEHOLDER_JETSON_COMPARE)
- [PCPartPicker Build Guides](https://pcpartpicker.com/) | [Archive](https://archive.is/PLACEHOLDER_PCPARTPICKER)
- [r/robotics Hardware Recommendations](https://www.reddit.com/r/robotics/wiki/resources#wiki_hardware) | [Archive](https://archive.is/PLACEHOLDER_REDDIT_ROBOTICS)

---

**Previous**: [Verification & Testing](./verification-testing.md) | **Next**: [Next Steps & References](./next-steps-references.md)
