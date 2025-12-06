# Contract: Hardware Matrix Table Schema

**Purpose**: Defines the structure and validation rules for the 2025 hardware procurement matrix.

**Scope**: Applies to tables in `docs/module-01-physical-ai-intro/hardware-requirements.md`

---

## Table 1: Workstation Configurations (3 Tiers)

### Schema

| Column Name        | Data Type | Required | Format/Constraints                                      |
|--------------------|-----------|----------|--------------------------------------------------------|
| Tier               | String    | Yes      | "Entry", "Mid", or "High-End"                          |
| GPU Model          | String    | Yes      | Full model name + VRAM (e.g., "RTX 4070 Ti 12GB")     |
| CPU                | String    | Yes      | Model + core count (e.g., "i7-14700K (20 cores)")     |
| RAM                | String    | Yes      | Capacity in GB (e.g., "32GB DDR5")                     |
| Storage            | String    | Yes      | Type + capacity (e.g., "1TB NVMe SSD")                 |
| Price (2025-12)    | String    | Yes      | USD with date stamp (e.g., "$2,799 (2025-12)")         |
| Vendor Links       | Markdown  | Yes      | Primary + archive links (see format below)             |
| Use Cases          | String    | Yes      | Comma-separated list (≤100 chars total)                |

### Example Row

```markdown
| Mid       | RTX 4070 Ti 12GB | i7-14700K (20 cores) | 32GB DDR5 | 1TB NVMe SSD | $2,799 (2025-12) | [NVIDIA](https://nvidia.com/...) ([archive](https://archive.is/...)) | Complex Isaac Sim scenes, multi-robot simulation, moderate VLM fine-tuning |
```

### Validation Rules

1. **Tier Count**: Exactly 3 rows (Entry, Mid, High-End)
2. **Price Format**:
   - MUST include `$` symbol
   - MUST include date stamp `(2025-12)` in parentheses
   - Example: `$1,499 (2025-12)`
3. **Vendor Links**:
   - MUST have at least one vendor link per row
   - Format: `[Vendor Name](primary-url) ([archive](archive-url))`
   - Multiple vendors separated by commas: `[NVIDIA](...) ([archive](...)), [Dell](...) ([archive](...))`
4. **GPU VRAM**: MUST specify VRAM capacity (e.g., "12GB", "24GB")
5. **CPU Cores**: SHOULD include core count in parentheses for clarity
6. **Use Cases**: Maximum 3 use cases per tier, comma-separated

---

## Table 2: Jetson Development Kits (Minimum 2 Models)

### Schema

| Column Name        | Data Type | Required | Format/Constraints                                      |
|--------------------|-----------|----------|--------------------------------------------------------|
| Model              | String    | Yes      | Full product name (e.g., "Jetson Orin Nano (8GB)")    |
| GPU                | String    | Yes      | Architecture + specs (e.g., "Ampere, 1024 CUDA cores")|
| CPU                | String    | Yes      | Cores + architecture (e.g., "6-core Arm Cortex-A78AE")|
| RAM                | String    | Yes      | Unified memory (e.g., "8GB shared")                    |
| AI Performance     | String    | Yes      | TOPS rating (e.g., "40 TOPS (INT8)")                   |
| Price (2025-12)    | String    | Yes      | USD with date stamp (e.g., "$499 (2025-12)")           |
| Vendor Link        | Markdown  | Yes      | NVIDIA Developer Store link + archive                  |
| Use Cases          | String    | Yes      | Edge-specific applications (≤100 chars)                |

### Example Row

```markdown
| Jetson Orin Nano (8GB) | Ampere (1024 CUDA, 32 Tensor cores) | 6-core Arm Cortex-A78AE | 8GB shared | 40 TOPS (INT8) | $499 (2025-12) | [NVIDIA Dev Store](https://developer.nvidia.com/...) ([archive](https://archive.is/...)) | Edge inference, ROS 2 navigation, lightweight VLM deployment |
```

### Validation Rules

1. **Model Count**: Minimum 2 rows (Orin Nano 8GB + Orin NX 16GB recommended)
2. **Price Format**: Same as workstation table (`$XXX (2025-12)`)
3. **AI Performance**: MUST include TOPS rating and precision (INT8/FP16/FP32)
4. **Vendor Link**: MUST be NVIDIA Developer Store (official source)
5. **Use Cases**: Focus on edge deployment scenarios (inference, navigation, SLAM)

---

## Table 3: Robot Hardware (Optional, Minimum 2 Platforms)

### Schema

| Column Name        | Data Type | Required | Format/Constraints                                      |
|--------------------|-----------|----------|--------------------------------------------------------|
| Platform           | String    | Yes      | Product name (e.g., "TurtleBot 4 Standard")            |
| Type               | String    | Yes      | Form factor (e.g., "Mobile robot", "Quadruped")        |
| ROS 2 Support      | String    | Yes      | "Native Humble" or "Community drivers" + link          |
| Sensors            | String    | Yes      | Key sensors (e.g., "OAK-D camera, LIDAR, IMU")         |
| Price (2025-12)    | String    | Yes      | USD with date stamp (e.g., "$1,695 (2025-12)")         |
| Vendor Link        | Markdown  | Yes      | Manufacturer link + archive                            |
| Use Cases          | String    | Yes      | Sim-to-real applications (≤100 chars)                  |

### Example Row

```markdown
| TurtleBot 4 Standard | Mobile robot (differential drive) | Native ROS 2 Humble support | OAK-D stereo camera, RPLIDAR A1, IMU | $1,695 (2025-12) | [Clearpath Robotics](https://clearpathrobotics.com/...) ([archive](https://archive.is/...)) | Mobile navigation, SLAM, sim-to-real transfer for Module 04 |
```

### Validation Rules

1. **Platform Count**: Minimum 2 rows (e.g., TurtleBot 4 + Unitree Go2)
2. **ROS 2 Support**: MUST specify Humble compatibility (native or community drivers)
3. **Price Format**: Same as other tables (`$XXX (2025-12)`)
4. **Use Cases**: Emphasize sim-to-real and module-specific relevance
5. **Optional Label**: Table header SHOULD include "(Optional)" to clarify not required for Module 01

---

## General Formatting Rules

### Table Alignment (Recommended)

Use consistent column widths for readability in markdown source:

```markdown
| Tier      | GPU Model        | CPU              | RAM       | Storage      | Price (2025-12) |
|-----------|------------------|------------------|-----------|--------------|-----------------|
| Entry     | RTX 4060 Ti 16GB | i5-13600K        | 16GB DDR5 | 512GB NVMe   | $1,499          |
| Mid       | RTX 4070 Ti 12GB | i7-14700K        | 32GB DDR5 | 1TB NVMe     | $2,799          |
| High-End  | RTX 4090 24GB    | i9-14900K        | 64GB DDR5 | 2TB NVMe     | $5,499          |
```

### Header Row

- **First row**: Column names in plain text or **bold**
- **Second row**: Separator using `|---|---|---|`  (minimum 3 dashes per column)
- **Data rows**: Follow schema exactly

### Link Format in Cells

**Single vendor**:
```markdown
[NVIDIA](https://...) ([archive](https://archive.is/...))
```

**Multiple vendors**:
```markdown
[NVIDIA](https://...) ([archive](https://archive.is/...)), [Dell](https://...) ([archive](https://archive.is/...))
```

**No line breaks** within table cells (use commas for multiple items).

---

## Validation Checklist

Before committing hardware matrix tables:

- [ ] **Workstation table** has exactly 3 rows (Entry, Mid, High-End)
- [ ] **Jetson table** has minimum 2 rows (Orin Nano, Orin NX)
- [ ] **Robot table** has minimum 2 rows (TurtleBot 4, Unitree Go2 or equivalent)
- [ ] All prices include `(2025-12)` date stamp
- [ ] All vendor links have archive.is backups
- [ ] All GPU models specify VRAM capacity
- [ ] All Jetson models specify AI TOPS rating
- [ ] All robot platforms specify ROS 2 Humble compatibility
- [ ] Use cases are concise (≤100 characters per cell)
- [ ] Tables render correctly in Docusaurus preview

**Automated Checks** (CI/CD):
```bash
npm run check-links     # Validate all vendor links
npm run build           # Ensure tables render in Docusaurus
```

---

## Example: Complete Workstation Table

```markdown
| Tier      | GPU Model                | CPU                       | RAM       | Storage        | Price (2025-12) | Vendor Links | Use Cases |
|-----------|--------------------------|---------------------------|-----------|----------------|-----------------|--------------|-----------|
| Entry     | NVIDIA RTX 4060 Ti (16GB) | Intel Core i5-13600K (14 cores) | 16GB DDR5 | 512GB NVMe SSD | $1,499 (2025-12) | [NVIDIA Store](https://www.nvidia.com/en-us/geforce/graphics-cards/40-series/rtx-4060-4060ti/) ([archive](https://archive.is/EXAMPLE1)) | Basic Isaac Sim scenes, ROS 2 development, light VLM inference |
| Mid       | NVIDIA RTX 4070 Ti (12GB) | Intel Core i7-14700K (20 cores) | 32GB DDR5 | 1TB NVMe SSD   | $2,799 (2025-12) | [Dell](https://www.dell.com/...) ([archive](https://archive.is/EXAMPLE2)) | Complex Isaac Sim environments, multi-robot simulation, moderate VLM fine-tuning |
| High-End  | NVIDIA RTX 4090 (24GB)   | Intel Core i9-14900K (24 cores) | 64GB DDR5 | 2TB NVMe SSD   | $5,499 (2025-12) | [HP Z-series](https://www.hp.com/...) ([archive](https://archive.is/EXAMPLE3)) | Large-scale Isaac Sim scenes, heavy VLM training, multi-GPU setups |
```

---

## Data Sources

**Hardware Pricing** (as of 2025-12):
- NVIDIA GeForce GPUs: [NVIDIA Store](https://www.nvidia.com/en-us/geforce/graphics-cards/)
- Pre-built workstations: Dell Precision, HP Z-series, Lenovo ThinkStation
- Jetson kits: [NVIDIA Developer Store](https://developer.nvidia.com/embedded/buy/jetson-orin-nano-devkit)
- Robot platforms: Manufacturer websites (Clearpath Robotics, Unitree Robotics)

**Archive Links**:
- Create at https://archive.is/ for each vendor URL
- Verify archive snapshots load correctly before committing

---

**Contract Status**: ✅ Active
**Last Updated**: 2025-12-05
**Enforced By**: CI/CD link checker + manual review during PR
