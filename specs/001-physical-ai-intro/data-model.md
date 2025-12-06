# Phase 1: Data Model — Module 01

**Date**: 2025-12-05
**Feature**: 001-physical-ai-intro
**Purpose**: Define content entities extracted from spec.md and research.md

---

## Entity Definitions

### 1. Content Page

**Description**: Represents one of 8 markdown pages in the Module 01 Docusaurus documentation.

**Fields**:
- `title` (string, required): Page title, displayed as H1
- `slug` (string, required): URL-friendly identifier (e.g., `hardware-requirements`)
- `heading_hierarchy` (structure, required): H1 (one) → H2 (multiple) → H3 (multiple per H2)
- `markdown_content` (text, required): Full page content in CommonMark format
- `diagram_refs` (array of strings, optional): Paths to Mermaid diagrams embedded in page (e.g., `['four-pillars-2025.mmd']`)
- `external_links` (array of objects, required): All outbound links with archive backups
  - `link_text` (string): Display text for link
  - `primary_url` (string): Original target URL
  - `archive_url` (string): Archive.is backup URL
  - `http_status` (integer): Last verified status code (200 expected)
- `page_count_estimate` (float, derived): Estimated formatted pages (contributes to 20-30 total limit)
- `last_updated` (date): Last modification date

**Validation Rules**:
- Exactly one H1 per page
- All H2/H3 must follow hierarchy (no H3 without parent H2)
- All `external_links` must have `archive_url` populated
- `page_count_estimate` sum across all pages must be ≥20 and ≤30
- All embedded diagrams in `diagram_refs` must exist in `static/diagrams/`

**State Transitions**:
draft → reviewed → published

**Relationships**:
- References `Architecture Diagram` entities via `diagram_refs`
- References `Curated Reference` entities in `next-steps-references.md`

**Example Instance**:
```yaml
title: "Hardware Requirements & 2025 Procurement Matrix"
slug: "hardware-requirements"
heading_hierarchy:
  - H1: "Hardware Requirements & 2025 Procurement Matrix"
  - H2: "Workstation Configurations"
    - H3: "Entry Tier ($1,500-$2,000)"
    - H3: "Mid Tier ($2,500-$3,500)"
    - H3: "High-End Tier ($4,000-$6,000+)"
  - H2: "Jetson Development Kits"
  - H2: "Optional Robot Platforms"
diagram_refs: []
external_links:
  - {link_text: "NVIDIA GeForce RTX 4070 Ti", primary_url: "https://...", archive_url: "https://archive.is/...", http_status: 200}
page_count_estimate: 4.5
last_updated: 2025-12-05
```

---

### 2. Hardware Configuration

**Description**: Represents one tested hardware setup (workstation or Jetson kit) with procurement details.

**Fields**:
- `tier` (enum, required): entry | mid | high-end (for workstations) OR N/A (for Jetson)
- `category` (enum, required): workstation | jetson | robot
- `name` (string, required): Configuration display name (e.g., "Mid Tier Workstation")
- `gpu_model` (string, required): Exact GPU model (e.g., "NVIDIA GeForce RTX 4070 Ti 12GB")
- `cpu_specs` (string, required): CPU model and core count (e.g., "Intel Core i7-14700K (20 cores)")
- `ram_gb` (integer, required): System RAM capacity in GB
- `storage_type` (enum, required): NVMe | SATA SSD | HDD
- `storage_gb` (integer, required): Storage capacity in GB
- `price_usd` (decimal, required): Total system price in USD
- `price_date` (string, required): Price verification date (format: "YYYY-MM", e.g., "2025-12")
- `vendor_links` (array of objects, required): Purchase links with archives
  - `vendor_name` (string): "NVIDIA", "Dell", "HP", etc.
  - `product_url` (string): Direct product page link
  - `archive_url` (string): Archive.is backup
- `use_cases` (array of strings, required): Supported activities (e.g., ["Basic Isaac Sim scenes", "ROS 2 development"])
- `notes` (string, optional): Additional guidance (e.g., "Recommended for learners")

**Validation Rules**:
- `price_date` must be "2025-12" for all configurations
- All `vendor_links` must have both `product_url` and `archive_url`
- Workstation configs must have `tier` set; Jetson/robot configs have `tier` = N/A
- `price_usd` must be > 0
- At least one `vendor_link` per configuration

**Relationships**:
- Referenced by `Content Page` entity (`hardware-requirements.md`)
- Organized in hardware matrix table

**Example Instance (Workstation)**:
```yaml
tier: "mid"
category: "workstation"
name: "Mid Tier Workstation"
gpu_model: "NVIDIA GeForce RTX 4070 Ti 12GB"
cpu_specs: "Intel Core i7-14700K (20 cores, 3.4-5.6 GHz)"
ram_gb: 32
storage_type: "NVMe"
storage_gb: 1000
price_usd: 2799.00
price_date: "2025-12"
vendor_links:
  - {vendor_name: "NVIDIA Store", product_url: "https://...", archive_url: "https://archive.is/..."}
use_cases: ["Complex Isaac Sim environments", "Multi-robot simulation", "Moderate VLM fine-tuning"]
notes: "Best balance of price/performance for serious developers"
```

**Example Instance (Jetson)**:
```yaml
tier: "N/A"
category: "jetson"
name: "Jetson Orin Nano Developer Kit (8GB)"
gpu_model: "NVIDIA Ampere (1024 CUDA cores, 32 Tensor cores)"
cpu_specs: "6-core Arm Cortex-A78AE"
ram_gb: 8
storage_type: "microSD"
storage_gb: 64
price_usd: 499.00
price_date: "2025-12"
vendor_links:
  - {vendor_name: "NVIDIA Developer Store", product_url: "https://developer.nvidia.com/...", archive_url: "https://archive.is/..."}
use_cases: ["Edge inference", "ROS 2 navigation", "Lightweight VLM deployment"]
notes: "Entry-level edge AI kit, ideal for Module 03 sim-to-real experiments"
```

---

### 3. Architecture Diagram

**Description**: Represents the "4 Pillars of Physical AI in 2025" Mermaid diagram and its exports.

**Fields**:
- `title` (string, required): "The 4 Pillars of Physical AI in 2025"
- `mermaid_source_path` (string, required): Path to `.mmd` file (e.g., `static/diagrams/four-pillars-2025.mmd`)
- `png_export_path` (string, required): Path to PNG fallback (e.g., `static/diagrams/four-pillars-2025.png`)
- `nodes` (array of objects, required): Diagram nodes
  - `id` (string): Node identifier (e.g., "ROS2")
  - `label` (string): Display text (e.g., "ROS 2 Humble<br/>Message Passing & Control")
  - `color` (string): Fill color hex code (e.g., "#4CAF50")
  - `description` (string): Explanation of this pillar's role
- `edges` (array of objects, required): Directional connections
  - `from_node` (string): Source node ID
  - `to_node` (string): Target node ID
  - `label` (string): Edge description (e.g., "Standardized APIs")
- `description` (text, required): Full explanation of diagram's meaning
- `mobile_tested` (boolean, required): Verified rendering at ≥360px width
- `created_date` (date): Diagram creation date
- `last_updated` (date): Last modification date

**Validation Rules**:
- `nodes` array must have exactly 4 entries (ROS2, SIM, ISAAC, VLA)
- `edges` must form a left-to-right flow (ROS2 → SIM → ISAAC → VLA)
- Both `mermaid_source_path` and `png_export_path` files must exist
- `mobile_tested` must be true before deployment

**Relationships**:
- Embedded in `Content Page` entity (`four-pillars-architecture.md`)
- Source file stored in `static/diagrams/`

**Example Instance**:
```yaml
title: "The 4 Pillars of Physical AI in 2025"
mermaid_source_path: "static/diagrams/four-pillars-2025.mmd"
png_export_path: "static/diagrams/four-pillars-2025.png"
nodes:
  - {id: "ROS2", label: "ROS 2 Humble<br/>Message Passing & Control", color: "#4CAF50", description: "Foundation layer providing standardized communication"}
  - {id: "SIM", label: "Simulation<br/>Gazebo, Unity, Isaac", color: "#2196F3", description: "Physics-accurate virtual environments"}
  - {id: "ISAAC", label: "Isaac Platform<br/>Isaac Sim + Isaac ROS", color: "#FF9800", description: "NVIDIA's integrated simulation and perception platform"}
  - {id: "VLA", label: "Vision-Language-Action<br/>Embodied Intelligence", color: "#9C27B0", description: "AI models that connect perception to physical actions"}
edges:
  - {from_node: "ROS2", to_node: "SIM", label: "Standardized APIs"}
  - {from_node: "SIM", to_node: "ISAAC", label: "Physics & Sensors"}
  - {from_node: "ISAAC", to_node: "VLA", label: "Perception & Navigation"}
description: "The 4 Pillars diagram illustrates the technology stack progression from ROS 2 (foundation) to VLA models (application layer)..."
mobile_tested: true
created_date: 2025-12-05
last_updated: 2025-12-05
```

---

### 4. Curated Reference

**Description**: Represents one of 15 authoritative sources with citation and archival backup.

**Fields**:
- `reference_id` (integer, required): Sequential ID (1-15)
- `ieee_citation` (string, required): Full IEEE-style citation
- `publication_date` (date, required): Original publication or last update date
- `content_type` (enum, required): paper | video | documentation | blog | repository
- `primary_url` (string, required): Original source URL
- `archive_url` (string, required): Archive.is backup URL
- `relevance_notes` (string, required): Brief explanation of relevance to Physical AI
- `module_relevance` (array of strings, optional): Which modules reference this source (e.g., ["Module 01", "Module 04"])
- `http_status_primary` (integer, derived): Last verified HTTP status for primary_url
- `http_status_archive` (integer, derived): Last verified HTTP status for archive_url

**Validation Rules**:
- Total `Curated Reference` entities must be between 12-15
- At least 40% must have `publication_date` from 2023-01-01 to 2025-12-31
- Both `primary_url` and `archive_url` must return HTTP 200 (or archive accessible if primary fails)
- `content_type` distribution: minimum 3 papers, 2 videos, 3 documentation

**Relationships**:
- Listed in `Content Page` entity (`next-steps-references.md`)
- May be cross-referenced in other pages

**Example Instance**:
```yaml
reference_id: 1
ieee_citation: "A. Brohan et al., 'RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control,' arXiv:2307.15818, 2023."
publication_date: 2023-07-28
content_type: "paper"
primary_url: "https://arxiv.org/abs/2307.15818"
archive_url: "https://archive.is/PLACEHOLDER_RT2"
relevance_notes: "Foundational VLA architecture from Google DeepMind, directly relevant to Module 04 capstone project"
module_relevance: ["Module 01", "Module 04"]
http_status_primary: 200
http_status_archive: 200
```

---

### 5. GitHub Actions Workflow

**Description**: Represents one of 3 CI/CD workflows automating build, deployment, and quality validation.

**Fields**:
- `workflow_name` (enum, required): build | deploy | lint
- `yaml_path` (string, required): Path to workflow file (e.g., `.github/workflows/build.yml`)
- `trigger_conditions` (array of strings, required): Events that start workflow (e.g., ["push", "pull_request"])
- `trigger_branches` (array of strings, optional): Branch filters (e.g., ["main", "001-physical-ai-intro"])
- `steps` (array of objects, required): Workflow steps
  - `step_name` (string): Human-readable step name
  - `action` (string): GitHub Action used (e.g., "actions/checkout@v4") OR shell command
  - `description` (string): What this step does
- `success_criteria` (string, required): Definition of successful workflow completion
- `timeout_minutes` (integer, required): Maximum execution time before timeout
- `failure_blocks_deployment` (boolean, required): Whether failure prevents merge/deploy

**Validation Rules**:
- All 3 workflows (build, deploy, lint) must be defined
- `timeout_minutes` must be ≤10 for all workflows
- `deploy` workflow must have `failure_blocks_deployment` = true for link checking step
- `yaml_path` files must exist and be valid YAML

**Relationships**:
- Documented in `Content Page` entity (`repository-structure.md`)
- Executes validation for all other entities (checks links, builds site, lints markdown)

**Example Instance**:
```yaml
workflow_name: "lint"
yaml_path: ".github/workflows/lint.yml"
trigger_conditions: ["push", "pull_request"]
trigger_branches: ["main", "001-*", "002-*", "003-*", "004-*"]
steps:
  - {step_name: "Checkout code", action: "actions/checkout@v4", description: "Clone repository"}
  - {step_name: "Setup Node.js", action: "actions/setup-node@v4", description: "Install Node.js 18 LTS"}
  - {step_name: "Install dependencies", action: "npm ci", description: "Install package.json dependencies"}
  - {step_name: "Run markdownlint", action: "npm run lint-markdown", description: "Validate markdown syntax and formatting"}
  - {step_name: "Check links", action: "npm run check-links", description: "Verify all external links return HTTP 200"}
success_criteria: "All linting and link checks pass with zero errors"
timeout_minutes: 5
failure_blocks_deployment: true
```

---

## Entity Relationships Diagram

```
Content Page (8 instances)
├─ References: Architecture Diagram (1 instance in four-pillars-architecture.md)
├─ References: Hardware Configuration (multiple in hardware-requirements.md)
├─ References: Curated Reference (15 instances in next-steps-references.md)
└─ Validated by: GitHub Actions Workflow (lint workflow)

Architecture Diagram (1 instance)
├─ Stored in: static/diagrams/ (mermaid_source_path, png_export_path)
└─ Embedded in: Content Page (four-pillars-architecture.md)

Hardware Configuration (multiple instances)
├─ Organized in: Content Page (hardware-requirements.md table)
└─ Categories: 3 workstation tiers + 2 Jetson kits + 2 robot platforms

Curated Reference (12-15 instances)
├─ Listed in: Content Page (next-steps-references.md)
└─ Validated by: GitHub Actions Workflow (deploy workflow link check)

GitHub Actions Workflow (3 instances: build, deploy, lint)
├─ Validates: Content Page (lint workflow)
├─ Validates: Curated Reference links (deploy workflow)
└─ Documented in: Content Page (repository-structure.md)
```

---

## Data Model Summary

**Total Entities**: 5 core types
1. Content Page (8 instances)
2. Hardware Configuration (~7-10 instances: 3 workstation + 2 Jetson + 2-3 robot)
3. Architecture Diagram (1 instance: 4 Pillars)
4. Curated Reference (12-15 instances)
5. GitHub Actions Workflow (3 instances: build, deploy, lint)

**Key Validation Constraints**:
- Content Pages: 20-30 total formatted pages, H1→H2→H3 hierarchy, 100% archive backups
- Hardware Configurations: All prices dated "2025-12", HTTP 200 vendor links
- Architecture Diagram: Mobile-tested (≥360px), 4 nodes, left-to-right flow
- Curated References: 40%+ from 2023-2025, mix of types (3 papers, 2 videos, 3 docs)
- GitHub Workflows: All <10 minutes, deploy blocks on link check failure

**State Transitions** (Content Pages only):
draft → reviewed → published

---

**Data Model Status**: ✅ Complete
**Next Phase**: Generate contracts/ specifications
