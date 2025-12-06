# Contract: Content Page Structure

**Purpose**: Defines the required structure, formatting, and validation rules for all Module 01 Docusaurus pages.

**Scope**: Applies to all 8 pages in `docs/module-01-physical-ai-intro/`

---

## Required Page Structure

### 1. Front Matter (YAML)

Every page MUST begin with YAML front matter:

```yaml
---
title: "Exact Page Title (Matches H1)"
sidebar_label: "Short Label for Sidebar"
sidebar_position: 1
description: "Brief meta description for SEO (100-160 characters)"
---
```

**Fields**:
- `title`: MUST match the H1 heading in the page body
- `sidebar_label`: Optional shorter version for navigation (≤30 characters)
- `sidebar_position`: Integer determining sidebar order (1-8 for Module 01)
- `description`: SEO meta description, 100-160 characters

---

## 2. Heading Hierarchy

**Rule**: H1 → H2 → H3 (NO H4 or deeper)

### H1 (Level 1)
- **Quantity**: Exactly ONE per page
- **Format**: `# Page Title`
- **MUST match**: Front matter `title` field
- **Example**: `# Hardware Requirements & 2025 Procurement Matrix`

### H2 (Level 2)
- **Quantity**: 2-6 per page (recommended 3-4 for readability)
- **Format**: `## Section Title`
- **Purpose**: Major topic sections
- **Example**: `## Workstation Configurations`

### H3 (Level 3)
- **Quantity**: 0-12 per page (recommended ≤3 per H2 parent)
- **Format**: `### Subsection Title`
- **Rule**: MUST have a parent H2 (no orphan H3s)
- **Example**: `### Entry Tier ($1,500-$2,000)`

**Validation**:
```bash
# Check hierarchy compliance
grep -E "^#{4,}" page.md && echo "ERROR: H4+ detected" || echo "PASS"
```

---

## 3. External Links

ALL external links MUST include archive.is backup:

**Format**:
```markdown
[Link Text](primary-url) ([archive](archive.is-url))
```

**Examples**:

✅ **CORRECT**:
```markdown
[NVIDIA GeForce RTX 4070 Ti](https://www.nvidia.com/en-us/geforce/graphics-cards/40-series/rtx-4070-4070ti/) ([archive](https://archive.is/abc123))
```

❌ **INCORRECT** (no archive):
```markdown
[NVIDIA GeForce RTX 4070 Ti](https://www.nvidia.com/en-us/geforce/graphics-cards/40-series/rtx-4070-4070ti/)
```

**Archive.is Creation**:
1. Visit https://archive.is/
2. Enter primary URL
3. Click "Save the page"
4. Copy resulting archive URL (format: `https://archive.is/XXXXX`)

**HTTP Status Validation**:
- Primary URL SHOULD return HTTP 200
- If primary URL fails (404/500), archive URL MUST be accessible
- CI/CD link checker validates both URLs at deployment time

---

## 4. Internal Links

**Cross-references** to other Module 01 pages:

**Format**: Relative path from current page
```markdown
See [Ubuntu 22.04 Setup](./ubuntu-ros2-setup.md) for installation instructions.
```

**Anchor links** within same page:
```markdown
Jump to [Hardware Matrix](#workstation-configurations) below.
```

**Validation**:
- All internal links MUST resolve to existing pages or anchors
- Use lowercase-with-dashes for anchor IDs (auto-generated from H2/H3)

---

## 5. Code Blocks

**Rule**: All code blocks MUST be copy-paste functional (no placeholders)

**Format**:
````markdown
```bash
# This is a comment explaining the command
sudo apt update && sudo apt install -y ros-humble-desktop
```
````

**Language Tags** (required for syntax highlighting):
- `bash`: Shell commands
- `python`: Python code
- `yaml`: YAML configurations
- `mermaid`: Mermaid diagrams
- `text`: Plain text output

**Version Tags** (when applicable):
```bash
# Ubuntu 22.04 LTS, ROS 2 Humble
sudo apt install ros-humble-nav2-bringup=1.1.9-1*
```

**NO Placeholders**:
❌ **INCORRECT**:
```bash
sudo apt install <package-name>
```

✅ **CORRECT**:
```bash
# Replace 'ros-humble-desktop' with specific package if needed
sudo apt install ros-humble-desktop
```

---

## 6. Images & Diagrams

**Mermaid Diagrams** (inline):
````markdown
```mermaid
graph LR
    A[Start] --> B[End]
```
````

**Static Images**:
```markdown
![Alt text describing the image](/img/module-01/hardware-workstation-example.jpg)
```

**Requirements**:
- **Alt text**: REQUIRED for accessibility (describe image content)
- **Path**: Static images in `/static/images/module-01/` or `/static/diagrams/`
- **Formats**: PNG (preferred), JPG (photos), SVG (vector graphics)
- **Size**: Optimize images to ≤500KB for web performance

**PNG Export for Mermaid** (fallback):
```markdown
<details>
<summary>Diagram (click to expand if JavaScript disabled)</summary>

![4 Pillars of Physical AI](/diagrams/four-pillars-2025.png)
</details>
```

---

## 7. Tables

**Hardware Matrix Example**:
```markdown
| Tier      | GPU Model        | CPU            | RAM   | Price (2025-12) | Vendor Links |
|-----------|------------------|----------------|-------|-----------------|--------------|
| Entry     | RTX 4060 Ti 16GB | i5-13600K      | 16GB  | $1,499          | [NVIDIA](url) ([archive](archive-url)) |
| Mid       | RTX 4070 Ti 12GB | i7-14700K      | 32GB  | $2,799          | [Dell](url) ([archive](archive-url)) |
| High-End  | RTX 4090 24GB    | i9-14900K      | 64GB  | $5,499          | [HP](url) ([archive](archive-url)) |
```

**Table Rules**:
- Use `|` for column separators
- Header row + separator row (`|---|---|---|`) + data rows
- Align columns for readability (optional but recommended)
- Links in cells MUST include archive backups

---

## 8. Callouts & Admonitions

**Docusaurus Admonitions**:
```markdown
:::tip
This is a helpful tip for readers.
:::

:::warning
Be careful when running this command with sudo privileges.
:::

:::danger
This action will delete all data. Ensure you have backups before proceeding.
:::

:::info
Additional context or background information.
:::
```

**Use Cases**:
- `:::tip`: Best practices, shortcuts, recommendations
- `:::warning`: Potential pitfalls, non-destructive cautions
- `:::danger`: Destructive actions, data loss risks
- `:::info`: Side notes, historical context, fun facts

---

## 9. Page Length Guidelines

**Target**: 20-30 formatted pages total for Module 01 (8 pages average 2.5-3.75 pages each)

**Estimation**:
- 1 Docusaurus page ≈ 800-1200 words (3-5 screens of content)
- Include diagrams, code blocks, and tables in estimate
- Use `docusaurus build` + manual inspection for final page count

**Validation**:
```bash
# Rough word count (exclude front matter, code blocks)
wc -w docs/module-01-physical-ai-intro/*.md
# Target: 6,400-9,600 words across 8 pages (20-30 formatted pages)
```

---

## 10. SEO & Metadata

**Meta Description** (front matter):
- Length: 100-160 characters
- Include: Module number, key topic, target audience
- Example: `Module 01 covers Physical AI foundations, hardware requirements, and Isaac Sim + ROS 2 setup for robotics developers.`

**Title Tags** (from front matter `title`):
- Format: `Page Title | Physical AI Book`
- Automatically generated by Docusaurus

---

## Validation Checklist

Before committing any page:

- [ ] Front matter YAML is valid and complete
- [ ] Exactly one H1, matches front matter title
- [ ] H2/H3 hierarchy correct (no H4+, no orphan H3s)
- [ ] All external links have archive.is backups
- [ ] All internal links resolve (no 404s)
- [ ] All code blocks have language tags and are copy-paste functional
- [ ] All images have alt text and exist in `/static/`
- [ ] Tables are properly formatted with Markdown syntax
- [ ] Page length contributes appropriately to 20-30 total page target
- [ ] No spelling/grammar errors (use markdownlint and spell check)

**Automated Checks** (CI/CD):
```bash
npm run lint-markdown   # Markdown syntax and style
npm run check-links     # Validate all links (primary + archive)
npm run build          # Ensure Docusaurus builds without errors
```

---

## Page List (Module 01)

1. `index.md` — What is Physical AI? (landing page)
2. `four-pillars-architecture.md` — The 4 Pillars diagram + explanation
3. `hardware-requirements.md` — 2025 hardware matrix + procurement
4. `ubuntu-ros2-setup.md` — OS + ROS 2 Humble installation
5. `isaac-sim-installation.md` — Isaac Sim 2024.x setup
6. `verification-testing.md` — Carter example + validation
7. `repository-structure.md` — Repo layout + CI/CD workflows
8. `next-steps-references.md` — Curated sources + Module 02 preview

---

**Contract Status**: ✅ Active
**Last Updated**: 2025-12-05
**Enforced By**: CI/CD lint workflow + pre-commit hooks
