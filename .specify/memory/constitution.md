<!--
Sync Impact Report:
- Version change: Not previously versioned → 1.0.0
- Modified principles: All principles created from scratch
- Added sections: Core Principles (7 principles), Content Standards, Development & Deployment Workflow, Governance
- Removed sections: None (first version)
- Templates requiring updates:
  ✅ plan-template.md (Constitution Check section aligned)
  ✅ spec-template.md (Requirements and success criteria aligned with educational clarity)
  ✅ tasks-template.md (Task structure compatible with spec-driven approach)
- Follow-up TODOs: None - all placeholders filled
-->

# AI/Spec-Driven Book: Physical AI, Humanoid Robotics, and Agentic Engineering — Constitution

## Core Principles

### I. Educational Clarity

Content MUST guide learners progressively from beginner to advanced concepts. Every technical explanation MUST be accessible to the intended audience while maintaining engineering rigor. Complex topics MUST be scaffolded with prerequisite knowledge explicitly identified.

**Rationale**: The book's value depends on readers successfully building understanding and skills. Clarity and progression prevent cognitive overload and abandonment.

### II. Engineering Accuracy

All robotics, AI, cloud-native, and agentic design content MUST reflect current industry standards and be technically correct. Claims MUST be verifiable against authoritative sources (official documentation, peer-reviewed research, or validated industry frameworks such as ROS2, URDF, modern LLM architectures, and agent orchestration patterns).

**Rationale**: Inaccurate technical information damages reader trust, wastes learning time, and can lead to unsafe or ineffective implementations in physical robotics contexts.

### III. Practical Applicability (NON-NEGOTIABLE)

Every concept introduced MUST translate into executable, hands-on implementation. Code examples MUST be runnable locally with specified dependencies. Theoretical concepts without implementation pathways are OUT OF SCOPE.

**Rationale**: The book's mission is to enable builders, not just readers. Concepts without practical application do not serve this mission.

### IV. Spec-Driven Development

Every chapter MUST be driven by an approved `.spec` file created using Spec-Kit Plus. Changes to chapter content MUST be reflected in the spec. The spec serves as the contract between planned content and delivered content.

**Rationale**: Spec-Driven Development ensures consistency, traceability, and quality gates. It enables collaborative review and prevents scope drift.

### V. Ethical Responsibility

Content involving physical robotics MUST include safety guidelines. AI and agentic system chapters MUST address responsible AI usage, including bias, transparency, and failure modes. Physical interaction with humanoid systems MUST emphasize testing protocols and risk mitigation.

**Rationale**: Robotics and AI systems can cause physical harm or societal impact. Ethical guidance is a professional and moral obligation.

### VI. Reproducibility & Open Knowledge

All documentation, code examples, and architectural designs MUST be reproducible by readers. Dependency versions MUST be specified. Environment setup MUST be documented. The repository and deployed documentation site MUST be publicly accessible and community-contributable.

**Rationale**: Open knowledge maximizes impact and enables community validation, improvement, and collaboration.

### VII. Zero Broken State

The repository MUST build and deploy successfully on every push to `main`. The deployed GitHub Pages site MUST have zero broken links and zero console errors. All code examples MUST pass automated tests before merge.

**Rationale**: A broken deployment or broken examples erode trust and block learning. Continuous validation ensures the book remains usable.

## Content Standards

### Originality & Attribution

- Content MUST be original or derived from authoritative technical sources, official documentation, or peer-reviewed research.
- All external references MUST be cited inline using standard academic format or direct links to official documentation.
- Images and diagrams MUST be original or licensed under CC-BY/CC0 with proper attribution.

### Code Quality

- Code examples MUST be executable and version-accurate.
- Maximum of 5 external dependencies per hands-on example (prefer pure Python/ROS2).
- All code examples MUST pass automated tests in CI (GitHub Actions).
- Examples MUST specify exact dependency versions and environment requirements.

### Technical Frameworks

- Robotics content MUST follow industry-validated frameworks: ROS2, URDF, standard control systems.
- AI content MUST reflect modern engineering practice: LLM architectures, embeddings, autonomy patterns, agent orchestration.
- Architectural diagrams MUST be described textually (Spec-Driven) and optionally rendered.

### Tone & Structure

- Documentation tone: mentor-to-student, respectful, high clarity.
- Progressive difficulty: fundamentals → intermediate → advanced projects.
- Each chapter MUST include end-of-chapter exercises or mini-projects with full solutions in a separate branch.
- Glossary and cross-references MUST be automatically generated and kept up-to-date.

## Development & Deployment Workflow

### Chapter Development

1. Create or update the chapter `.spec` file using `/sp.specify`.
2. Plan architecture and structure using `/sp.plan`.
3. Generate tasks using `/sp.tasks`.
4. Implement content following Spec-Driven Development principles.
5. Ensure code examples pass automated tests locally.
6. Create pull request; CI MUST pass (tests, build, link validation).
7. Peer review by at least one other contributor.
8. Merge to `main` only when all checks pass.

### Deployment

- Platform: Docusaurus documentation site deployed to GitHub Pages.
- Deployment MUST be automated via GitHub Actions on push to `main`.
- Pre-deployment checks: build success, zero broken links, zero console errors.

### Testing

- All code examples MUST have automated tests.
- Tests MUST run in CI on every pull request.
- Examples MUST include instructions for running tests locally.

### Community Contributions

- External contributors MUST follow the same Spec-Driven workflow.
- All contributions MUST pass the same quality gates.
- Contributor guidelines MUST be documented in `CONTRIBUTING.md`.

## Governance

### Amendment Process

1. Amendments to this constitution MUST be proposed via pull request to `.specify/memory/constitution.md`.
2. Amendments MUST include:
   - Rationale for the change.
   - Impact analysis on existing chapters, specs, and templates.
   - Migration plan if existing content is affected.
3. Amendments MUST be reviewed and approved by at least two project maintainers.
4. Version MUST be incremented according to semantic versioning:
   - **MAJOR**: Backward-incompatible governance or principle removals/redefinitions.
   - **MINOR**: New principle/section added or materially expanded guidance.
   - **PATCH**: Clarifications, wording, typo fixes, non-semantic refinements.

### Compliance

- All pull requests and reviews MUST verify compliance with this constitution.
- Complexity or deviations from principles MUST be explicitly justified in the PR description.
- Unjustified violations MUST block merge.

### Documentation

- This constitution supersedes all other practices and guidelines.
- In case of conflict between this constitution and other documentation, this constitution takes precedence.
- Runtime development guidance for agents and contributors MUST reference this constitution as the authoritative source.

### Release Criteria (v1.0)

The book MUST meet the following criteria before v1.0 release:

- 100% of chapters have merged, versioned `.spec` files.
- All code examples pass automated tests (GitHub Actions).
- Deployed site has zero dead links and zero console errors.
- At least one real humanoid-relevant or agentic project is fully reproducible by readers.
- Content passes review by at least two domain experts (one in robotics, one in AI/agent engineering).

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
