---
id: 0002
title: Create Book Site Redesign Implementation Plan
stage: plan
date: 2025-12-19
surface: agent
model: claude-sonnet-4-5-20250929
feature: site-redesign
branch: 001-site-redesign
user: User
command: /sp.plan
labels: ["implementation-plan", "docusaurus", "tailwind-css", "ui-redesign", "accessibility"]
links:
  spec: specs/site-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/site-redesign/plan.md
tests:
  - Manual browser testing strategy documented
  - Lighthouse CI performance/accessibility audits
  - Cross-browser compatibility testing
---

## Prompt

```
Book Site Redesign (Homepage + Sidebar)

Follow the approved spec in specs/site-redesign/spec.md exactly.

Create full execution plan without any diagrams (no need for Mermaid).

1. Detailed folder/file structure changes (src/theme/ overrides, Tailwind config, new components)
2. Step-by-step integration plan:
   - Update homepage (index.md) with hero, cards, CTA
   - Customize sidebar (collapsible, icons, active highlight)
   - Add Tailwind classes for modern look (gradients, hover effects)
   - Improve docs pages (typography, code blocks)
   - Add subtle robotics-themed visuals (backgrounds/icons)
3. Testing strategy with exact commands:
   - npm run start → verify hero, cards, sidebar on desktop/mobile
   - Check responsiveness and loading speed
4. Risk mitigation: Mobile layout breaks, performance impact

Output in specs/site-redesign/PLAN.md

Go.
```

## Response snapshot

Created comprehensive 989-line implementation plan covering all aspects of the book site redesign:

**Technical Context**:
- Stack: TypeScript 5.x + React 18 + Docusaurus v3 + Tailwind CSS 3.x
- Target: Static site (GitHub Pages/Vercel)
- Performance goals: Lighthouse ≥90, FCP <1.8s, TTI <3.0s
- Accessibility: WCAG 2.1 Level AA, Lighthouse accessibility ≥90

**Constitution Check**: ✅ ALL 7 PRINCIPLES PASS
- Educational Clarity, Engineering Accuracy, Practical Applicability
- Spec-Driven Development, Ethical Responsibility
- Reproducibility & Open Knowledge, Zero Broken State

**Project Structure**:
- Docusaurus site in `physical-robotics-ai-book/`
- New components: HomepageHero, ModuleCards, FeatureShowcase
- Swizzled components: DocSidebar, DocItem, CodeBlock
- Static assets: Circuit patterns, geometric accents, 4 module icons
- Config files: tailwind.config.js, postcss.config.js

**Technology Decisions (Phase 0)**:
1. Docusaurus customization: Swizzling + Custom Components (upgrade-safe)
2. Styling: Tailwind CSS 3.x via docusaurus-plugin-tailwindcss
3. Icons: React Icons (tree-shakeable, emoji fallbacks)
4. Visuals: SVG patterns + CSS geometric accents (performance-optimized)

**12-Step Implementation Plan (Phase 1)**:
1. Setup Tailwind CSS (npm install, config files)
2. Create static assets (circuit-pattern.svg, module icons, emoji fallbacks)
3-6. Homepage components (Hero, ModuleCards, FeatureShowcase)
7. Swizzle sidebar (spacing, hover, active highlight, keyboard nav)
8-9. Swizzle docs/code blocks (typography, tables, syntax highlighting)
10-11. Add robotics theme + responsive validation
12. Graceful degradation for older browsers

**Comprehensive Testing Strategy**:
- Local manual testing: 25-item checklist (homepage, sidebar, docs, responsive, a11y)
- Build validation: `npm run build` + `npm run serve`
- Performance: Lighthouse CI (target all scores ≥90)
- Cross-browser: Chrome, Firefox, Safari, Edge, mobile browsers
- Accessibility: axe DevTools, keyboard testing, screen readers, WCAG 2.1 AA checklist

**Risk Mitigation** (5 risks identified):
1. Mobile layout breaks (Medium likelihood, High impact) - Responsive-first approach, device testing
2. Performance impact (Low likelihood, Medium impact) - Bundle monitoring, lazy loading, SVG optimization
3. Accessibility regression (Low likelihood, High impact) - Automated testing, focus management, contrast checks
4. Broken links/assets (Low likelihood, Medium impact) - Docusaurus validation, asset path testing
5. Swizzled components break on upgrade (Medium likelihood, Medium impact) - Minimal swizzling, --wrap flag, version pinning

**Implementation Sequence**: Foundation → Homepage → Navigation → Content → Polish → Hardening → Validation
**Estimated Timeline**: 3-5 days implementation + 1-2 days testing

## Outcome

- ✅ Impact: Created production-ready implementation plan with 12 concrete steps, comprehensive testing strategy, and risk mitigation for all 5 identified risks
- 🧪 Tests: Documented manual testing checklist (25 items), Lighthouse CI strategy, cross-browser testing (6 browsers), accessibility audit (axe DevTools + WCAG 2.1 AA)
- 📁 Files: specs/site-redesign/plan.md (989 lines)
- 🔁 Next prompts: /sp.tasks to generate atomic implementation tasks with acceptance criteria
- 🧠 Reflection: Plan successfully maps all 18 functional requirements and 10 success criteria to concrete implementation steps. Constitution check passed all 7 principles. Technology decisions favor upgrade-safe patterns (swizzling with --wrap) and performance (SVG patterns, Tailwind JIT, lazy loading). Risk analysis comprehensive with detection and rollback plans.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan generated successfully with all required sections
- Graders run and results (PASS/FAIL): Constitution Check PASS (7/7 principles), All technical context resolved, No critical unknowns
- Prompt variant (if applicable): Standard /sp.plan workflow adapted for UI redesign (no data model/contracts needed)
- Next experiment (smallest change to try): Proceed to /sp.tasks to break down 12-step plan into atomic, testable tasks
