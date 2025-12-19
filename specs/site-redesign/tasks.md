---
description: "Task list for Book Site Redesign implementation"
---

# Tasks: Book Site Redesign

**Input**: Design documents from `/specs/site-redesign/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are NOT included per specification - manual testing strategy documented in plan.md

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus web application. All paths relative to `physical-robotics-ai-book/` directory.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependencies

- [x] T001 Install Tailwind CSS dependencies in physical-robotics-ai-book/package.json
- [x] T002 [P] Install React Icons library in physical-robotics-ai-book/package.json
- [x] T003 [P] Create tailwind.config.js with custom robotics theme colors and background patterns
- [x] T004 [P] Create postcss.config.js for Tailwind processing
- [x] T005 Update docusaurus.config.ts to add docusaurus-plugin-tailwindcss
- [x] T006 Modify src/css/custom.css to import Tailwind directives (@tailwind base, components, utilities)

**Validation**: Run `npm run start` and verify Tailwind utilities work on a test element

---

## Phase 2: Foundational (Static Assets & Global Styles)

**Purpose**: Core visual assets and styling that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 [P] Create static/img/circuit-pattern.svg (monochrome, low opacity, tile-able)
- [x] T008 [P] Create static/img/geometric-accent.svg (abstract angular shapes with gradients)
- [x] T009 [P] Create static/img/module-icons/module-1.svg (Physical AI icon, 64x64px)
- [x] T010 [P] Create static/img/module-icons/module-2.svg (ROS 2 icon, 64x64px)
- [x] T011 [P] Create static/img/module-icons/module-3.svg (Isaac Sim icon, 64x64px)
- [x] T012 [P] Create static/img/module-icons/module-4.svg (VLA models icon, 64x64px)
- [x] T013 Add enhanced typography CSS to src/css/custom.css (h1-h4 styles with Tailwind utilities)
- [x] T014 [P] Add section divider CSS to src/css/custom.css (gradient pseudo-elements on h2)
- [x] T015 [P] Add table styling CSS to src/css/custom.css (headers, borders, alternating rows)
- [x] T016 [P] Add code block enhancement CSS to src/css/custom.css (rounded corners, shadow, border)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - First-Time Visitor Discovery (Priority: P1) 🎯 MVP

**Goal**: New visitors can quickly understand the book's value proposition and start reading

**Independent Test**: Navigate to homepage and verify hero section, 4 module cards, feature showcase, and "Start Reading" CTA all display correctly and function

### Implementation for User Story 1

- [ ] T017 [P] [US1] Create src/components/HomepageHero/index.tsx with title, subtitle, CTA button
- [ ] T018 [P] [US1] Create src/components/HomepageHero/styles.module.css with gradient background and circuit pattern overlay
- [ ] T019 [P] [US1] Create src/components/ModuleCards/index.tsx with 4 card grid layout and icon fallback logic
- [ ] T020 [P] [US1] Create src/components/ModuleCards/styles.module.css with hover effects and responsive grid
- [ ] T021 [P] [US1] Create src/components/FeatureShowcase/index.tsx highlighting RAG chatbot and personalization
- [ ] T022 [P] [US1] Create src/components/FeatureShowcase/styles.module.css with alternating left-right layout
- [ ] T023 [US1] Update src/pages/index.tsx to import and render HomepageHero, ModuleCards, FeatureShowcase components
- [ ] T024 [US1] Update src/pages/index.module.css to remove old styles and add homepage-specific overrides
- [ ] T025 [US1] Verify "Start Reading" CTA navigates to /docs/module-01-physical-ai-intro/
- [ ] T026 [US1] Test icon fallback by temporarily removing SVG files and verifying emojis display (🤖📚🎯🚀)
- [ ] T027 [US1] Verify hero section displays correctly on mobile (375px), tablet (768px), desktop (1440px)
- [ ] T028 [US1] Verify module cards stack vertically on mobile and display 2x2 grid on tablet/desktop
- [ ] T029 [US1] Verify all hover effects activate in <100ms using browser DevTools Performance tab
- [ ] T030 [US1] Run Lighthouse audit and verify Performance ≥90, Accessibility ≥90

**Checkpoint**: Homepage redesign complete and fully functional - User Story 1 delivers value independently

---

## Phase 4: User Story 2 - Returning User Navigation (Priority: P2)

**Goal**: Returning users can navigate efficiently through documentation with modern, responsive sidebar

**Independent Test**: Open any documentation page and verify sidebar displays with modern styling, smooth hover effects, clear active state, and sticky positioning

### Implementation for User Story 2

- [ ] T031 [US2] Run `npm run swizzle @docusaurus/theme-classic DocSidebar -- --wrap` in physical-robotics-ai-book/
- [ ] T032 [US2] Modify src/theme/DocSidebar/index.tsx to add improved spacing (space-y-2 py-4)
- [ ] T033 [US2] Add hover effects to sidebar navigation items in src/theme/DocSidebar/index.tsx (hover:bg-gray-100 transition-colors duration-100)
- [ ] T034 [US2] Enhance active page highlight with left border in src/theme/DocSidebar/index.tsx (border-l-4 border-blue-600)
- [ ] T035 [US2] Add keyboard navigation focus rings to sidebar items in src/theme/DocSidebar/index.tsx (focus:ring-2 focus:ring-blue-600)
- [ ] T036 [US2] Add aria-current="page" attribute to active sidebar item in src/theme/DocSidebar/index.tsx
- [ ] T037 [US2] Create src/theme/DocSidebar/styles.module.css for sidebar-specific overrides if needed
- [ ] T038 [US2] Verify sidebar sticky positioning works when scrolling long documentation pages
- [ ] T039 [US2] Test keyboard navigation: Tab through items, verify focus visible, test Enter key activation
- [ ] T040 [US2] Verify hover effects on desktop browsers (Chrome, Firefox, Safari, Edge)
- [ ] T041 [US2] Verify mobile sidebar opens as slide-out menu with hamburger icon
- [ ] T042 [US2] Test sidebar spacing and visual hierarchy on tablet (768px) and desktop (1440px)

**Checkpoint**: Sidebar navigation redesign complete - User Stories 1 AND 2 both work independently

---

## Phase 5: User Story 3 - Content Reading Experience (Priority: P2)

**Goal**: Documentation content has enhanced visual presentation with improved typography, code blocks, and tables

**Independent Test**: View any documentation page with text, code blocks, and tables to verify enhanced styling is applied throughout

### Implementation for User Story 3

- [ ] T043 [US3] Run `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap` in physical-robotics-ai-book/
- [ ] T044 [US3] Verify src/theme/DocItem/Layout/index.tsx wraps content in semantic HTML5 structure
- [ ] T045 [US3] Run `npm run swizzle @docusaurus/theme-classic CodeBlock -- --wrap` in physical-robotics-ai-book/
- [ ] T046 [US3] Modify src/theme/CodeBlock/index.tsx to add custom wrapper class for enhanced shadow and border
- [ ] T047 [US3] Ensure copy button is keyboard accessible in src/theme/CodeBlock/index.tsx
- [ ] T048 [US3] Add language badge display to code blocks in src/theme/CodeBlock/index.tsx
- [ ] T049 [US3] Create src/theme/CodeBlock/styles.module.css for code block enhancements
- [ ] T050 [US3] Verify typography enhancements from custom.css apply to all documentation pages (line height 1.7, proper heading hierarchy)
- [ ] T051 [US3] Verify code blocks display with rounded corners, shadow, and border in light mode
- [ ] T052 [US3] Verify code blocks display correctly in dark mode with proper syntax highlighting
- [ ] T053 [US3] Verify tables have clear headers, alternating row colors, and borders
- [ ] T054 [US3] Verify section dividers appear above h2 headings as subtle gradient lines
- [ ] T055 [US3] Test code block copy functionality works correctly
- [ ] T056 [US3] Verify no horizontal scroll on code blocks/tables at 375px+ viewport width (except intentionally long code lines)

**Checkpoint**: Documentation page enhancements complete - All content displays with improved readability

---

## Phase 6: User Story 4 - Mobile User Experience (Priority: P3)

**Goal**: All redesigned pages are fully responsive and functional on mobile devices

**Independent Test**: Access site on mobile devices (iPhone, Android) at various viewports (320px, 375px, 768px) and verify all elements adapt appropriately

### Implementation for User Story 4

- [ ] T057 [US4] Verify HomepageHero component uses responsive Tailwind utilities (text-4xl md:text-5xl lg:text-6xl)
- [ ] T058 [US4] Verify ModuleCards component responsive grid (grid-cols-1 md:grid-cols-2)
- [ ] T059 [US4] Verify FeatureShowcase component stacks vertically on mobile (flex-col md:flex-row)
- [ ] T060 [US4] Test homepage on iPhone Safari (iOS 15+) at 375px viewport width
- [ ] T061 [US4] Test homepage on Android Chrome at 360px viewport width
- [ ] T062 [US4] Test sidebar slide-out menu on mobile (tap hamburger, verify overlay, close behavior)
- [ ] T063 [US4] Verify touch targets are ≥44x44px for all interactive elements on mobile
- [ ] T064 [US4] Test documentation pages on mobile: verify text wraps, no overflow, readable font size
- [ ] T065 [US4] Test code blocks on mobile: verify horizontal scroll works for long lines
- [ ] T066 [US4] Test tables on mobile: verify horizontal scroll or appropriate wrapping
- [ ] T067 [US4] Verify no horizontal page scroll on any page at 320px viewport width (smallest mobile)
- [ ] T068 [US4] Test landscape orientation on mobile devices (verify layout adapts)
- [ ] T069 [US4] Verify touch interactions work smoothly (tap, swipe, pinch-to-zoom if applicable)

**Checkpoint**: All user stories now fully responsive and mobile-friendly

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final enhancements affecting multiple user stories and production readiness

- [ ] T070 [P] Add graceful degradation CSS fallbacks for gradients in src/css/custom.css
- [ ] T071 [P] Add CSS feature queries for flexbox/grid fallbacks in src/css/custom.css
- [ ] T072 [P] Add fallback values for CSS custom properties in src/css/custom.css
- [ ] T073 [P] Optimize all SVG assets with SVGO (circuit-pattern.svg, module icons)
- [ ] T074 [P] Verify Tailwind JIT mode purges unused CSS in production build
- [ ] T075 [P] Add ARIA labels to all icon-only buttons and decorative elements
- [ ] T076 [P] Verify semantic HTML and proper heading hierarchy across all pages
- [ ] T077 Run `npm run build` and verify build completes without errors or warnings
- [ ] T078 Run `npm run serve` and verify static site serves correctly with all assets loading
- [ ] T079 Check browser console for errors on all redesigned pages (homepage, docs pages)
- [ ] T080 [P] Run axe DevTools accessibility scan on homepage - fix all critical/serious issues
- [ ] T081 [P] Run axe DevTools accessibility scan on documentation page - fix all critical/serious issues
- [ ] T082 Test keyboard navigation on all interactive elements (no keyboard traps, logical tab order)
- [ ] T083 Test with NVDA or VoiceOver screen reader (page titles, headings, links, images announced correctly)
- [ ] T084 Verify color contrast ratio ≥4.5:1 for all text using browser DevTools
- [ ] T085 Verify all images have appropriate alt text or are marked decorative with empty alt=""
- [ ] T086 [P] Run Lighthouse CI audit on homepage (verify Performance ≥90, Accessibility ≥90, SEO ≥90)
- [ ] T087 [P] Run Lighthouse CI audit on documentation page (verify all scores ≥90)
- [ ] T088 Measure First Contentful Paint (verify <1.8s)
- [ ] T089 Measure Time to Interactive (verify <3.0s)
- [ ] T090 Measure Cumulative Layout Shift (verify <0.1)
- [ ] T091 [P] Cross-browser test on Chrome latest (all features work correctly)
- [ ] T092 [P] Cross-browser test on Firefox latest (all features work correctly)
- [ ] T093 [P] Cross-browser test on Safari latest (macOS/iOS - all features work correctly)
- [ ] T094 [P] Cross-browser test on Edge latest (all features work correctly)
- [ ] T095 Test responsive design at breakpoints: 320px, 375px, 768px, 1024px, 1440px, 1920px
- [ ] T096 Verify text zoom up to 200% works without horizontal scroll or content loss
- [ ] T097 Create quickstart.md in specs/site-redesign/ with local development setup instructions
- [ ] T098 Document all swizzled components and modifications in specs/site-redesign/plan.md notes section
- [ ] T099 Final manual validation checklist (complete all 25 items from plan.md testing strategy)
- [ ] T100 Commit all changes with descriptive message following project conventions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Phase 1 completion - BLOCKS all user stories
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P2 → P3)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1) - Homepage**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2) - Sidebar**: Can start after Foundational (Phase 2) - Independent from US1
- **User Story 3 (P2) - Content**: Can start after Foundational (Phase 2) - Independent from US1/US2
- **User Story 4 (P3) - Mobile**: Depends on US1, US2, US3 implementation (validates their responsiveness)

### Within Each User Story

- Tasks marked [P] within a story can run in parallel (different files)
- Component creation (T017-T022) can happen in parallel
- Verification/testing tasks should run after implementation tasks
- Mobile testing (US4) should happen after components are implemented

### Parallel Opportunities

- **Phase 1 Setup**: T002-T004 can run in parallel (different config files)
- **Phase 2 Foundational**: T007-T012 (all SVG assets) can run in parallel, T013-T016 (all CSS sections) can run in parallel
- **Phase 3 User Story 1**: T017-T022 (all component creation) can run in parallel
- **Phase 7 Polish**: T070-T072 (CSS fallbacks), T073-T074 (asset optimization), T075-T076 (ARIA/semantic), T080-T081 (accessibility scans), T086-T087 (Lighthouse audits), T091-T094 (cross-browser tests) can all run in parallel
- Once Foundational completes, User Stories 1-3 can be worked on in parallel by different team members

---

## Parallel Example: User Story 1 (Homepage)

```bash
# Launch all homepage component creation tasks together:
Task T017: "Create src/components/HomepageHero/index.tsx with title, subtitle, CTA button"
Task T018: "Create src/components/HomepageHero/styles.module.css with gradient background"
Task T019: "Create src/components/ModuleCards/index.tsx with 4 card grid layout"
Task T020: "Create src/components/ModuleCards/styles.module.css with hover effects"
Task T021: "Create src/components/FeatureShowcase/index.tsx highlighting features"
Task T022: "Create src/components/FeatureShowcase/styles.module.css with layout"

# Then integrate after all components are created:
Task T023: "Update src/pages/index.tsx to import and render all components"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T016) - CRITICAL: blocks all stories
3. Complete Phase 3: User Story 1 (T017-T030)
4. **STOP and VALIDATE**: Test homepage independently with manual checklist
5. Deploy/demo if ready

**Deliverable**: Attractive homepage with hero, 4 module cards, feature showcase, and "Start Reading" CTA

### Incremental Delivery

1. Complete Setup + Foundational (T001-T016) → Foundation ready
2. Add User Story 1 (T017-T030) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (T031-T042) → Test independently → Deploy/Demo (navigation improved)
4. Add User Story 3 (T043-T056) → Test independently → Deploy/Demo (content enhanced)
5. Add User Story 4 (T057-T069) → Test independently → Deploy/Demo (mobile optimized)
6. Polish (T070-T100) → Final validation → Production release

Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T016)
2. Once Foundational is done:
   - Developer A: User Story 1 - Homepage (T017-T030)
   - Developer B: User Story 2 - Sidebar (T031-T042)
   - Developer C: User Story 3 - Content (T043-T056)
3. After US1-3 complete:
   - Developer D: User Story 4 - Mobile (T057-T069) validates all previous work
4. Team completes Polish together (T070-T100)

---

## Task Count Summary

- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 10 tasks (BLOCKS all user stories)
- **Phase 3 (User Story 1 - P1)**: 14 tasks 🎯 MVP
- **Phase 4 (User Story 2 - P2)**: 12 tasks
- **Phase 5 (User Story 3 - P2)**: 14 tasks
- **Phase 6 (User Story 4 - P3)**: 13 tasks
- **Phase 7 (Polish)**: 31 tasks
- **Total**: 100 tasks

**Parallel Opportunities Identified**: 47 tasks marked [P] can run in parallel when dependencies allow

**Independent Test Criteria**:
- **US1**: Navigate to homepage, verify hero/cards/CTA functional
- **US2**: Open docs page, verify sidebar styling/hover/active state
- **US3**: View docs with code/tables, verify enhanced styling
- **US4**: Access on mobile, verify responsive behavior

**Suggested MVP Scope**: Phases 1-3 only (Setup + Foundational + User Story 1) = 30 tasks for initial attractive homepage

---

## Notes

- All paths relative to `physical-robotics-ai-book/` directory in project root
- [P] tasks can run in parallel (different files, no dependencies)
- [Story] labels (US1, US2, US3, US4) map tasks to user stories from spec.md
- Each user story should be independently completable and testable
- Tests are manual as per specification - documented in plan.md testing strategy
- Commit after completing each user story phase for clean rollback points
- Stop at any checkpoint to validate story independently
- No ADRs required - UI redesign with clear Docusaurus + Tailwind approach
