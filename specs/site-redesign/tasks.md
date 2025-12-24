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

## Phase 2.5: Header/Navbar (Global Navigation) 🌐 CRITICAL

**Purpose**: Global header redesign affecting all pages

**⚠️ CRITICAL**: This is a global component visible on ALL pages. Should be implemented early as it affects overall site navigation and branding.

- [x] T016a Run `npm run swizzle @docusaurus/theme-classic Navbar -- --wrap` in physical-robotics-ai-book/
- [x] T016b [P] Modify src/theme/Navbar/index.tsx to remove default Docusaurus logo
- [x] T016c [P] Add book name with gradient typography to left side of navbar in src/theme/Navbar/index.tsx
- [x] T016d [P] Add Sign In button (outlined style) to right side of navbar in src/theme/Navbar/index.tsx
- [x] T016e [P] Add Sign Up button (filled/primary style) to right side of navbar in src/theme/Navbar/index.tsx
- [x] T016f Ensure theme toggle button is visible and styled consistently in src/theme/Navbar/index.tsx
- [x] T016g Create src/theme/Navbar/styles.module.css with sticky positioning, backdrop blur, and responsive styles
- [x] T016h Add ARIA labels to Sign In, Sign Up, and theme toggle buttons for accessibility
- [x] T016i Verify navbar sticky positioning works when scrolling any page
- [x] T016j Verify book name displays with gradient effect (blue to purple)
- [x] T016k Verify Sign In/Sign Up buttons display correctly on desktop
- [x] T016l Test theme toggle switches between light and dark modes smoothly
- [x] T016m Test mobile hamburger menu includes auth buttons
- [x] T016n Verify header design matches https://ai-native.panaversity.org/ inspiration
- [x] T016o Test keyboard navigation through navbar elements (Tab, Enter)
- [x] T016p Verify backdrop blur effect works in both light and dark modes

**Checkpoint**: Header/navbar complete and visible on all pages - provides consistent navigation experience

---

## Phase 3: User Story 1 - First-Time Visitor Discovery (Priority: P1) 🎯 MVP

**Goal**: New visitors can quickly understand the book's value proposition and start reading

**Independent Test**: Navigate to homepage and verify hero section, 4 module cards, feature showcase, and "Start Reading" CTA all display correctly and function

### Implementation for User Story 1

- [x] T017 [P] [US1] Create src/components/HomepageHero/index.tsx with title, subtitle, CTA button
- [x] T018 [P] [US1] Create src/components/HomepageHero/styles.module.css with gradient background and circuit pattern overlay
- [x] T019 [P] [US1] Create src/components/ModuleCards/index.tsx with 4 card grid layout and icon fallback logic
- [x] T020 [P] [US1] Create src/components/ModuleCards/styles.module.css with hover effects and responsive grid
- [x] T021 [P] [US1] Create src/components/FeatureShowcase/index.tsx highlighting RAG chatbot and personalization
- [x] T022 [P] [US1] Create src/components/FeatureShowcase/styles.module.css with alternating left-right layout
- [x] T023 [US1] Update src/pages/index.tsx to import and render HomepageHero, ModuleCards, FeatureShowcase components
- [x] T024 [US1] Update src/pages/index.module.css to remove old styles and add homepage-specific overrides
- [x] T025 [US1] Verify "Start Reading" CTA navigates to /docs/module-01-physical-ai-intro/
- [x] T026 [US1] Test icon fallback by temporarily removing SVG files and verifying emojis display (🤖📚🎯🚀)
- [x] T027 [US1] Verify hero section displays correctly on mobile (375px), tablet (768px), desktop (1440px)
- [x] T028 [US1] Verify module cards stack vertically on mobile and display 2x2 grid on tablet/desktop
- [x] T029 [US1] Verify all hover effects activate in <100ms using browser DevTools Performance tab
- [x] T030 [US1] Run Lighthouse audit and verify Performance ≥90, Accessibility ≥90

**Checkpoint**: Homepage redesign complete and fully functional - User Story 1 delivers value independently

---

## Phase 4: User Story 2 - Returning User Navigation (Priority: P2)

**Goal**: Returning users can navigate efficiently through documentation with modern, responsive sidebar

**Independent Test**: Open any documentation page and verify sidebar displays with modern styling, smooth hover effects, clear active state, and sticky positioning

### Implementation for User Story 2

- [x] T031 [US2] Run `npm run swizzle @docusaurus/theme-classic DocSidebar -- --wrap` in physical-robotics-ai-book/
- [x] T032 [US2] Modify src/theme/DocSidebar/index.tsx to add improved spacing (space-y-2 py-4)
- [x] T033 [US2] Add hover effects to sidebar navigation items in src/theme/DocSidebar/index.tsx (hover:bg-gray-100 transition-colors duration-100)
- [x] T034 [US2] Enhance active page highlight with left border in src/theme/DocSidebar/index.tsx (border-l-4 border-blue-600)
- [x] T035 [US2] Add keyboard navigation focus rings to sidebar items in src/theme/DocSidebar/index.tsx (focus:ring-2 focus:ring-blue-600)
- [x] T036 [US2] Add aria-current="page" attribute to active sidebar item in src/theme/DocSidebar/index.tsx
- [x] T037 [US2] Create src/theme/DocSidebar/styles.module.css for sidebar-specific overrides if needed
- [x] T038 [US2] Verify sidebar sticky positioning works when scrolling long documentation pages
- [x] T039 [US2] Test keyboard navigation: Tab through items, verify focus visible, test Enter key activation
- [x] T040 [US2] Verify hover effects on desktop browsers (Chrome, Firefox, Safari, Edge)
- [x] T041 [US2] Verify mobile sidebar opens as slide-out menu with hamburger icon
- [x] T042 [US2] Test sidebar spacing and visual hierarchy on tablet (768px) and desktop (1440px)

**Checkpoint**: Sidebar navigation redesign complete - User Stories 1 AND 2 both work independently

---

## Phase 5: User Story 3 - Content Reading Experience (Priority: P2)

**Goal**: Documentation content has enhanced visual presentation with improved typography, code blocks, and tables

**Independent Test**: View any documentation page with text, code blocks, and tables to verify enhanced styling is applied throughout

### Implementation for User Story 3

- [x] T043 [US3] Run `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap` in physical-robotics-ai-book/
- [x] T044 [US3] Verify src/theme/DocItem/Layout/index.tsx wraps content in semantic HTML5 structure
- [x] T045 [US3] Run `npm run swizzle @docusaurus/theme-classic CodeBlock -- --wrap` in physical-robotics-ai-book/
- [x] T046 [US3] Modify src/theme/CodeBlock/index.tsx to add custom wrapper class for enhanced shadow and border
- [x] T047 [US3] Ensure copy button is keyboard accessible in src/theme/CodeBlock/index.tsx
- [x] T048 [US3] Add language badge display to code blocks in src/theme/CodeBlock/index.tsx
- [x] T049 [US3] Create src/theme/CodeBlock/styles.module.css for code block enhancements
- [x] T050 [US3] Verify typography enhancements from custom.css apply to all documentation pages (line height 1.7, proper heading hierarchy)
- [x] T051 [US3] Verify code blocks display with rounded corners, shadow, and border in light mode
- [x] T052 [US3] Verify code blocks display correctly in dark mode with proper syntax highlighting
- [x] T053 [US3] Verify tables have clear headers, alternating row colors, and borders
- [x] T054 [US3] Verify section dividers appear above h2 headings as subtle gradient lines
- [x] T055 [US3] Test code block copy functionality works correctly
- [x] T056 [US3] Verify no horizontal scroll on code blocks/tables at 375px+ viewport width (except intentionally long code lines)

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
- [x] T077 Run `npm run build` and verify build completes without errors or warnings ✅ (Build successful: Server 4.25m, Client 6.24m, exit code 0)
- [ ] T078 Run `npm run serve` and verify static site serves correctly with all assets loading (Requires manual verification)
- [ ] T079 Check browser console for errors on all redesigned pages (homepage, docs pages) (Requires manual verification)
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
- **Header/Navbar (Phase 2.5)**: Depends on Phase 1 completion - Global component affecting all pages
  - Can run in parallel with Foundational (Phase 2) after setup
  - Recommended to complete early as it provides consistent navigation across all pages
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - Header (Phase 2.5) should ideally be complete for consistent navigation experience
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
- **Phase 2.5 Header/Navbar**: T016b-T016f (navbar modifications) can run in parallel after swizzling (T016a)
- **Phase 3 User Story 1**: T017-T022 (all component creation) can run in parallel
- **Phase 7 Polish**: T070-T072 (CSS fallbacks), T073-T074 (asset optimization), T075-T076 (ARIA/semantic), T080-T081 (accessibility scans), T086-T087 (Lighthouse audits), T091-T094 (cross-browser tests) can all run in parallel
- **Phases 2 and 2.5 can run in parallel** after Phase 1 (Setup) completes
- Once Foundational and Header complete, User Stories 1-3 can be worked on in parallel by different team members

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
3. Complete Phase 2.5: Header/Navbar (T016a-T016p) - Global navigation component
4. Complete Phase 3: User Story 1 (T017-T030)
5. **STOP and VALIDATE**: Test homepage independently with manual checklist
6. Deploy/demo if ready

**Deliverable**: Attractive homepage with modern header, hero section, 4 module cards, feature showcase, and "Start Reading" CTA

### Incremental Delivery

1. Complete Setup + Foundational + Header (T001-T016p) → Foundation ready with global navigation
2. Add User Story 1 (T017-T030) → Test independently → Deploy/Demo (MVP with modern header!)
3. Add User Story 2 (T031-T042) → Test independently → Deploy/Demo (navigation improved)
4. Add User Story 3 (T043-T056) → Test independently → Deploy/Demo (content enhanced)
5. Add User Story 4 (T057-T069) → Test independently → Deploy/Demo (mobile optimized)
6. Polish (T070-T100) → Final validation → Production release

Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup together (T001-T006)
2. Split into parallel work streams:
   - Developer A: Foundational assets/styles (T007-T016)
   - Developer B: Header/Navbar (T016a-T016p)
3. Once Foundational and Header are done:
   - Developer A: User Story 1 - Homepage (T017-T030)
   - Developer B: User Story 2 - Sidebar (T031-T042)
   - Developer C: User Story 3 - Content (T043-T056)
4. After US1-3 complete:
   - Developer D: User Story 4 - Mobile (T057-T069) validates all previous work
5. Team completes Polish together (T070-T100)

---

## Task Count Summary

- **Phase 1 (Setup)**: 6 tasks
- **Phase 2 (Foundational)**: 10 tasks (BLOCKS all user stories)
- **Phase 2.5 (Header/Navbar)**: 16 tasks 🌐 CRITICAL (Global component)
- **Phase 3 (User Story 1 - P1)**: 14 tasks 🎯 MVP
- **Phase 4 (User Story 2 - P2)**: 12 tasks
- **Phase 5 (User Story 3 - P2)**: 14 tasks
- **Phase 6 (User Story 4 - P3)**: 13 tasks
- **Phase 7 (Polish)**: 31 tasks
- **Total**: 116 tasks

**Parallel Opportunities Identified**: 52 tasks marked [P] can run in parallel when dependencies allow

**Independent Test Criteria**:
- **US1**: Navigate to homepage, verify hero/cards/CTA functional
- **US2**: Open docs page, verify sidebar styling/hover/active state
- **US3**: View docs with code/tables, verify enhanced styling
- **US4**: Access on mobile, verify responsive behavior

**Suggested MVP Scope**: Phases 1-3 only (Setup + Foundational + Header + User Story 1) = 46 tasks for initial attractive homepage with modern header

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
