---
id: 0006
title: Fix Sidebar CodeBlock ChatKit Bugs
stage: green
date: 2024-12-24
surface: agent
model: claude-sonnet-4-5-20250929
feature: site-redesign
branch: 001-site-redesign
user: developer
command: bug fixes
labels: ["bug-fixes", "sidebar", "mobile", "chatkit", "ux"]
links:
  spec: specs/site-redesign/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - physical-robotics-ai-book/src/theme/DocSidebar/index.tsx
  - physical-robotics-ai-book/src/theme/DocSidebar/styles.module.css
  - physical-robotics-ai-book/src/theme/CodeBlock/index.tsx
  - physical-robotics-ai-book/src/components/ChatKitWidget.tsx
  - physical-robotics-ai-book/src/components/ChatKitWidget.module.css
tests:
  - Manual testing on mobile and desktop browsers
---

## Prompt

User reported multiple bugs after initial implementation:

1. Sidebar has irrelevant padding/margin from top
2. Module tile's bullet is overlapping with "module no." - needs space between bullet and module number
3. Code block has dark text on dark background making it invisible to read
4. Blue outline appearing on selected module tile
5. Sub-tiles inside modules need margin from left
6. Two gray shades appearing on hover - full module getting grayed when hovering sub-tile
7. Large gap between sub-tiles needs reduction
8. ChatKit chat widget not responsive on mobile screens - getting cut from left side
9. ChatKit scroll propagation - scrolling in chat box causes background page to scroll on mobile

User requested step-by-step fixes with verification between each bug fix.

## Response snapshot

**Bug #1: Sidebar Top Padding/Margin**
- Removed extra `<nav>` wrapper causing spacing
- Removed `.sidebarNav` CSS with `gap: 0.5rem`
- Added aggressive `!important` overrides for all top margins/padding
- Simplified structure to single `.customSidebar` wrapper
- Result: Sidebar starts immediately at top without gap

**Bug #2: Module Bullet Overlapping**
- Initially added margin to all list items (caused hierarchy issues)
- Corrected to add `padding-left: 1.5rem` only to `.menu__list-item-collapsible > .menu__link`
- Keeps visual hierarchy intact while preventing overlap
- Result: Bullets have proper spacing from "Module 01" text

**Bug #3: Code Block Visibility**
- User requested complete removal of code block redesign
- Reverted CodeBlock wrapper to pass-through only: `<CodeBlock {...props} />`
- Removed all custom styling (language badges, syntax highlighting overrides)
- Result: Original Docusaurus code blocks restored

**Blue Outline Removal**
- Removed `border-left: 4px solid #2563eb` from active states
- Removed from both `.menu__link--active` and `[aria-current="page"]`
- Set all focus outlines to `none !important`
- Result: No blue borders on active/selected items

**Bug #4: Sub-Tiles Left Margin**
- Added `margin-left: 1.5rem !important` to nested `.menu__list .menu__list`
- Creates proper visual hierarchy with indentation
- Result: Chapters properly indented under modules

**Bug #5: Hover Double Gray**
- Set `.theme-doc-sidebar-item-category:hover` to `transparent !important`
- Removed parent container hover effect
- Only `.menu__link:hover` applies now (rgba(0, 0, 0, 0.05))
- Result: Single gray shade only on hovered item

**Bug #6: Sub-Tile Gap Reduction**
- Reduced item margin: `0.25rem → 0.1rem`
- Reduced padding: `0.75rem → 0.5rem`
- Reduced collapsible margin: `0.5rem → 0.1rem`
- Nested list margin: `0.25rem → 0`
- Result: Tighter spacing while maintaining readability

**Bug #7: ChatKit Mobile Responsiveness**
- Created `ChatKitWidget.module.css` with responsive breakpoints
- Mobile (≤768px): Full width with margins, 500px height
- Small mobile (≤480px): 70vh height, not full screen
- Replaced all inline styles with CSS classes
- Result: Widget fully visible on all screen sizes

**Bug #8: ChatKit Scroll Propagation**
- Added `overscroll-behavior: contain;` to chat wrapper
- Added `-webkit-overflow-scrolling: touch;` for iOS
- Result: Scrolling isolated to chat widget, no background scroll

## Outcome

- ✅ Impact: Fixed 9 critical UX bugs affecting sidebar navigation, code blocks, and mobile ChatKit widget. All fixes verified through step-by-step testing with user confirmation.
- 🧪 Tests: Manual testing after each fix. User verified each change before proceeding to next bug.
- 📁 Files: Modified 5 files (DocSidebar wrapper/styles, CodeBlock wrapper, ChatKitWidget component/styles). Created new ChatKitWidget.module.css for responsive styling.
- 🔁 Next prompts: Run build verification, create git commit, continue with Phase 7 (accessibility audits, cross-browser testing, documentation).
- 🧠 Reflection: Iterative bug fixing with user validation proved effective. Initial solutions sometimes required refinement (e.g., bullet spacing affected hierarchy). Mobile responsiveness required balancing full visibility with good UX (70vh vs 100vh). Scroll propagation is a common mobile web issue solved with CSS `overscroll-behavior`.

## Evaluation notes (flywheel)

- Failure modes observed: Initial sidebar fixes were too aggressive (removed too much spacing). Bullet spacing fix initially broke visual hierarchy by adding margin to all items instead of just category links. First ChatKit mobile fix used full screen (bad UX).
- Graders run and results (PASS/FAIL): No automated tests. All fixes manually verified by user after each change.
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Build verification to ensure all changes compile correctly, then proceed with accessibility testing and git commit.
