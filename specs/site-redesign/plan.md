# Implementation Plan: Book Site Redesign

**Branch**: `001-site-redesign` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/site-redesign/spec.md`

## Summary

Redesign the Physical AI & Humanoid Robotics documentation site (homepage, sidebar, and documentation pages) with a modern, attractive visual experience inspired by https://ai-native.panaversity.org/. The redesign will enhance user discovery, navigation, and content readability while maintaining WCAG 2.1 Level AA accessibility, SEO optimization, and mobile responsiveness.

**Technical Approach**: Leverage Docusaurus v3's theming system with custom React components and Tailwind CSS for styling. Override default theme components using swizzling where necessary, create custom homepage components for hero and module cards, and apply global CSS customizations for typography and visual elements.

## Technical Context

**Language/Version**: TypeScript 5.x + React 18 (Docusaurus v3 requirements)
**Primary Dependencies**:
- Docusaurus v3.x (documentation framework)
- Tailwind CSS 3.x (utility-first styling)
- React Icons or Heroicons (icon library)
- @docusaurus/theme-classic (base theme for swizzling)

**Storage**: N/A (static site generation, no database)
**Testing**:
- Manual browser testing (Chrome, Firefox, Safari, mobile browsers)
- Lighthouse CI for accessibility and performance audits
- Docusaurus build validation (`npm run build`)

**Target Platform**: Static site deployed to GitHub Pages / Vercel
**Project Type**: Web (Docusaurus-based documentation site)
**Performance Goals**:
- Lighthouse Performance score ≥90
- First Contentful Paint (FCP) <1.8s
- Time to Interactive (TTI) <3.0s
- Cumulative Layout Shift (CLS) <0.1

**Constraints**:
- Homepage loads above-the-fold content within 3 seconds (SC-006)
- Sidebar hover effects activate within 100ms (SC-004)
- No horizontal scroll on viewports ≥375px (SC-003)
- Maintain Google Lighthouse accessibility score of 90+ with WCAG 2.1 Level AA compliance (SC-007)

**Scale/Scope**:
- 1 homepage redesign
- 4 module highlight cards
- Sidebar customization affecting all documentation pages
- Typography and styling enhancements across ~50+ documentation pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Educational Clarity
**Status**: PASS
**Rationale**: The redesign directly supports educational clarity by improving visual hierarchy (SC-009), making navigation more intuitive (User Story 2), and enhancing content readability through better typography (FR-007). The improved homepage provides clear entry points for new learners (User Story 1).

### ✅ II. Engineering Accuracy
**Status**: PASS
**Rationale**: The plan follows industry-standard practices for Docusaurus customization, uses officially documented theming APIs, and applies well-established accessibility standards (WCAG 2.1 Level AA). No custom technical claims are made.

### ✅ III. Practical Applicability
**Status**: PASS
**Rationale**: Every design element specified in the spec has a concrete implementation path using Docusaurus theming and Tailwind CSS. All visual enhancements are implementable without theoretical blockers. The testing strategy includes specific commands for local validation.

### ✅ IV. Spec-Driven Development
**Status**: PASS
**Rationale**: This plan is directly derived from the approved spec in `specs/site-redesign/spec.md`. All changes maintain traceability to specific functional requirements (FR-001 through FR-018) and success criteria (SC-001 through SC-010).

### ✅ V. Ethical Responsibility
**Status**: PASS
**Rationale**: The redesign prioritizes accessibility (WCAG 2.1 Level AA, FR-016), includes keyboard navigation support, screen reader compatibility, and graceful degradation for older browsers. These measures ensure inclusive access to educational content.

### ✅ VI. Reproducibility & Open Knowledge
**Status**: PASS
**Rationale**: All dependencies are version-pinned. The implementation uses public, open-source tools (Docusaurus, Tailwind CSS, React Icons). Setup and testing commands are documented. The site remains publicly accessible on GitHub Pages.

### ✅ VII. Zero Broken State
**Status**: PASS
**Rationale**: The testing strategy includes build validation (`npm run build`), broken link checking (via existing CI), and manual verification commands. Changes are isolated to theming/styling with minimal risk to content or functionality.

**Overall Gate Status**: ✅ ALL GATES PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/site-redesign/
├── spec.md              # Feature specification
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (if unknowns exist)
├── checklists/
│   └── requirements.md  # Specification quality checklist
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
physical-robotics-ai-book/
├── src/
│   ├── components/
│   │   ├── HomepageFeatures/        # Existing feature cards component
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── HomepageHero/            # NEW: Hero section component
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── ModuleCards/             # NEW: 4 module highlight cards
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   └── FeatureShowcase/         # NEW: RAG chatbot & features section
│   │       ├── index.tsx
│   │       └── styles.module.css
│   ├── css/
│   │   └── custom.css               # MODIFIED: Global styles, Tailwind imports
│   ├── pages/
│   │   ├── index.tsx                # MODIFIED: New homepage layout
│   │   └── index.module.css         # MODIFIED: Homepage-specific styles
│   └── theme/                       # NEW: Docusaurus theme overrides
│       ├── Navbar/                  # Swizzled navbar component (header)
│       │   ├── index.tsx
│       │   └── styles.module.css
│       ├── DocSidebar/              # Swizzled sidebar component
│       │   ├── index.tsx
│       │   └── styles.module.css
│       ├── DocItem/                 # Swizzled doc page wrapper (typography)
│       │   ├── index.tsx
│       │   └── styles.module.css
│       └── CodeBlock/               # Swizzled code block component
│           ├── index.tsx
│           └── styles.module.css
├── static/
│   ├── img/
│   │   ├── circuit-pattern.svg      # NEW: Robotics theme background
│   │   ├── geometric-accent.svg     # NEW: Geometric visual elements
│   │   └── module-icons/            # NEW: Icons for 4 module cards
│   │       ├── module-1.svg
│   │       ├── module-2.svg
│   │       ├── module-3.svg
│   │       └── module-4.svg
│   └── fonts/                       # OPTIONAL: Custom typography if needed
├── docusaurus.config.ts             # MODIFIED: Tailwind plugin, theme config
├── tailwind.config.js               # NEW: Tailwind configuration
├── postcss.config.js                # NEW: PostCSS for Tailwind
└── package.json                     # MODIFIED: Add Tailwind dependencies
```

**Structure Decision**: This is a Docusaurus web application. The structure follows Docusaurus conventions with custom components in `src/components/`, theme overrides in `src/theme/` (using swizzling pattern), and global styles in `src/css/`. Static assets (icons, backgrounds) are placed in `static/` for public access.

## Complexity Tracking

No constitution violations detected. All gates pass without requiring justification.

## Phase 0: Research & Technology Decisions

### Research Tasks

No critical unknowns requiring research. All technical context is well-defined:
- Docusaurus v3 theming: Official documentation available at https://docusaurus.io/docs/swizzling
- Tailwind CSS integration: Community plugin `docusaurus-plugin-tailwindcss` available
- WCAG 2.1 Level AA: Standards documented at https://www.w3.org/WAI/WCAG21/quickref/
- React Icons: Standard icon library with NPM package `react-icons`

### Technology Selection

**Decision 1: Docusaurus Theme Customization Approach**
- **Chosen**: Swizzling + Custom Components
- **Rationale**: Swizzling allows selective override of Docusaurus theme components while maintaining upgrade compatibility. Custom components provide full control for homepage redesign.
- **Alternatives Considered**:
  - Full custom theme (rejected: too much maintenance overhead)
  - CSS-only approach (rejected: insufficient control for sidebar customization)

**Decision 2: Styling Framework**
- **Chosen**: Tailwind CSS 3.x via `docusaurus-plugin-tailwindcss`
- **Rationale**: Tailwind provides utility-first approach matching modern design requirements, excellent responsiveness utilities, and no runtime overhead. Well-supported Docusaurus plugin exists.
- **Alternatives Considered**:
  - CSS Modules only (rejected: verbose for responsive/hover effects)
  - Styled Components (rejected: adds runtime overhead)

**Decision 3: Icon Library**
- **Chosen**: React Icons (includes Heroicons)
- **Rationale**: Tree-shakeable, large icon collection, zero dependencies, supports Unicode emoji fallbacks natively via conditional rendering.
- **Alternatives Considered**:
  - Font Awesome (rejected: heavier bundle size)
  - Custom SVGs only (rejected: limited variety)

**Decision 4: Robotics Theme Visual Elements**
- **Chosen**: SVG circuit patterns + CSS geometric accents
- **Rationale**: SVGs are scalable, lightweight, and can be embedded as background images. CSS-based geometric accents (gradients, borders) have zero HTTP overhead.
- **Alternatives Considered**:
  - Canvas-based animations (rejected: complexity and accessibility concerns)
  - Image-based patterns (rejected: larger file sizes, scaling issues)

**Output**: `research.md` not required - all decisions documented above.

## Phase 1: Design & Implementation Plan

### Detailed File Structure Changes

#### 1. Package Dependencies (package.json)

**Add Dependencies**:
```json
{
  "dependencies": {
    "react-icons": "^5.0.1"
  },
  "devDependencies": {
    "tailwindcss": "^3.4.0",
    "postcss": "^8.4.32",
    "autoprefixer": "^10.4.16",
    "docusaurus-plugin-tailwindcss": "^0.2.0"
  }
}
```

#### 2. Tailwind Configuration

**File**: `tailwind.config.js` (NEW)
```javascript
/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx,md,mdx}',
    './docs/**/*.{md,mdx}',
  ],
  darkMode: ['class', '[data-theme="dark"]'], // Docusaurus theme integration
  theme: {
    extend: {
      colors: {
        'robotics-blue': '#2563eb',
        'robotics-green': '#10b981',
        'circuit-gray': '#6b7280',
      },
      backgroundImage: {
        'circuit-pattern': "url('/img/circuit-pattern.svg')",
        'geometric-accent': "url('/img/geometric-accent.svg')",
      },
      transitionTimingFunction: {
        'hover-smooth': 'cubic-bezier(0.4, 0, 0.2, 1)',
      },
    },
  },
  plugins: [],
};
```

**File**: `postcss.config.js` (NEW)
```javascript
module.exports = {
  plugins: {
    tailwindcss: {},
    autoprefixer: {},
  },
};
```

#### 3. Docusaurus Configuration

**File**: `docusaurus.config.ts` (MODIFICATIONS)

Add Tailwind plugin:
```typescript
plugins: [
  // ... existing plugins
  'docusaurus-plugin-tailwindcss',
],
```

Update theme config for accessibility:
```typescript
themeConfig: {
  // ... existing config
  metadata: [{name: 'theme-color', content: '#2563eb'}],
  colorMode: {
    respectPrefersColorScheme: true, // Already set
  },
  // ... rest of config
}
```

#### 4. Global Styles

**File**: `src/css/custom.css` (MODIFICATIONS)

Add Tailwind imports at top:
```css
@tailwind base;
@tailwind components;
@tailwind utilities;

/* Existing custom CSS below */
:root {
  --ifm-color-primary: #2563eb;
  --ifm-font-family-base: 'Inter', system-ui, -apple-system, sans-serif;
  --ifm-line-height-base: 1.7;
  --ifm-heading-line-height: 1.3;
}

/* Enhanced typography for documentation pages */
.markdown {
  @apply text-base leading-relaxed;
}

.markdown h1 {
  @apply text-4xl font-bold mb-6 mt-8;
}

.markdown h2 {
  @apply text-3xl font-semibold mb-4 mt-6;
}

.markdown h3 {
  @apply text-2xl font-semibold mb-3 mt-5;
}

.markdown h4 {
  @apply text-xl font-medium mb-2 mt-4;
}

/* Section dividers */
.markdown > h2::before {
  content: '';
  display: block;
  width: 100%;
  height: 1px;
  background: linear-gradient(to right, transparent, #e5e7eb, transparent);
  margin-bottom: 1.5rem;
}

/* Code block enhancements */
.theme-code-block {
  @apply rounded-lg shadow-md border border-gray-200 dark:border-gray-700;
}

/* Table styling */
.markdown table {
  @apply w-full border-collapse my-6;
}

.markdown th {
  @apply bg-gray-100 dark:bg-gray-800 px-4 py-3 text-left font-semibold border-b-2 border-gray-300 dark:border-gray-600;
}

.markdown td {
  @apply px-4 py-3 border-b border-gray-200 dark:border-gray-700;
}

.markdown tr:nth-child(even) {
  @apply bg-gray-50 dark:bg-gray-900;
}
```

### Step-by-Step Integration Plan

#### Step 1: Setup Tailwind CSS

**Commands**:
```bash
cd physical-robotics-ai-book
npm install --save-dev tailwindcss postcss autoprefixer docusaurus-plugin-tailwindcss
npm install react-icons
npx tailwindcss init
```

**Actions**:
1. Create `tailwind.config.js` with custom theme colors and background patterns
2. Create `postcss.config.js` for Tailwind processing
3. Update `docusaurus.config.ts` to add Tailwind plugin
4. Modify `src/css/custom.css` to import Tailwind directives

**Validation**: Run `npm run start` and verify Tailwind utilities work on a test element

#### Step 2: Create Static Assets

**Files to Create**:
1. `static/img/circuit-pattern.svg` - Subtle circuit board pattern for backgrounds
2. `static/img/geometric-accent.svg` - Geometric lines/shapes for visual interest
3. `static/img/module-icons/module-1.svg` through `module-4.svg` - Icons for module cards

**Design Specifications**:
- Circuit pattern: Monochrome, low opacity (10-15%), tile-able SVG
- Geometric accents: Abstract angular shapes, use brand colors with gradients
- Module icons: 64x64px, simple line icons representing each module theme

**Fallback Strategy**: If icons fail to load, React components will render Unicode emojis:
- 🤖 for robotics modules
- 📚 for learning/documentation modules
- 🎯 for practical application modules
- 🚀 for advanced topics

#### Step 3: Update Homepage (index.tsx)

**File**: `src/pages/index.tsx` (REPLACE CONTENT)

**New Structure**:
```tsx
import React from 'react';
import Layout from '@theme/Layout';
import HomepageHero from '@site/src/components/HomepageHero';
import ModuleCards from '@site/src/components/ModuleCards';
import FeatureShowcase from '@site/src/components/FeatureShowcase';

export default function Home(): JSX.Element {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Master ROS 2, Isaac Sim, and Vision-Language-Action Models for Next-Gen Robotics">
      <main>
        <HomepageHero />
        <ModuleCards />
        <FeatureShowcase />
      </main>
    </Layout>
  );
}
```

**Rationale**: Replaces default Docusaurus homepage with custom component composition, enabling full control over layout and styling.

#### Step 4: Customize Header/Navbar (Swizzle Navbar)

**Command**:
```bash
npm run swizzle @docusaurus/theme-classic Navbar -- --wrap
```

**File**: `src/theme/Navbar/index.tsx` (GENERATED, then MODIFIED)

**Component Responsibilities**:
- Remove default Docusaurus logo, replace with stylish book name/title
- Add Sign In and Sign Up buttons on the right side
- Integrate theme toggle button (light/dark mode)
- Ensure sticky/fixed positioning at top of page
- Responsive mobile menu (hamburger icon)
- Follow design inspiration from https://ai-native.panaversity.org/

**Key Features**:
- Left side: Book name with custom typography (e.g., gradient text, bold font)
- Right side:
  - Sign In button (outlined style)
  - Sign Up button (filled/primary style)
  - Theme toggle button (icon-based: sun/moon)
- Sticky header: `position: sticky; top: 0; z-index: 100`
- Mobile: Hamburger menu for navigation, buttons stack or hide gracefully
- Smooth transitions on theme toggle and hover states

**Modifications**:
1. **Remove Logo**: Comment out or remove logo rendering in navbar
2. **Add Book Title**: Render book name with Tailwind classes:
   ```tsx
   <div className="text-2xl font-bold bg-gradient-to-r from-blue-600 to-purple-600 bg-clip-text text-transparent">
     Physical AI & Humanoid Robotics
   </div>
   ```
3. **Add Auth Buttons**:
   ```tsx
   <div className="flex items-center gap-3">
     <button className="px-4 py-2 border-2 border-blue-600 text-blue-600 rounded-lg hover:bg-blue-50 dark:hover:bg-blue-900/20 transition-colors">
       Sign In
     </button>
     <button className="px-4 py-2 bg-blue-600 text-white rounded-lg hover:bg-blue-700 transition-colors">
       Sign Up
     </button>
   </div>
   ```
4. **Keep Theme Toggle**: Docusaurus provides this by default, ensure it's visible and styled consistently
5. **Mobile Menu**: Ensure hamburger menu includes auth buttons on mobile
6. **Accessibility**: Add ARIA labels to all buttons, ensure keyboard navigation works

**File**: `src/theme/Navbar/styles.module.css` (NEW)

**Custom Styles**:
```css
.navbar {
  position: sticky;
  top: 0;
  z-index: 100;
  backdrop-filter: blur(10px);
  background-color: rgba(255, 255, 255, 0.9);
  border-bottom: 1px solid rgba(0, 0, 0, 0.1);
  transition: background-color 0.3s ease;
}

[data-theme='dark'] .navbar {
  background-color: rgba(0, 0, 0, 0.9);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.bookTitle {
  font-size: 1.5rem;
  font-weight: 700;
  background: linear-gradient(to right, #2563eb, #7c3aed);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.authButtons {
  display: flex;
  align-items: center;
  gap: 0.75rem;
}

@media (max-width: 768px) {
  .authButtons {
    flex-direction: column;
    width: 100%;
    padding: 1rem;
  }
}
```

**Validation**:
- Verify navbar displays with book name (no logo)
- Verify Sign In/Sign Up buttons visible on desktop
- Verify theme toggle works
- Test mobile menu includes all elements
- Check sticky positioning on scroll

#### Step 5: Create HomepageHero Component

**File**: `src/components/HomepageHero/index.tsx` (NEW)

**Component Responsibilities**:
- Display book title from `docusaurus.config.ts`
- Render compelling subtitle
- Prominent "Start Reading" CTA button navigating to `/docs/module-01-physical-ai-intro/`
- Responsive layout with background gradient and circuit pattern overlay
- ARIA labels for accessibility

**Key Features**:
- Tailwind gradient background: `bg-gradient-to-br from-blue-600 to-purple-700`
- Circuit pattern overlay with low opacity
- CTA button with hover animation (<100ms per SC-004)
- Minimum contrast ratio 4.5:1 for WCAG AA compliance

**File**: `src/components/HomepageHero/styles.module.css` (NEW)

**Custom Styles**:
```css
.heroBanner {
  position: relative;
  overflow: hidden;
  min-height: 500px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.heroBackground {
  position: absolute;
  inset: 0;
  background-image: var(--circuit-pattern);
  opacity: 0.1;
  pointer-events: none;
}

.ctaButton {
  transition: transform 80ms cubic-bezier(0.4, 0, 0.2, 1);
}

.ctaButton:hover {
  transform: translateY(-2px);
}
```

#### Step 5: Create ModuleCards Component

**File**: `src/components/ModuleCards/index.tsx` (NEW)

**Component Responsibilities**:
- Render exactly 4 module highlight cards (per clarification answer)
- Each card displays: icon, module title, brief description
- Responsive grid layout (2x2 on desktop, 1 column on mobile)
- Icon fallback to Unicode emoji if SVG fails to load
- Hover effects with subtle elevation change

**Card Data Structure**:
```typescript
interface ModuleCard {
  id: string;
  icon: string; // Path to SVG
  emojiF fallback: string; // Unicode emoji
  title: string;
  description: string;
  link: string; // Link to module documentation
}

const modules: ModuleCard[] = [
  {
    id: 'physical-ai-intro',
    icon: '/img/module-icons/module-1.svg',
    emojiFallback: '🤖',
    title: 'Physical AI Fundamentals',
    description: 'Introduction to physical AI, humanoid robotics, and foundational concepts',
    link: '/docs/module-01-physical-ai-intro/',
  },
  {
    id: 'ros2-mastery',
    icon: '/img/module-icons/module-2.svg',
    emojiFallback: '⚙️',
    title: 'ROS 2 Mastery',
    description: 'Master Robot Operating System 2 for professional robotics development',
    link: '/docs/module-02-ros2-mastery/',
  },
  {
    id: 'isaac-sim',
    icon: '/img/module-icons/module-3.svg',
    emojiFallback: '🎮',
    title: 'Isaac Sim & Simulation',
    description: 'Photorealistic robot simulation with NVIDIA Isaac Sim',
    link: '/docs/module-04-isaac-platform/',
  },
  {
    id: 'vla-models',
    icon: '/img/module-icons/module-4.svg',
    emojiFallback: '🧠',
    title: 'Vision-Language-Action',
    description: 'Implement VLA models for intelligent robot decision-making',
    link: '/docs/advanced/vla-models/', // Adjust based on actual structure
  },
];
```

**Tailwind Classes**:
- Grid: `grid grid-cols-1 md:grid-cols-2 gap-6 max-w-6xl mx-auto px-4`
- Card: `bg-white dark:bg-gray-800 rounded-lg shadow-lg hover:shadow-xl transition-all duration-200 p-6`
- Icon: `w-16 h-16 mb-4`

#### Step 6: Create FeatureShowcase Component

**File**: `src/components/FeatureShowcase/index.tsx` (NEW)

**Component Responsibilities**:
- Showcase RAG chatbot feature with description and visual
- Highlight personalization capabilities
- Responsive two-column layout (stacks on mobile)
- Includes call-to-action to explore features

**Features to Highlight**:
1. **RAG Chatbot**: "Ask questions about any content and get instant, context-aware answers"
2. **Personalized Learning**: "Track your progress and get tailored recommendations"

**Layout**: Alternating left-right layout with icon/screenshot on one side, text on the other

#### Step 7: Customize Sidebar (Swizzle DocSidebar)

**Command**:
```bash
npm run swizzle @docusaurus/theme-classic DocSidebar -- --wrap
```

**File**: `src/theme/DocSidebar/index.tsx` (GENERATED, then MODIFIED)

**Modifications**:
1. **Improved Spacing**: Add `className="space-y-2 py-4"` to sidebar container
2. **Hover Effects**: Add Tailwind hover classes to navigation items
   ```tsx
   className="hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors duration-100 rounded-md px-3 py-2"
   ```
3. **Active Page Highlight**: Enhance active state with left border
   ```tsx
   className="border-l-4 border-blue-600 bg-blue-50 dark:bg-blue-900/20"
   ```
4. **Sticky Positioning**: Already handled by Docusaurus, verify with `position: sticky`
5. **Keyboard Navigation**: Ensure all items are focusable with visible focus rings
   ```tsx
   className="focus:outline-none focus:ring-2 focus:ring-blue-600 focus:ring-offset-2"
   ```

**Accessibility Enhancements**:
- Add `aria-current="page"` to active sidebar item
- Ensure proper heading hierarchy (`<nav aria-label="Documentation sidebar">`)
- Keyboard navigation: Tab, Arrow keys, Enter

#### Step 8: Enhance Documentation Pages (Swizzle DocItem)

**Command**:
```bash
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

**File**: `src/theme/DocItem/Layout/index.tsx` (GENERATED, then MODIFIED)

**Modifications**:
1. Wrap content in semantic HTML5 structure
2. Add visual section dividers (already handled in `custom.css`)
3. Apply enhanced typography (already handled in `custom.css`)

**Note**: Most typography and table styling is handled via global CSS in `src/css/custom.css` to avoid over-swizzling.

#### Step 9: Enhance Code Blocks (Swizzle CodeBlock)

**Command**:
```bash
npm run swizzle @docusaurus/theme-classic CodeBlock -- --wrap
```

**File**: `src/theme/CodeBlock/index.tsx` (GENERATED, then MODIFIED)

**Modifications**:
1. Add custom wrapper class for enhanced shadow and border
2. Ensure copy button is keyboard accessible
3. Add language badge display
4. Verify syntax highlighting themes work in both light/dark modes

**Styling** (via `custom.css`):
```css
.theme-code-block {
  @apply rounded-lg shadow-md border border-gray-200 dark:border-gray-700 my-6;
}

.theme-code-block-title {
  @apply bg-gray-100 dark:bg-gray-800 px-4 py-2 text-sm font-mono border-b border-gray-300 dark:border-gray-600;
}
```

#### Step 10: Add Robotics-Themed Visual Elements

**Background Patterns**:
1. Apply circuit pattern to hero section (already in HomepageHero component)
2. Add subtle geometric accents to feature showcase sections
3. Use gradient overlays on cards and buttons

**Implementation**:
- Hero: `background-image: url('/img/circuit-pattern.svg')` with low opacity overlay
- Feature cards: Gradient borders using Tailwind `bg-gradient-to-r`
- Section dividers: CSS pseudo-elements with gradient backgrounds (in `custom.css`)

**Performance Consideration**: Use CSS `background-image` with SVGs instead of `<img>` tags to reduce HTTP requests

#### Step 11: Responsive Design Validation

**Breakpoints to Test**:
1. Mobile: 320px - 768px (FR-011)
2. Tablet: 768px - 1024px (FR-012)
3. Desktop: 1024px+ (FR-013)

**Responsive Behaviors**:
- Homepage hero: Full-width on all devices, adjust padding
- Module cards: 1 column on mobile, 2x2 grid on tablet/desktop
- Feature showcase: Stack vertically on mobile, side-by-side on tablet/desktop
- Sidebar: Slide-out menu on mobile (default Docusaurus behavior)
- Code blocks: Horizontal scroll allowed for long lines
- Tables: Horizontal scroll or wrap as needed

**Tailwind Responsive Utilities**:
```tsx
<div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-2 gap-4 md:gap-6">
```

#### Step 12: Graceful Degradation for Older Browsers

**CSS Fallbacks**:
1. **Gradients**: Provide solid color fallback
   ```css
   .hero {
     background-color: #2563eb; /* Fallback */
     background-image: linear-gradient(to right, #2563eb, #7c3aed); /* Modern */
   }
   ```

2. **Flexbox/Grid**: Use CSS feature queries
   ```css
   @supports not (display: grid) {
     .module-cards {
       display: flex;
       flex-wrap: wrap;
     }
   }
   ```

3. **Custom Properties**: Provide fallback values
   ```css
   .element {
     color: #2563eb; /* Fallback */
     color: var(--ifm-color-primary); /* Modern */
   }
   ```

**Testing**: Verify in IE 11 (if required) or use Autoprefixer for vendor prefixes

### Testing Strategy

#### Local Development Testing

**Command**: `npm run start`

**Manual Test Checklist**:
0. **Header/Navbar** (test on all pages):
   - [ ] Book name displays with stylish typography (gradient text) on left side
   - [ ] No default Docusaurus logo visible
   - [ ] Sign In button visible and styled correctly (outlined style)
   - [ ] Sign Up button visible and styled correctly (filled/primary style)
   - [ ] Theme toggle button visible and functional (switches light/dark mode)
   - [ ] Header remains sticky/fixed at top when scrolling
   - [ ] Hover effects work on Sign In/Sign Up buttons
   - [ ] Mobile: Hamburger menu includes auth buttons and works correctly
   - [ ] Backdrop blur effect visible on header background
   - [ ] Header design matches https://ai-native.panaversity.org/ inspiration

1. **Homepage**:
   - [ ] Hero section displays title, subtitle, CTA button
   - [ ] Circuit pattern background visible with low opacity
   - [ ] "Start Reading" button navigates to first module
   - [ ] 4 module cards display with icons, titles, descriptions
   - [ ] Module card icons fallback to emojis when SVGs blocked (test by removing static files temporarily)
   - [ ] Feature showcase displays RAG chatbot and personalization sections
   - [ ] All hover effects activate in <100ms (use browser DevTools Performance tab)

2. **Sidebar Navigation** (test on any docs page):
   - [ ] Sidebar displays with improved spacing
   - [ ] Hover effects work smoothly on navigation items
   - [ ] Active/current page is clearly highlighted with blue left border
   - [ ] Sidebar remains accessible when scrolling long pages (sticky positioning)
   - [ ] Keyboard navigation works (Tab, Arrow keys, Enter)
   - [ ] Focus states are visible (blue ring)

3. **Documentation Pages**:
   - [ ] Typography is enhanced (larger line height, appropriate font sizes)
   - [ ] Code blocks have rounded corners, shadow, border
   - [ ] Code syntax highlighting works in light and dark modes
   - [ ] Tables have clear headers, alternating row colors, borders
   - [ ] Section headings have subtle dividers above them
   - [ ] Content is readable without horizontal scroll on 375px+ viewports

4. **Responsive Design**:
   - [ ] Test on mobile (375px width): All content stacks vertically, sidebar becomes hamburger menu
   - [ ] Test on tablet (768px width): Module cards display 2x2 grid, sidebar visible
   - [ ] Test on desktop (1440px width): Full layout, all elements properly spaced
   - [ ] Test on ultra-wide (1920px+ width): Content max-width constrains layout

5. **Accessibility**:
   - [ ] Keyboard navigation works for all interactive elements
   - [ ] Focus indicators are visible (2px blue ring)
   - [ ] Screen reader announces page structure correctly (use NVDA/JAWS)
   - [ ] Color contrast meets 4.5:1 minimum (use browser DevTools Accessibility panel)
   - [ ] All images have `alt` text or `aria-label`
   - [ ] Icon fallbacks work (emojis have `role="img"` and `aria-label`)

#### Build Validation

**Command**: `npm run build`

**Validation**:
- [ ] Build completes without errors
- [ ] No warnings about missing dependencies
- [ ] No TypeScript type errors
- [ ] No broken links (Docusaurus checks this automatically)

**Post-Build Check**:
```bash
npm run serve
```
- [ ] Static site serves correctly
- [ ] All assets load (check Network tab for 404s)
- [ ] No console errors in browser DevTools

#### Performance Testing

**Command** (using Lighthouse CI):
```bash
npm install -g @lhci/cli
lhci autorun --collect.url=http://localhost:3000 --collect.numberOfRuns=3
```

**Target Scores**:
- [ ] Performance: ≥90
- [ ] Accessibility: ≥90 (must meet WCAG 2.1 Level AA)
- [ ] Best Practices: ≥90
- [ ] SEO: ≥90

**Key Metrics** (from Lighthouse):
- [ ] First Contentful Paint (FCP): <1.8s
- [ ] Time to Interactive (TTI): <3.0s
- [ ] Cumulative Layout Shift (CLS): <0.1
- [ ] Largest Contentful Paint (LCP): <2.5s

**Performance Optimization Checklist**:
- [ ] Images optimized (use WebP format where possible)
- [ ] SVG icons compressed
- [ ] No unused CSS (Tailwind JIT mode enabled)
- [ ] Lazy loading enabled for images below fold
- [ ] Code splitting working (check bundle sizes)

#### Cross-Browser Testing

**Browsers to Test**:
- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest - macOS/iOS)
- [ ] Edge (latest)
- [ ] Mobile Safari (iOS 15+)
- [ ] Mobile Chrome (Android)

**Test Focus**:
- [ ] CSS gradients render correctly
- [ ] Hover effects work (desktop browsers)
- [ ] Touch interactions work (mobile browsers)
- [ ] Fonts load and display correctly
- [ ] Icons display or fallback to emojis

#### Accessibility Audit

**Tools**:
1. **axe DevTools** (browser extension)
   - [ ] Run automated scan on homepage
   - [ ] Run automated scan on documentation page
   - [ ] Fix all critical and serious issues

2. **Manual Keyboard Testing**:
   - [ ] Tab through all interactive elements in logical order
   - [ ] No keyboard traps
   - [ ] Skip to content link works

3. **Screen Reader Testing** (NVDA on Windows or VoiceOver on macOS):
   - [ ] Page title announced correctly
   - [ ] Headings structure makes sense
   - [ ] Links have descriptive text
   - [ ] Images have appropriate alt text or are marked decorative
   - [ ] Form elements (if any) have labels

4. **WCAG 2.1 Level AA Checklist**:
   - [ ] Color contrast ratio ≥4.5:1 for normal text, ≥3:1 for large text
   - [ ] All functionality available via keyboard
   - [ ] Focus visible on all interactive elements
   - [ ] No content flashes more than 3 times per second
   - [ ] Responsive design allows text zoom up to 200% without horizontal scroll

### Risk Mitigation

#### Risk 1: Mobile Layout Breaks

**Likelihood**: Medium
**Impact**: High (affects mobile users - significant traffic)

**Mitigation Strategies**:
1. **Responsive Design First**: Use mobile-first CSS approach with Tailwind
2. **Testing on Real Devices**: Test on physical iPhone and Android devices
3. **Viewport Meta Tag**: Verify `<meta name="viewport">` is set correctly in Docusaurus config
4. **Max-Width Constraints**: Use `max-w-` utilities to prevent overflow on small screens
5. **Fallback Layouts**: Ensure all grid layouts have single-column fallback for smallest viewports

**Detection**:
- Monitor for horizontal scroll on viewports <375px
- Check for overlapping elements or text cutoff
- Verify touch targets are ≥44x44px

**Rollback Plan**:
- If critical mobile breakage occurs, revert homepage changes and keep only sidebar/typography improvements
- Deploy fix within 24 hours or rollback to previous version

#### Risk 2: Performance Impact (Slower Load Times)

**Likelihood**: Low
**Impact**: Medium (affects user experience and SEO)

**Mitigation Strategies**:
1. **Bundle Size Monitoring**: Check Webpack bundle analyzer before/after changes
2. **Lazy Loading**: Only load icons when visible (React lazy loading)
3. **SVG Optimization**: Compress all SVG assets with SVGO
4. **Tailwind Purging**: Ensure unused CSS is purged in production builds (default in Docusaurus + Tailwind plugin)
5. **Code Splitting**: Verify Docusaurus code splitting works correctly for new components

**Detection**:
- Run Lighthouse before and after changes, compare scores
- Monitor Time to Interactive (TTI) metric
- Check network waterfall for slow-loading assets

**Thresholds**:
- If Performance score drops below 85: Investigate bundle size, optimize assets
- If FCP exceeds 2.5s: Enable resource preloading, optimize critical CSS
- If LCP exceeds 3.0s: Optimize largest image/element, consider lazy loading

**Rollback Plan**:
- If performance degrades >10% after deployment, temporarily disable Tailwind plugin and use CSS modules
- Investigate specific bottleneck and optimize before re-enabling

#### Risk 3: Accessibility Regression

**Likelihood**: Low
**Impact**: High (legal/compliance risk, excludes users)

**Mitigation Strategies**:
1. **Automated Testing**: Integrate axe-core into CI pipeline
2. **Focus Management**: Test all keyboard interactions manually before deployment
3. **Color Contrast Checks**: Use browser DevTools contrast checker on all text
4. **ARIA Labels**: Add explicit labels to all icon-only buttons and decorative elements
5. **Semantic HTML**: Use proper heading hierarchy and landmark roles

**Detection**:
- Run axe DevTools scan before deployment
- Test with screen reader (NVDA/VoiceOver)
- Verify keyboard navigation on all pages
- Check Lighthouse Accessibility score (must be ≥90)

**Compliance Checklist**:
- [ ] All interactive elements keyboard accessible
- [ ] All images have alt text or aria-label
- [ ] Color is not the only visual means of conveying information
- [ ] Focus indicators have 3:1 contrast with background
- [ ] Page structure uses proper heading levels (no skipped levels)

**Rollback Plan**:
- If accessibility score drops below 90 or critical violations found: Pause deployment, fix issues
- If issues cannot be fixed within 48 hours: Rollback changes and create tasks for remediation

#### Risk 4: Broken Links or Missing Assets

**Likelihood**: Low
**Impact**: Medium (poor user experience, SEO penalty)

**Mitigation Strategies**:
1. **Docusaurus Link Validation**: Leverage built-in broken link detection during build
2. **Asset Path Testing**: Verify all static assets load correctly in production build
3. **404 Page**: Ensure custom 404 page guides users back to valid content
4. **Relative Links**: Use Docusaurus `@site/` alias for internal links to avoid path issues

**Detection**:
- Run `npm run build` and check for broken link warnings
- Test `npm run serve` and browse all pages
- Check browser console for 404 errors
- Use link checker tool (e.g., `broken-link-checker` NPM package)

**Rollback Plan**:
- If critical navigation broken: Hotfix within 4 hours
- If minor broken links detected: Create fix tasks and deploy within 1 week

#### Risk 5: Swizzled Components Break on Docusaurus Upgrade

**Likelihood**: Medium (inherent risk of swizzling)
**Impact**: Medium (blocks future upgrades)

**Mitigation Strategies**:
1. **Minimal Swizzling**: Only swizzle components where absolutely necessary (sidebar, code blocks)
2. **Wrapper Swizzling**: Use `--wrap` flag instead of `--eject` to minimize override surface
3. **Version Pinning**: Pin Docusaurus version in `package.json` until changes are stable
4. **Upgrade Testing**: Test Docusaurus upgrades in separate branch before merging
5. **Documentation**: Document all swizzled components and modifications made

**Detection**:
- After Docusaurus upgrade, run `npm run build` and check for TypeScript errors
- Test all swizzled components manually
- Check for deprecation warnings in console

**Rollback Plan**:
- If swizzled components break: Temporarily revert to previous Docusaurus version
- Re-implement customizations to match new theme structure
- If not fixable: Remove swizzling and achieve same effect via CSS-only approach

### Phase 1 Output

**Data Model**: Not applicable - this is a static site redesign with no data persistence

**Contracts**: Not applicable - no API endpoints, UI-only changes

**Quickstart**: Create `quickstart.md` with local development setup instructions

**File**: `specs/site-redesign/quickstart.md` (NEW)

## Implementation Sequence

Based on the step-by-step plan above, the implementation should follow this order:

1. **Foundation** (Steps 1-2): Setup Tailwind CSS, create static assets
2. **Header** (Step 4): Customize navbar - book name, auth buttons, theme toggle (affects all pages)
3. **Homepage** (Steps 3, 5-7): Update index.tsx, create Hero, ModuleCards, FeatureShowcase
4. **Navigation** (Step 8): Customize sidebar via swizzling
5. **Content** (Steps 9-10): Enhance documentation pages and code blocks
6. **Polish** (Steps 11-12): Add visual themes, validate responsiveness
7. **Hardening** (Step 13): Implement graceful degradation
8. **Validation** (Testing Strategy): Full test suite before deployment

**Estimated Timeline**: 4-6 days for implementation + 1-2 days for testing/fixes

## Success Criteria Verification

After implementation, verify each success criterion from the specification:

- **SC-001**: Time new visitors identifying book purpose and "Start Reading" button (target: ≤5 seconds)
- **SC-002**: Count clicks required to reach any top-level section from homepage (target: ≤3 clicks)
- **SC-003**: Test code blocks/tables on 375px viewport (target: no horizontal scroll)
- **SC-004**: Measure sidebar hover effect activation time with DevTools Performance (target: ≤100ms)
- **SC-005**: Verify all interactive elements have visible hover/focus states
- **SC-006**: Measure homepage load time with Lighthouse (target: ≤3 seconds for above-fold content)
- **SC-007**: Run Lighthouse accessibility audit (target: ≥90 score, WCAG 2.1 AA compliance)
- **SC-008**: Test mobile navigation without horizontal scroll (except oversized code/tables)
- **SC-009**: Visually verify content type distinction (headings, body, code, tables)
- **SC-010**: Time returning users locating sidebar (target: ≤2 seconds for 90% of users)

## Phase 2: Tasks

Phase 2 (task generation via `/sp.tasks` command) is NOT executed by `/sp.plan`. The tasks file will be created separately and will break down the implementation plan into atomic, testable tasks with acceptance criteria.

**Next Command**: `/sp.tasks` to generate `tasks.md`

## Notes

- All file paths use absolute Windows paths: `F:/Q4 Hackathon/Project 1/`
- Docusaurus project location: `F:/Q4 Hackathon/Project 1/physical-robotics-ai-book/`
- Current branch: `001-site-redesign`
- No ADRs required - this is a UI redesign with clear technical choices (Docusaurus + Tailwind)
- No backend changes needed
- Existing RAG chatbot integration remains unchanged (only visual showcase on homepage)
