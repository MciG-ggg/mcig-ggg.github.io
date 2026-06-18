# Design

## Source of truth
- Status: Active
- Last refreshed: 2026-06-18
- Primary product surfaces: Home page, about page, blog navigation.
- Evidence reviewed: `src/pages/[...locale]/index.astro`, `src/pages/[...locale]/about.astro`, `src/components/Heatmap.svelte`, `src/content/information/introduction.md`, `src/styles/global.css`, `site.config.ts`.

## Brand
- Personality: Quiet, study-focused, undergraduate research homepage.
- Trust signals: Clear identity, university affiliation, explicit current learning focus, direct contact links.
- Avoid: Inflated CV claims, fake publications, decorative landing-page effects, marketing copy, dense blog previews on the homepage.

## Product goals
- Goals: Make the home page a concise personal academic-style entry point; keep blog content discoverable through navigation; preserve ThoughtLite readability.
- Non-goals: Replacing the blog index, presenting a full formal CV, adding new dependencies or analytics.
- Success signals: Visitors can identify who MciG-ggg is, what he is studying, and how to contact him within the first screen.

## Personas and jobs
- Primary personas: Classmates, potential collaborators, teachers, visitors from GitHub or blog links.
- User jobs: Understand the author's background, current interests, and contact options quickly.
- Key contexts of use: Desktop academic browsing, mobile link sharing, dark/light theme reading.

## Information architecture
- Primary navigation: Keep existing home, notes, jottings, about, feed controls.
- Core routes/screens: `/` for personal summary; `/about` for longer self-introduction, linkroll, and timeline; `/note` and `/jotting` for writing.
- Content hierarchy: Identity first, then research interests, current focus, education, skills/tools, writing activity, contact.

## Design principles
- Principle 1: Home page content should be about the person, not a blog feed.
- Principle 2: Use existing theme tokens and components before adding new visual systems.
- Tradeoffs: Keep the academic-homepage structure while preserving the site's literary/minimal visual language.

## Visual language
- Color: Mostly existing primary/secondary/background tokens with small restrained green/blue/warm accents for section markers.
- Typography: Existing serif body and mono metadata; avoid oversized hero typography inside compact panels.
- Spacing/layout rhythm: Two-column desktop hero, single-column mobile, full-width unframed sections.
- Shape/radius/elevation: Minimal borders and underlines; small repeated stat cards are acceptable for the activity section; no nested cards or heavy shadows.
- Motion: Existing transitions only; no new decorative animations.
- Imagery/iconography: GitHub avatar and existing Iconify icons; no generated imagery.

## Components
- Existing components to reuse: `Base`, `Icon`, `Heatmap`, theme/navigation/footer components.
- New/changed components: `Heatmap` gets a backward-compatible `showEntries` prop and a GitHub-style yearly activity presentation with month labels and a Less/More legend.
- Variants and states: Home page uses `showEntries={false}` and 52 weeks so activity tooltips show counts without post links.
- Token/component ownership: Keep local page styling inside the home page unless reused elsewhere.

## Accessibility
- Target standard: Practical WCAG AA-minded readability.
- Keyboard/focus behavior: Links remain normal anchors with visible browser focus.
- Contrast/readability: Use theme colors and avoid low-opacity text for primary content; dark-mode secondary text must remain visibly lighter than borders and inactive heatmap cells.
- Screen-reader semantics: Use one `h1`, ordered section headings, meaningful avatar alt text.
- Reduced motion and sensory considerations: No new motion beyond existing theme behavior.

## Responsive behavior
- Supported breakpoints/devices: Mobile and desktop through Tailwind responsive utilities.
- Layout adaptations: Hero stacks on mobile; section grids collapse to one column.
- Touch/hover differences: Heatmap remains hover-oriented but not required for core page comprehension.

## Interaction states
- Loading: Avatar may load externally; content remains readable without it.
- Empty: Heatmap can show empty days using existing fallback copy.
- Error: Avatar failure should not break layout.
- Success: Contact and navigation links resolve to target routes.
- Disabled: Not applicable.
- Offline/slow network, if applicable: External avatar may be missing; local text remains primary.

## Content voice
- Tone: Direct, modest, Chinese-first with useful English technical terms.
- Terminology: Use "正在学习", "关注", "方向" for unverified experience; reserve stronger claims for proven work.
- Microcopy rules: Avoid describing UI mechanics; labels should name destinations or content.

## Implementation constraints
- Framework/styling system: Astro 5, Svelte 5, Tailwind v4, Iconify, existing content collections.
- Design-token constraints: Prefer existing CSS variables and Tailwind theme colors.
- Performance constraints: No new client-side dependencies; homepage should stay mostly static.
- Compatibility constraints: `Heatmap.svelte` default behavior must remain unchanged for existing callers.
- Test/screenshot expectations: Run lint, Astro check, build, and visually inspect `/` and `/about`.

## Open questions
- [ ] Replace GitHub avatar with a local portrait if the author later wants a self-hosted image / owner: MciG-ggg / impact: reliability and tone.
