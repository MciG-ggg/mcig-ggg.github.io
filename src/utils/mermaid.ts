/**
 * Render the mermaid diagrams on the page.
 *
 * The blog highlights code with Shiki, which writes the language on the
 * `<pre>` as `data-language`. The `code.language-mermaid` selector that the
 * old code used no longer matches, so the diagrams were found by guessing
 * from the first word of the code. That guess also matched Python, because
 * `graph = torch.cuda.CUDAGraph()` starts with `graph`.
 */

const DIAGRAM_SELECTOR = 'pre[data-language="mermaid"]';

let instance: typeof import("mermaid").default | undefined;

/**
 * Read a CSS custom property as a plain color.
 *
 * The palette uses `light-dark()`, and getComputedStyle returns that
 * expression unparsed. Mermaid cannot parse it, so the value is resolved
 * through a throwaway element, which makes the browser compute the color.
 */
function resolveColor(value: string, fallback: string): string {
	const probe = document.createElement("span");
	probe.style.color = value;
	probe.style.display = "none";
	document.body.appendChild(probe);
	const resolved = getComputedStyle(probe).color;
	probe.remove();
	return resolved || fallback;
}

/** Read the site's surface colors so the diagram matches the theme. */
function themeColors(): { background: string; primary: string; text: string; border: string; line: string } {
	const style = getComputedStyle(document.documentElement);
	const read = (name: string, fallback: string) => resolveColor(style.getPropertyValue(name).trim(), fallback);

	return {
		background: read("--background-color", "#ffffff"),
		primary: read("--block-color", "#eeeeee"),
		text: read("--primary-color", "#2a2a28"),
		border: read("--shadow-color", "#cdcdcc"),
		line: read("--secondary-color", "#50504d")
	};
}

/** Tell a dark surface from a light one by the brightness of its channels. */
function isDark(color: string): boolean {
	const channels = color.match(/\d+/g);
	if (!channels || channels.length < 3) return false;

	const [red, green, blue] = channels.map(Number);
	return (red * 299 + green * 587 + blue * 114) / 1000 < 128;
}

/**
 * Set the label color to the site's text color.
 *
 * Mermaid derives the label color from the node fill, so the label keeps the
 * theme default instead of following the site palette. The style goes inside
 * the SVG, where it wins over the style block mermaid renders.
 */
function applySiteColors(block: HTMLElement, colors: { background: string; primary: string; text: string; border: string; line: string }) {
	const svg = block.querySelector("svg");
	if (!svg) return;

	const style = document.createElementNS("http://www.w3.org/2000/svg", "style");
	style.textContent = `
		.nodeLabel, .edgeLabel, .label, text, .node text, .edgeLabel p, .label p {
			color: ${colors.text} !important;
			fill: ${colors.text} !important;
		}
		.edgePath path, .flowchart-link, .messageLine0, .messageLine1 {
			stroke: ${colors.line} !important;
		}
		.edgeLabel, .label {
			background-color: ${colors.background} !important;
		}
	`;
	svg.appendChild(style);

	// Mermaid puts the node fill on the shape as an inline style, so a class
	// rule cannot reach it. Set the attribute directly.
	for (const shape of svg.querySelectorAll<SVGElement>(".node rect, .node circle, .node ellipse, .node polygon")) {
		shape.style.setProperty("fill", colors.primary, "important");
		shape.style.setProperty("stroke", colors.border, "important");
	}
}

async function getMermaid() {
	if (!instance) {
		// Load the library only when a page has a diagram. The bundle is large
		// and most pages have none.
		instance = (await import("mermaid")).default;
	}
	return instance;
}

async function renderDiagrams() {
	const blocks = Array.from(document.querySelectorAll<HTMLElement>(DIAGRAM_SELECTOR));
	if (blocks.length === 0) return;

	const mermaid = await getMermaid();
	const colors = themeColors();
	// The theme attribute is only set when the user picks a theme. Without it,
	// the site follows the system preference through `color-scheme: light dark`.
	// The resolved background color says which one the browser picked.
	const dark = isDark(colors.background);

	mermaid.initialize({
		startOnLoad: false,
		// The base theme keeps only the layout. Every color comes from the
		// theme variables, so the diagram matches the site in both modes.
		theme: "base",
		themeVariables: {
			darkMode: dark,
			background: colors.background,
			primaryColor: colors.primary,
			primaryTextColor: colors.text,
			primaryBorderColor: colors.border,
			// The node label has its own color variable. Without it the label
			// keeps the theme default, which is unreadable on the site's own
			// surfaces.
			nodeTextColor: colors.text,
			textColor: colors.text,
			lineColor: colors.line,
			edgeLabelBackground: colors.background,
			clusterBkg: colors.primary,
			clusterBorder: colors.border,
			noteBkgColor: colors.primary,
			noteTextColor: colors.text,
			noteBorderColor: colors.border,
			actorBkg: colors.primary,
			actorTextColor: colors.text,
			actorBorder: colors.border,
			actorLineColor: colors.line,
			signalColor: colors.line,
			signalTextColor: colors.text,
			labelBoxBkgColor: colors.primary,
			labelBoxBorderColor: colors.border,
			labelTextColor: colors.text,
			loopTextColor: colors.text,
			activationBkgColor: colors.primary,
			activationBorderColor: colors.border
		}
	});

	for (const block of blocks) {
		if (block.dataset.mermaidRendered === "true") continue;

		// Shiki splits the code into per-line spans. The text content still
		// holds the original source. After a re-render the code element is
		// gone, so fall back to the copy kept on the block.
		const source = block.querySelector("code")?.textContent ?? block.dataset.mermaidSource ?? block.textContent ?? "";
		if (!source.trim()) continue;

		const id = `mermaid-${Math.random().toString(36).slice(2, 11)}`;
		try {
			const { svg } = await mermaid.render(id, source);
			// Replace the content of the block, not the block itself. Replacing
			// the container also removes the copy button.
			block.innerHTML = svg;
			block.classList.add("mermaid-figure");
			applySiteColors(block, colors);
			block.dataset.mermaidRendered = "true";
			block.dataset.mermaidSource = source;
		} catch {
			// A diagram that fails to parse stays as its source code, which is
			// more useful than an empty box.
		}
	}
}

/** Render the diagrams and re-render them when the theme changes. */
export async function renderMermaid() {
	await renderDiagrams();

	const root = document.documentElement;
	if (root.dataset.mermaidBound === "true") return;
	root.dataset.mermaidBound = "true";

	document.addEventListener("theme:change", async () => {
		// The rendered SVG carries the old theme's colors, so render it again.
		for (const block of document.querySelectorAll<HTMLElement>(DIAGRAM_SELECTOR)) {
			const source = block.dataset.mermaidSource;
			if (!source) continue;
			block.dataset.mermaidRendered = "false";
		}
		await renderDiagrams();
	});
}
