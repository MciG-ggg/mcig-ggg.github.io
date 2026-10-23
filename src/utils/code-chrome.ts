import type { Element, Root } from "hast";
import type { ShikiTransformer } from "shiki";

/** The class of the code block. The block carries the language label. */
const BLOCK_CLASS = "iw-code";

/** The class of a line that the meta of the fence marked. */
const LINE_CLASS = "iw-code-highlight";

/** The language of the mermaid diagrams. The diagram renders itself. */
const MERMAID = "mermaid";

/**
 * Read the numbers of the marked lines from the meta of a fence.
 *
 * The meta of a fence holds the numbers of the lines to mark. The fence
 * ```python {1,3}``` marks the first line and the third line. A range such as
 * `{1,3-5}` marks every line from the first one to the last one.
 *
 * @param raw - The raw meta of a fence.
 * @returns The numbers of the marked lines.
 */
const parseMarkedLines = (raw: string | undefined): Set<number> => {
	const lines = new Set<number>();
	if (!raw) return lines;

	const group = /\{([\d\s,-]+)\}/.exec(raw);
	if (!group) return lines;

	for (const part of group[1].split(",")) {
		const range = /^\s*(\d+)\s*-\s*(\d+)\s*$/.exec(part);
		if (range) {
			const first = Number(range[1]);
			const last = Number(range[2]);
			for (let line = first; line <= last; line++) lines.add(line);
			continue;
		}

		const line = Number(part.trim());
		if (Number.isInteger(line) && line > 0) lines.add(line);
	}

	return lines;
};

/**
 * Find the `<pre>` element of a code block.
 *
 * The copy button transformer wraps the `<pre>` element in a
 * `div.code-container`, so the element is one level down.
 *
 * @param node - The element that holds the code block.
 * @returns The `<pre>` element, or `null` when the block has none.
 */
const findPre = (node: Element): Element | null => {
	if (node.tagName === "pre") return node;

	for (const child of node.children) {
		if (child.type === "element" && child.tagName === "pre") return child;
	}

	return null;
};

/**
 * Add a class to an element.
 *
 * The copy button transformer writes the class of the wrapper as a list, the
 * language transformer of Astro writes the class of the `<pre>` element as a
 * string. Both forms are handled here.
 *
 * @param node - The element to change.
 * @param name - The name of the class to add.
 */
const addClass = (node: Element, name: string): void => {
	const className = node.properties.className;
	if (Array.isArray(className)) {
		if (!className.includes(name)) className.push(name);
		return;
	}

	const current = node.properties.class;
	node.properties.class = typeof current === "string" && current !== "" ? `${current} ${name}` : name;
};

/**
 * Shiki transformer that adds the language label and the line highlight to
 * the code blocks.
 *
 * The transformer runs after the copy button transformer, so the `<pre>`
 * element already sits inside a `div.code-container`. The label belongs to
 * that wrapper, because the `<pre>` element scrolls sideways with the code.
 */
const chrome = (): ShikiTransformer => ({
	name: "code-chrome",

	line(node, lineNumber) {
		// A mermaid block holds a diagram, not code. The diagram replaces the
		// code of the block on the client, so the lines keep their own look.
		if (this.options.lang === MERMAID) return;

		if (!parseMarkedLines(this.options.meta?.__raw).has(lineNumber)) return;

		addClass(node, LINE_CLASS);
	},

	root(node: Root) {
		const block = node.children[0];
		if (!block || block.type !== "element") return;

		const pre = findPre(block);
		if (!pre) return;

		const language = pre.properties.dataLanguage;
		if (typeof language !== "string" || language === MERMAID) return;

		// The wrapper of the `<pre>` element does not scroll, so the label
		// stays in place while the code scrolls sideways. Without a wrapper
		// the label would scroll away with the code, so the block gets none.
		if (block.tagName === "pre") return;

		addClass(block, BLOCK_CLASS);
		block.properties.dataLanguage = language;
	}
});

export default chrome;
