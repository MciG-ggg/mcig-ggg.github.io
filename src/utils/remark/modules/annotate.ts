/**
 * Build-time renderer for the annotate module.
 *
 * The container holds one image and a list of `:pin` text directives. The
 * renderer puts the image and the pins into a stage and turns every pin into
 * a button. The button keeps its label in `data-label`, so the CSS shows the
 * label in a bubble without any client script.
 */
import type { Properties } from "hast";
import type { Image, RootContent } from "mdast";

import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/** A node as the module reads it. The mdast types do not cover directives. */
interface LooseNode {
	type: string;
	name?: string;
	attributes?: Record<string, string | null | undefined> | null;
	children?: RootContent[];
}

const PIN_NAME = "pin";

/**
 * Read the position of a pin.
 *
 * The position is a percentage between 0 and 100. A value that is not a
 * number has no position, so the pin is dropped.
 *
 * @param value - The raw attribute value.
 * @returns The percentage, or `null` when the value is not a number.
 */
const toPercent = (value: string | null | undefined): number | null => {
	if (value === null || value === undefined || value.trim() === "") return null;
	const parsed = Number(value);
	if (!Number.isFinite(parsed)) return null;
	return Math.min(100, Math.max(0, parsed));
};

/**
 * Collect the pins of the container.
 *
 * @param nodes - The nodes to search.
 * @param pins - The pins found so far.
 */
const collectPins = (nodes: RootContent[], pins: LooseNode[]): void => {
	for (const node of nodes) {
		const loose = node as unknown as LooseNode;
		if (loose.type === "textDirective") {
			if (loose.name === PIN_NAME) pins.push(loose);
			continue;
		}
		if (loose.children) collectPins(loose.children, pins);
	}
};

/**
 * Find the base image of the container.
 *
 * @param nodes - The nodes to search.
 * @returns The first image, or `undefined` when there is none.
 */
const findImage = (nodes: RootContent[]): Image | undefined => {
	for (const node of nodes) {
		const loose = node as unknown as LooseNode;
		if (loose.type === "image") return node as Image;
		if (loose.children) {
			const image = findImage(loose.children);
			if (image) return image;
		}
	}
	return undefined;
};

/**
 * Check whether a list of nodes holds nothing but whitespace.
 *
 * @param nodes - The nodes to check.
 * @returns `true` when the nodes render nothing.
 */
const isBlank = (nodes: RootContent[]): boolean => nodes.every(node => node.type === "text" && node.value.trim() === "");

/**
 * Remove the pins and the base image from the content of the container.
 *
 * The pins move into the stage. The rest of the content stays in the figure.
 * A heading in the content becomes a paragraph, so it does not land in the
 * table of contents.
 *
 * @param nodes - The nodes to clean.
 * @param base - The image that moves into the stage.
 * @returns The cleaned nodes.
 */
const clean = (nodes: RootContent[], base: Image): RootContent[] => {
	const kept: RootContent[] = [];
	for (const node of nodes) {
		const loose = node as unknown as LooseNode;
		if (node === base) continue;
		if (loose.type === "textDirective") {
			// The pins move into the stage. Other directives keep their own
			// rendering.
			if (loose.name === PIN_NAME) continue;
			kept.push(node);
			continue;
		}
		if (loose.type === "heading") {
			kept.push({ ...node, data: { ...node.data, hName: "p" } } as RootContent);
			continue;
		}
		if (loose.children) {
			const children = clean(loose.children, base);
			if (loose.type === "paragraph" && isBlank(children)) continue;
			kept.push({ ...node, children } as RootContent);
			continue;
		}
		kept.push(node);
	}
	return kept;
};

export const renderAnnotate = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const attributes = node.attributes ?? {};

	// Without an image there is nothing to annotate.
	const base = findImage(node.children);
	if (!base) return false;

	const pins: LooseNode[] = [];
	collectPins(node.children, pins);

	/*
	 * The stage holds the base image and the pins. The pins sit on top of the
	 * image, so the image must not zoom under the pointer.
	 */
	const baseProperties: Properties = { "data-nozoom": "true" };
	base.data = { ...base.data, hProperties: { ...base.data?.hProperties, ...baseProperties } };

	const stage: RootContent[] = [base];
	for (const pin of pins) {
		const x = toPercent(pin.attributes?.x);
		const y = toPercent(pin.attributes?.y);
		if (x === null || y === null) continue;
		const label = pin.attributes?.label ?? "";
		stage.push(
			context.element(
				"button",
				{
					className: ["iw-pin"],
					type: "button",
					ariaLabel: label || "标注点",
					"data-label": label,
					style: `--x: ${x}%; --y: ${y};`
				},
				[]
			)
		);
	}

	const children: RootContent[] = [context.element("div", { className: ["iw-annotate-stage"] }, stage)];

	// The content that is not part of the stage stays in the figure.
	children.push(...clean(node.children, base));

	const title = attributes.title;
	if (title) {
		children.push(context.element("figcaption", { className: ["iw-annotate-caption"] }, [{ type: "text", value: title }]));
	}

	const properties: Properties = { ...(node.data?.hProperties ?? {}) };
	const className = properties.className;
	properties.className = Array.isArray(className) ? [...className, "iw-annotate"] : ["iw-annotate"];

	node.data = { ...node.data, hName: "figure", hProperties: properties };
	node.children = children;
	return true;
};

export default renderAnnotate;
