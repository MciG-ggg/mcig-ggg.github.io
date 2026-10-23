/**
 * Build-time renderer for the compare module.
 */
import { visit } from "unist-util-visit";
import type { Image, Paragraph, Root, RootContent } from "mdast";
import type { Node, Parent } from "unist";
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/** The widget puts two images on top of each other. */
const IMAGE_COUNT = 2;

/** A node and the parent that holds it. */
interface Located {
	node: Node;
	parent: Parent;
	index: number;
}

/**
 * Remove the nodes from their parents.
 *
 * The nodes go out from the end of every parent, so the indexes of the other
 * nodes stay valid.
 */
const removeAll = (located: Located[]): void => {
	const byParent = new Map<Parent, Located[]>();

	for (const entry of located) {
		const list = byParent.get(entry.parent);
		if (list) list.push(entry);
		else byParent.set(entry.parent, [entry]);
	}

	for (const [parent, entries] of byParent) {
		entries.sort((a, b) => b.index - a.index);
		for (const entry of entries) parent.children.splice(entry.index, 1);
	}
};

/** Tell whether a paragraph holds nothing but white space. */
const isBlank = (node: Paragraph): boolean => node.children.every(child => child.type === "text" && child.value.trim() === "");

/**
 * Take the images out of the container.
 *
 * The images stay mdast image nodes. The markdown pipeline needs them there
 * to collect the local files and to optimize them. The paragraphs that hold
 * nothing but the images are dropped, so they leave no empty gap behind.
 */
const takeImages = (children: RootContent[]): { images: Image[]; rest: RootContent[] } => {
	const root: Root = { type: "root", children };
	const found: Located[] = [];

	visit(root, "image", (node, index, parent) => {
		if (found.length >= IMAGE_COUNT) return;
		if (typeof index !== "number" || !parent) return;
		found.push({ node, parent, index });
	});

	removeAll(found);

	const blank: Located[] = [];
	visit(root, "paragraph", (node, index, parent) => {
		if (!isBlank(node) || typeof index !== "number" || !parent) return;
		blank.push({ node, parent, index });
	});

	removeAll(blank);

	return { images: found.map(entry => entry.node as Image), rest: root.children };
};

/**
 * Keep the medium zoom of the site away from the images.
 *
 * The click on an image belongs to the range input of the widget. The site
 * attaches the zoom to every image that has no `data-nozoom`.
 *
 * The key is the attribute name on purpose. The image pipeline of the site
 * moves the properties of an image into a JSON blob and writes them back
 * later, and it keeps the key as it is. The value is a string, because the
 * pipeline escapes the value it writes.
 */
const markImage = (image: Image): void => {
	image.data = {
		...image.data,
		hProperties: {
			...image.data?.hProperties,
			"data-nozoom": "true"
		}
	};
};

/**
 * Turn the headings of the container into paragraphs.
 *
 * The table of contents only collects headings. A heading inside a widget
 * would show up there and break the outline of the page.
 */
const demoteHeadings = (children: RootContent[]): void => {
	visit({ type: "root", children }, "heading", node => {
		node.data = { ...node.data, hName: "p" };
	});
};

export const renderCompare = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const { images, rest } = takeImages(node.children);
	if (images.length < IMAGE_COUNT) return false;

	const [left, right] = images;
	markImage(left);
	markImage(right);
	demoteHeadings(rest);

	const attributes = node.attributes ?? {};
	const labels: RootContent[] = [];

	if (attributes.leftLabel) {
		labels.push(
			context.element("span", { className: ["iw-compare-label", "iw-compare-label-left"] }, [{ type: "text", value: attributes.leftLabel }])
		);
	}

	if (attributes.rightLabel) {
		labels.push(
			context.element("span", { className: ["iw-compare-label", "iw-compare-label-right"] }, [{ type: "text", value: attributes.rightLabel }])
		);
	}

	/*
	 * The range carries the name of the control. The two labels name the two
	 * sides, so the reader knows what the slider moves between.
	 */
	const names = [attributes.leftLabel, attributes.rightLabel].filter(Boolean);
	const ariaLabel = names.length === 2 ? `对比滑块：${names[0]} 与 ${names[1]}` : "对比滑块";

	// The right image is the base and gives the stage its height. The clip
	// layer with the left image comes after it, so it paints on top.
	const stage = context.element("div", { className: ["iw-compare-stage"] }, [
		right,
		context.element("div", { className: ["iw-compare-clip"] }, [left]),
		context.element("div", { className: ["iw-compare-divider"], ariaHidden: "true" }),
		context.element("input", {
			type: "range",
			className: ["iw-compare-range"],
			min: "0",
			max: "100",
			step: "1",
			value: "50",
			ariaLabel
		}),
		...labels
	]);

	node.data = {
		...node.data,
		hName: "div",
		hProperties: { ...node.data?.hProperties, className: ["iw-compare", "iw-widget"] }
	};
	node.children = [stage, ...rest];

	return true;
};

export default renderCompare;
