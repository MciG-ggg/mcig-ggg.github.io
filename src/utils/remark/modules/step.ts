/**
 * Build-time renderer for one step of the steps module.
 */
import { visit } from "unist-util-visit";
import type { Heading, Root } from "mdast";
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/**
 * Turn a step directive into one panel of a steps widget.
 *
 * The panel keeps the markdown of the step, so tables, formulas, code and
 * images inside a step render as usual. The title of the step becomes a
 * paragraph at the top of the panel. A paragraph and not a heading, because
 * the headings of a widget must not reach the table of contents.
 */
export const renderStep = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const title = node.attributes?.title ?? "";
	const children = [...node.children];

	if (title) {
		children.unshift(context.element("p", { className: ["iw-step-title"] }, [{ type: "text", value: title }]));
	}

	// A heading inside a widget would show up in the table of contents.
	visit(node as unknown as Root, "heading", (heading: Heading) => {
		heading.data = { ...heading.data, hName: "p" };
	});

	node.data = { ...node.data, hName: "div", hProperties: { className: ["iw-step"] } };
	node.children = children;

	return true;
};

export default renderStep;
