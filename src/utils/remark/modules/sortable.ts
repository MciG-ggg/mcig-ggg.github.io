/**
 * Build-time renderer for the sortable module.
 */
import type { Properties } from "hast";
import type { RootContent } from "mdast";
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/**
 * Find the table inside the directive.
 *
 * The table plugin of the site wraps every table in a div, so the table is a
 * direct child of the directive and the wrapper div is its sibling.
 */
const findTable = (children: RootContent[]): RootContent | undefined => children.find(child => child.type === "table");

/**
 * Turn the headings inside the widget into paragraphs.
 *
 * The table of contents collects the headings of the page. A heading inside a
 * widget is a part of the widget, so it must not show up in the contents.
 */
const demoteHeadings = (children: RootContent[]): void => {
	for (const child of children) {
		if (child.type === "heading") {
			child.data = { ...child.data, hName: "p" };
			continue;
		}

		if ("children" in child) demoteHeadings(child.children as RootContent[]);
	}
};

export const renderSortable = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	void context;

	const table = findTable(node.children);
	if (!table) return false;

	demoteHeadings(node.children);

	const properties: Properties = { ...(node.data?.hProperties ?? {}) };
	const className = properties.className;
	properties.className = Array.isArray(className) ? [...className, "iw-sortable"] : ["iw-sortable"];

	node.data = {
		...node.data,
		hName: "div",
		hProperties: properties
	};

	return true;
};

export default renderSortable;
