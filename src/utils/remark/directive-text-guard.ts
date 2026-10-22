import type { Plugin } from "unified";
import { visit } from "unist-util-visit";
import type { Root, Text } from "mdast";
import type { Parent } from "unist";

/** A text directive as mdast produces it, with the fields this plugin reads. */
type TextDirectiveNode = {
	type: "textDirective";
	name: string;
	label?: string | null;
	attributes?: Record<string, string | null | undefined> | null;
	children: Root["children"];
};

/**
 * Remark plugin that turns stray text directives back into plain text.
 *
 * `remark-directive` parses every `:` in inline text as a possible directive
 * marker. A link label like `[03:10](https://example.com/?t=190)` therefore
 * becomes the text `03` followed by a text directive named `10`, and the
 * unknown directive renders as an empty `<div>` — the visible label loses
 * everything after the first colon.
 *
 * A real text directive always carries a label, children or attributes
 * (`:ruby[音(reading)]`, `:pin{x="12" y="34"}`). A bare `:name` with none of
 * them is prose, so it is restored to `:name` text.
 * The colon is re-attached to the preceding text node when there is one, so
 * `03` + `:10` renders as `03:10`.
 */
const remarkDirectiveTextGuard: Plugin<[], Root> = () => {
	return tree => {
		visit(tree, "textDirective", (node: TextDirectiveNode, index, parent: Parent | undefined) => {
			if (node.label || node.children.length > 0) return;
			if (node.attributes && Object.keys(node.attributes).length > 0) return;
			if (typeof index !== "number" || !parent) return;

			const previous = parent.children[index - 1];
			const text: Text = { type: "text", value: `:${node.name}` };
			if (previous && previous.type === "text") {
				(previous as Text).value += text.value;
			} else {
				parent.children.splice(index, 0, text);
				index += 1;
			}
			parent.children.splice(index, 1);
		});
	};
};

export default remarkDirectiveTextGuard;
