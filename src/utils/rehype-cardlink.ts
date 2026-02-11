import { visit } from "unist-util-visit";
import type { Plugin } from "unified";
import type { Root, Element } from "hast";

/**
 * Rehype plugin to add additional processing to cardlink elements
 * This works in tandem with the remark-cardlink plugin
 */
const rehypeCardlink: Plugin<[], Root> = () => {
	return (tree: Root) => {
		visit(tree, "element", (node: Element) => {
			// Check if this is a cardlink element
			if (
				node.tagName === "a" &&
				node.properties &&
				typeof node.properties.className === "string" &&
				(node.properties.className as string).includes("card-link")
			) {
				// Ensure the link has the proper rel attributes for security
				if (!node.properties.rel) {
					node.properties.rel = ["nofollow", "noopener", "noreferrer"];
				}
				if (!node.properties.target) {
					node.properties.target = "_blank";
				}
			}
		});
	};
};

export default rehypeCardlink;
