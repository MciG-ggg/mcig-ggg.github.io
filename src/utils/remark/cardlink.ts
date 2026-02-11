import { visit } from "unist-util-visit";
import type { Plugin } from "unified";
import type { Code, Root } from "mdast";
import type { Html } from "mdast";

/**
 * Parsed card link attributes from YAML
 */
interface CardLinkAttributes {
	url: string;
	title: string;
	description?: string;
	host?: string;
	favicon?: string;
	image?: string;
}

/**
 * Regex to match cardlink code blocks
 */
const CARDLINK_BLOCK_REGEX = /^cardlink$/i;

/**
 * Escape HTML special characters
 */
const escapeHtml = (str: string): string => {
	return str.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;").replace(/'/g, "&#039;");
};

/**
 * Simple YAML-like parser for key-value pairs
 * Handles: key: value or key: "value with spaces"
 */
const parseCardlinkYAML = (value: string): CardLinkAttributes | null => {
	const lines = value.split("\n");
	const result: Record<string, string> = {};

	for (const line of lines) {
		const trimmed = line.trim();
		if (!trimmed) continue;

		const colonIndex = trimmed.indexOf(":");
		if (colonIndex <= 0) continue;

		const key = trimmed.slice(0, colonIndex).trim();
		let val = trimmed.slice(colonIndex + 1).trim();

		// Remove quotes if present (both single and double)
		if ((val.startsWith('"') && val.endsWith('"')) || (val.startsWith("'") && val.endsWith("'"))) {
			val = val.slice(1, -1);
		}

		result[key] = val;
	}

	if (typeof result.url !== "string" || typeof result.title !== "string") {
		return null;
	}

	return {
		url: result.url,
		title: result.title,
		description: result.description,
		host: result.host,
		favicon: result.favicon,
		image: result.image
	};
};

/**
 * Remark plugin that converts ```cardlink code blocks into styled card link elements
 *
 * Uses direct HTML injection which is more reliable in remark.
 *
 * YAML Syntax:
 * ```cardlink
 * url: https://example.com
 * title: "Example Site"
 * description: "An example website"
 * host: example.com
 * favicon: https://example.com/favicon.ico
 * image: https://example.com/preview.png
 * ```
 */
const remarkCardlink: Plugin<[], Root> = () => {
	return (tree: Root) => {
		visit(tree, "code", (node: Code, index, parent) => {
			if (!parent || typeof index === "undefined") return;

			if (node.lang && CARDLINK_BLOCK_REGEX.test(node.lang)) {
				const attrs = parseCardlinkYAML(node.value);

				if (attrs) {
					// Build HTML string directly
					const dataAttr = attrs.image ? ` data-card-image="${escapeHtml(attrs.image)}"` : "";
					const styleAttr = attrs.image ? ` style="--card-image: url('${escapeHtml(attrs.image)}');"` : "";

					let html = `<a href="${escapeHtml(attrs.url)}" class="card-link" target="_blank" rel="nofollow noopener noreferrer"${dataAttr}${styleAttr}>`;
					html += '<div class="card-link-content">';

					// Meta row: favicon + host
					if (attrs.host || attrs.favicon) {
						html += '<div class="card-link-meta">';
						if (attrs.favicon) {
							html += `<img class="card-link-favicon" src="${escapeHtml(attrs.favicon)}" alt="" loading="lazy">`;
						}
						if (attrs.host) {
							html += escapeHtml(attrs.host);
						}
						html += "</div>";
					}

					// Title
					html += `<div class="card-link-title">${escapeHtml(attrs.title)}</div>`;

					// Description
					if (attrs.description) {
						html += `<div class="card-link-description">${escapeHtml(attrs.description)}</div>`;
					}

					html += "</div></a>";

					// Replace with HTML node
					const htmlNode: Html = {
						type: "html",
						value: html
					};

					parent.children.splice(index, 1, htmlNode as any);
				}
			}
		});
	};
};

export default remarkCardlink;
