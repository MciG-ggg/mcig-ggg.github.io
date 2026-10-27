/**
 * Build-time renderer for the pdf module.
 *
 * The directive holds one PDF file. The reader sees the file inside the page
 * and can also open it in a new tab or save it. The links are always visible,
 * because a browser does not show a PDF inside a page on every device.
 */
import type { RootContent } from "mdast";

import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/** The class of the box that holds the file. */
const PDF_CLASS = "iw-pdf";

/** The class of the title above the file. */
const PDF_TITLE_CLASS = "iw-pdf-title";

/** The class of the row of links below the file. */
const PDF_LINKS_CLASS = "iw-pdf-links";

/** The height of the frame, in pixels, when the author gives no height. */
const DEFAULT_HEIGHT = "720";

export const renderPdf = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const attributes = node.attributes ?? {};
	const src = attributes.src?.trim();
	if (!src) return false;

	const title = attributes.title?.trim() ?? "";
	const height = attributes.height?.trim() || DEFAULT_HEIGHT;
	const parts = src.split("/");
	const download = attributes.download?.trim() || parts[parts.length - 1] || src;

	const children: RootContent[] = [];

	if (title) {
		children.push(context.element("figcaption", { className: [PDF_TITLE_CLASS] }, [{ type: "text", value: title }]));
	}

	children.push(
		context.element(
			"iframe",
			{
				className: ["iw-pdf-frame"],
				src,
				title: title || "PDF",
				height,
				loading: "lazy"
			},
			[]
		)
	);

	const openLink = context.element("a", { href: src, target: "_blank", rel: "noopener" }, [{ type: "text", value: "新标签页打开" }]);
	const saveLink = context.element("a", { href: src, download }, [{ type: "text", value: "下载 PDF" }]);

	children.push(context.element("div", { className: [PDF_LINKS_CLASS] }, [openLink, { type: "text", value: "　" }, saveLink]));

	node.data = { ...node.data, hName: "figure", hProperties: { className: ["iw-widget", PDF_CLASS] } };
	node.children = children;

	return true;
};

export default renderPdf;
