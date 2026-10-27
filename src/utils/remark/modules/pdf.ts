/**
 * Build-time renderer for the pdf module.
 *
 * The directive holds one PDF file. The reader sees the file inside the page
 * and can also open it in a new tab or save it. The links sit in the header
 * next to the title, because a browser does not show a PDF inside a page on
 * every device.
 */
import type { RootContent } from "mdast";

import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/** The class of the box that holds the file. */
const PDF_CLASS = "iw-pdf";

/** The class of the header that holds the title and the links. */
const PDF_TITLE_CLASS = "iw-pdf-title";

/** The class of the badge that marks the file type. */
const PDF_BADGE_CLASS = "iw-pdf-badge";

/** The class of the row of links in the header. */
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
	const linkTitle = title || download;

	const openLink = context.element("a", { href: src, target: "_blank", rel: "noopener", "aria-label": `新标签页打开：${linkTitle}` }, [
		{ type: "text", value: "新标签页打开" }
	]);
	const saveLink = context.element("a", { href: src, download, "aria-label": `下载：${download}` }, [{ type: "text", value: "下载" }]);
	const links = context.element("span", { className: [PDF_LINKS_CLASS] }, [openLink, saveLink]);

	const header: RootContent[] = [];
	if (title) {
		header.push(context.element("span", { className: [PDF_BADGE_CLASS] }, [{ type: "text", value: "PDF" }]));
		header.push({ type: "text", value: title });
	}
	header.push(links);

	const children: RootContent[] = [
		context.element("figcaption", { className: [PDF_TITLE_CLASS] }, header),
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
	];

	node.data = { ...node.data, hName: "figure", hProperties: { className: ["iw-widget", PDF_CLASS] } };
	node.children = children;

	return true;
};

export default renderPdf;
