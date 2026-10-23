/**
 * Build-time renderer for the slider module.
 */
import type { Root, RootContent } from "mdast";
import { visit } from "unist-util-visit";
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/** Bounds of a slider without explicit attributes. */
const DEFAULT_MIN = 0;
const DEFAULT_MAX = 100;
const DEFAULT_STEP = 1;

/**
 * Read a number from a directive attribute.
 *
 * A missing or broken attribute falls back to the given default.
 */
const readNumber = (raw: string | null | undefined, fallback: number): number => {
	const text = raw?.trim();
	if (!text) return fallback;
	const value = Number(text);
	return Number.isFinite(value) ? value : fallback;
};

/**
 * Read the step of the slider.
 *
 * The raw text of the attribute keeps the decimals of the author, so a step of
 * "0.010" gets three decimals and not one. A broken step falls back to the
 * default, and a range input takes the default too.
 */
const readStep = (raw: string | null | undefined): { text: string; decimals: number } => {
	const text = raw?.trim() ?? "";
	if (!text || !Number.isFinite(Number(text))) return { text: String(DEFAULT_STEP), decimals: 0 };
	const match = /\.(\d+)$/.exec(text);
	return { text, decimals: match ? match[1].length : 0 };
};

/**
 * Put a value on the grid of the step.
 *
 * A range input moves a value that does not sit on the grid of the step to the
 * closest point of the grid. The build time does the same, so the readout and
 * the input agree before the client script runs.
 */
const snap = (value: number, min: number, max: number, step: number): number => {
	const clamped = Math.min(Math.max(value, min), max);
	if (!Number.isFinite(step) || step <= 0) return clamped;
	const steps = Math.round((clamped - min) / step);
	const snapped = Math.min(Math.max(min + steps * step, min), max);
	return Number(snapped.toFixed(10));
};

/**
 * Build the slider of one directive.
 *
 * The directive becomes the root of the widget. The markdown of the directive
 * stays inside the widget, so tables, formulas, code and images keep their
 * normal rendering.
 */
export const renderSlider = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const attributes = node.attributes ?? {};
	const label = attributes.label?.trim() || "参数";
	const min = readNumber(attributes.min, DEFAULT_MIN);
	const max = readNumber(attributes.max, DEFAULT_MAX);
	const step = readStep(attributes.step);
	const value = snap(readNumber(attributes.value, min), min, max, Number(step.text));
	const span = max - min;
	const position = span > 0 ? Number(((value - min) / span).toFixed(6)) : 0;
	const text = value.toFixed(step.decimals);
	const id = context.uniqueId("iw-slider");

	// The markdown of the widget, before it moves into the body.
	const content: RootContent[] = node.children;
	const tree: Root = { type: "root", children: content };

	// Headings of the widget stay out of the table of contents.
	visit(tree, "heading", heading => {
		heading.data = { ...heading.data, hName: "p" };
	});

	// The zoom of medium-zoom does not work inside a widget.
	visit(tree, "image", image => {
		image.data = { ...image.data, hProperties: { ...image.data?.hProperties, "data-nozoom": "true" } };
	});

	const head = context.element("div", { className: ["iw-slider-head"] }, [
		context.element("label", { className: ["iw-slider-label"], htmlFor: id }, [{ type: "text", value: label }]),
		context.element("span", { className: ["iw-slider-readout"] }, [{ type: "text", value: text }]),
		context.element("span", { className: ["iw-slider-init-readout"] }, [{ type: "text", value: text }])
	]);

	const range = context.element("input", {
		type: "range",
		className: ["iw-slider-range"],
		id,
		min: String(min),
		max: String(max),
		step: step.text,
		value: String(value),
		ariaLabel: label
	});

	const body = context.element("div", { className: ["iw-slider-body"] }, content);

	node.data = {
		...node.data,
		hName: "div",
		hProperties: {
			className: ["iw-slider", "iw-widget"],
			/*
			 * The live values sit next to the values of the author. The printer
			 * takes the values of the author, because only they are the same for
			 * every reader.
			 */
			style: `--v:${position};--value:${value};--iw-slider-init-v:${position};--iw-slider-init-value:${value}`
		}
	};
	node.children = [head, range, body];

	return true;
};

export default renderSlider;
