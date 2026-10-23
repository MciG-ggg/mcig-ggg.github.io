/**
 * Set up the compare module on the current page.
 */

/** Marker of a widget that is already set up. */
const READY = "data-iw-ready";

/**
 * Write the position of the range input into the widget around it.
 *
 * The root node gets the custom property `--v`, the position of the slider
 * between 0 and 1. The CSS reads it for the width of the clip layer and for
 * the position of the divider.
 */
const apply = (input: HTMLInputElement): void => {
	const root = input.closest<HTMLElement>(".iw-compare");
	if (!root) return;

	const min = Number(input.min);
	const max = Number(input.max);
	const value = Number(input.value);
	const span = max - min;
	const position = span > 0 ? Number(((value - min) / span).toFixed(6)) : 0;

	root.style.setProperty("--v", String(position));
};

/**
 * Set up the compare module on the current page.
 *
 * The listener sits on the document, so a widget that arrives with a soft
 * navigation works without a new listener. The marker on the document root
 * keeps the listener from being added twice.
 */
export const initCompare = (): void => {
	const documentRoot = document.documentElement;

	if (documentRoot.dataset.iwCompareBound !== "true") {
		documentRoot.dataset.iwCompareBound = "true";
		document.addEventListener("input", event => {
			const target = event.target;
			if (target instanceof HTMLInputElement && target.classList.contains("iw-compare-range")) apply(target);
		});
	}

	document.querySelectorAll<HTMLElement>(".iw-compare").forEach(root => {
		if (root.hasAttribute(READY)) return;
		root.setAttribute(READY, "true");
		const input = root.querySelector<HTMLInputElement>(".iw-compare-range");
		if (input) apply(input);
	});
};
