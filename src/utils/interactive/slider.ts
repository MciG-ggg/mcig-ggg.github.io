/**
 * Set up the slider module on the current page.
 */

/** Marker of a widget that is already set up. */
const READY = "data-iw-ready";

/**
 * Count the decimals of a step value.
 *
 * The step decides how many decimals the readout shows. A broken step falls
 * back to the default, and a range input takes the default too.
 */
const decimalsOf = (step: string): number => {
	const text = step.trim();
	if (!text || !Number.isFinite(Number(text))) return 0;
	const match = /\.(\d+)$/.exec(text);
	return match ? match[1].length : 0;
};

/**
 * Write the value of one range input into the widget around it.
 *
 * The root node gets two custom properties: `--v` is the position of the
 * slider between 0 and 1, `--value` is the number of the slider itself. The
 * readout shows the number with the decimals of the step.
 */
const apply = (input: HTMLInputElement): void => {
	const root = input.closest<HTMLElement>(".iw-slider");
	if (!root) return;

	const min = Number(input.min);
	const max = Number(input.max);
	const value = Number(input.value);
	const span = max - min;
	const position = span > 0 ? Number(((value - min) / span).toFixed(6)) : 0;

	root.style.setProperty("--v", String(position));
	root.style.setProperty("--value", String(value));

	const readout = root.querySelector<HTMLElement>(".iw-slider-readout");
	if (readout) readout.textContent = value.toFixed(decimalsOf(input.getAttribute("step") ?? "1"));
};

/**
 * Set up the slider module on the current page.
 *
 * The listener sits on the document, so a widget that arrives with a soft
 * navigation works without a new listener. The marker on the document root
 * keeps the listener from being added twice.
 */
export const initSlider = (): void => {
	const documentRoot = document.documentElement;

	if (documentRoot.dataset.iwSliderBound !== "true") {
		documentRoot.dataset.iwSliderBound = "true";
		document.addEventListener("input", event => {
			const target = event.target;
			if (target instanceof HTMLInputElement && target.classList.contains("iw-slider-range")) apply(target);
		});
	}

	document.querySelectorAll<HTMLElement>(".iw-slider").forEach(root => {
		if (root.hasAttribute(READY)) return;
		root.setAttribute(READY, "true");
		const input = root.querySelector<HTMLInputElement>(".iw-slider-range");
		if (input) apply(input);
	});
};
