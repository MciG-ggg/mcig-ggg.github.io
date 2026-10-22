import { initAnnotate } from "./interactive/annotate";
import { initChart } from "./interactive/chart";
import { initCode } from "./interactive/code";
import { initCompare } from "./interactive/compare";
import { initSlider } from "./interactive/slider";
import { initSteps } from "./interactive/steps";
import { initTable } from "./interactive/table";

/**
 * Set up every interactive module on the current page.
 *
 * The function runs again after every soft navigation, so every module must
 * be safe to call on a page it already handled. Each module marks the
 * elements it prepared, and skips the ones that carry the mark.
 */
export const initInteractive = (): void => {
	initChart();
	initSlider();
	initCompare();
	initSteps();
	initCode();
	initAnnotate();
	initTable();
};
