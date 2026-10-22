/**
 * Build-time renderer for the chart module.
 */
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

export const renderChart = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	void node;
	void context;
	return false;
};

export default renderChart;
