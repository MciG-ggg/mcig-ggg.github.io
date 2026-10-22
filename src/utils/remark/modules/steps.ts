/**
 * Build-time renderer for the steps module.
 */
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

export const renderSteps = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	void node;
	void context;
	return false;
};

export default renderSteps;
