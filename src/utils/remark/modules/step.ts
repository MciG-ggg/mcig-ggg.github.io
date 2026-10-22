/**
 * Build-time renderer for the step module.
 */
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

export const renderStep = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	void node;
	void context;
	return false;
};

export default renderStep;
