/**
 * Build-time renderer for the compare module.
 */
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

export const renderCompare = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	void node;
	void context;
	return false;
};

export default renderCompare;
