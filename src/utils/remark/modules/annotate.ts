/**
 * Build-time renderer for the annotate module.
 */
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

export const renderAnnotate = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	void node;
	void context;
	return false;
};

export default renderAnnotate;
