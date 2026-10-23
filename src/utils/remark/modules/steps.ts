/**
 * Build-time renderer for the steps module.
 */
import type { Properties } from "hast";
import type { RootContent } from "mdast";
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";
import { renderStep } from "./step";

/**
 * Check whether a child of a steps directive is a step directive.
 */
const isStep = (child: RootContent): boolean => {
	const node = child as unknown as ContainerDirectiveNode;
	return node.type === "containerDirective" && node.name === "step";
};

/**
 * Build the steps widget.
 *
 * The widget holds a bar of tabs, one radio input per step, and one panel per
 * step. The panels keep the markdown of the steps, so the content of a step
 * renders as usual.
 */
export const renderSteps = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const steps = node.children.filter(isStep) as unknown as ContainerDirectiveNode[];
	if (steps.length === 0) return false;

	// One group name per widget, so two widgets on one page stay apart.
	const group = context.uniqueId("iw-steps");
	const title = node.attributes?.title ?? "";
	const tabs: RootContent[] = [];
	const panels: RootContent[] = [];

	steps.forEach((step, index) => {
		const id = context.uniqueId("iw-step");
		// A step without a title gets a number, so the label and the panel
		// carry the same name.
		const stepTitle = step.attributes?.title?.trim() || `第 ${index + 1} 步`;
		step.attributes = { ...step.attributes, title: stepTitle };
		const radio: Properties = { type: "radio", name: group, id, className: ["iw-steps-radio"] };

		if (index === 0) radio.checked = true;

		// The radio sits next to its label, so the label shows the state.
		tabs.push(context.element("input", radio, []));
		tabs.push(context.element("label", { htmlFor: id, className: ["iw-steps-label"] }, [{ type: "text", value: stepTitle }]));

		renderStep(step, context);
		panels.push(step as unknown as RootContent);
	});

	const children: RootContent[] = [];

	if (title) {
		children.push(context.element("p", { className: ["iw-steps-title"] }, [{ type: "text", value: title }]));
	}

	// The radio group takes the name of the group, so a screen reader says
	// which group of steps it reads.
	const tabProperties: Properties = { className: ["iw-steps-tabs"], role: "group", ariaLabel: title || "步骤" };

	children.push(context.element("div", tabProperties, tabs));
	children.push(context.element("div", { className: ["iw-steps-panels"] }, panels));

	node.data = { ...node.data, hName: "div", hProperties: { className: ["iw-steps", "iw-widget"] } };
	node.children = children;

	return true;
};

export default renderSteps;
