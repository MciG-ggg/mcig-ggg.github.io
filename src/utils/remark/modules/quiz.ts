import type { RootContent } from "mdast";
import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/** The class of the box that holds one question. */
const QUESTION_CLASS = "iw-quiz-q";

/** The class of the box that holds the answer. */
const ANSWER_CLASS = "iw-quiz-answer";

/** The class of the head bar that holds the type, the points and the button. */
const HEAD_CLASS = "iw-quiz-head";

/** The class of the button that shows the answer. */
const TOGGLE_CLASS = "iw-quiz-toggle";

/** The class of the button that shows every answer of one set. */
const SHOW_ALL_CLASS = "iw-quiz-show-all";

/** The label of the button while the answer is hidden. */
const HIDDEN_LABEL = "显示答案";

/**
 * Find the thematic break that separates the question from its answer.
 *
 * The author writes the question, an empty line, a `---` line, an empty
 * line, and then the answer. The rule needs a blank line before it, or the
 * line under the last paragraph turns into a heading.
 *
 * @param children - The children of the question node.
 * @returns The index of the break, or -1 when the question holds no answer.
 */
const findAnswerBreak = (children: RootContent[]): number => children.findIndex(child => child.type === "thematicBreak");

/**
 * Build the head bar of one question.
 *
 * @param context - The tools of the module.
 * @param id - The id of the answer box the button controls.
 * @param type - The type of the question.
 * @param points - The points of the question.
 * @returns The mdast nodes of the head bar.
 */
const buildHead = (context: InteractiveContext, id: string, type: string, points: string): RootContent[] => {
	const badge = type ? context.element("span", { className: ["iw-quiz-type"] }, [{ type: "text", value: type }]) : null;
	const score = points ? context.element("span", { className: ["iw-quiz-points"] }, [{ type: "text", value: `${points} 分` }]) : null;
	const button: RootContent = {
		type: "html",
		value: `<button class="${TOGGLE_CLASS}" type="button" aria-expanded="false" aria-controls="${id}">${HIDDEN_LABEL}</button>`
	};

	const children: RootContent[] = [];
	if (badge) children.push(badge);
	if (score) children.push(score);
	children.push(button);
	return children;
};

/**
 * Build the renderer of one question.
 *
 * The question holds the text of the question, then a thematic break, then
 * the answer. The answer sits in a box that stays hidden until the reader
 * presses the button.
 */
export const renderQ = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const type = node.attributes?.type ?? "";
	const points = node.attributes?.points ?? "";
	const id = context.uniqueId("iw-quiz-answer");

	const children = node.children ?? [];
	const split = findAnswerBreak(children);
	const questionChildren = split < 0 ? children : children.slice(0, split);
	const answerChildren = split < 0 ? [] : children.slice(split + 1);

	const answer = context.element("div", { className: [ANSWER_CLASS], id, hidden: true }, answerChildren);

	node.data = node.data ?? {};
	node.data.hName = "div";
	node.data.hProperties = {
		className: [QUESTION_CLASS],
		...(type ? { "data-type": type } : {}),
		...(points ? { "data-points": points } : {})
	};
	node.children = [
		context.element("div", { className: [HEAD_CLASS] }, buildHead(context, id, type, points)),
		context.element("div", { className: ["iw-quiz-body"] }, questionChildren),
		answer
	];

	return true;
};

/**
 * Build the renderer of one set of questions.
 *
 * The set holds a title and the questions. The renderer boxes the set and
 * adds a button that shows every answer at once. It does not take over the
 * children, so the dispatcher walks into the questions and renders them too.
 */
export const renderQuiz = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const title = node.attributes?.title ?? "";
	const id = context.uniqueId("iw-quiz");

	const children: RootContent[] = [];
	if (title) {
		children.push(context.element("div", { className: ["iw-quiz-title"] }, [{ type: "text", value: title }]));
	}
	children.push({
		type: "html",
		value: `<button class="${SHOW_ALL_CLASS}" type="button" aria-controls="${id}">全部显示答案</button>`
	});

	node.data = node.data ?? {};
	node.data.hName = "div";
	node.data.hProperties = { className: ["iw-quiz"], id };
	// The questions follow the head of the set. The dispatcher walks into them.
	node.children = [context.element("div", { className: ["iw-quiz-bar"] }, children), ...(node.children ?? [])];

	return false;
};

export default renderQ;
