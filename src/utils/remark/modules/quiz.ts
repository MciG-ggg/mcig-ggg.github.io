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

/** The class of the button that grades the set. */
const SUBMIT_CLASS = "iw-quiz-submit";

/** The class of the box that shows the score. */
const SCORE_CLASS = "iw-quiz-score";

/** The class of the list of options. */
const OPTIONS_CLASS = "iw-quiz-options";

/** The class of one option. */
const OPTION_CLASS = "iw-quiz-option";

/** The class of the input of a blank. */
const BLANK_CLASS = "iw-quiz-blank";

/** The class of the text area of an open question. */
const ESSAY_CLASS = "iw-quiz-essay";

/** The label of the button while the answer is hidden. */
const HIDDEN_LABEL = "显示答案";

/** The marker in the text that stands for one blank. */
const BLANK = "{{blank}}";

/** The kinds of question that the client can grade. */
type QuestionKind = "choice" | "fill" | "essay" | "none";

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
 * Pull the options of a multiple choice question out of the body.
 *
 * The options are an ordered list. The list is replaced by one input per
 * option, so the reader can pick an answer.
 *
 * @param children - The children of the question.
 * @param name - The name that groups the inputs of the question.
 * @param multiple - Whether the reader may pick more than one option.
 * @returns The options and the body without the list.
 */
const extractOptions = (
	context: InteractiveContext,
	children: RootContent[],
	name: string,
	multiple: boolean
): { inputs: RootContent[]; rest: RootContent[] } => {
	const inputs: RootContent[] = [];
	const rest: RootContent[] = [];

	for (const child of children) {
		if (child.type === "list" && child.ordered && inputs.length === 0) {
			let index = 0;
			for (const item of child.children) {
				index += 1;
				const type = multiple ? "checkbox" : "radio";
				const parts: RootContent[] = [
					{
						type: "html",
						value: `<input type="${type}" class="iw-quiz-choice" name="${name}" value="${index}">`
					},
					{ type: "text", value: " " }
				];
				for (const part of item.children as RootContent[]) parts.push(part);
				inputs.push(context.element("label", { className: [OPTION_CLASS] }, parts));
			}
			continue;
		}
		rest.push(child);
	}

	return { inputs, rest };
};

/**
 * Replace every blank marker in the body with a text input.
 *
 * @param children - The children of the question.
 * @param name - The name that groups the inputs of the question.
 * @returns The body with the inputs and the number of blanks.
 */
const injectBlanks = (children: RootContent[], name: string): { children: RootContent[]; count: number } => {
	let count = 0;

	const walk = (nodes: RootContent[]): RootContent[] => {
		const out: RootContent[] = [];
		for (const node of nodes) {
			if (node.type === "text" && node.value.includes(BLANK)) {
				const parts = node.value.split(BLANK);
				parts.forEach((part, index) => {
					if (part) out.push({ type: "text", value: part });
					if (index < parts.length - 1) {
						count += 1;
						out.push({
							type: "html",
							value: `<input type="text" class="${BLANK_CLASS}" data-blank="${count}" name="${name}-blank-${count}" aria-label="第 ${count} 空">`
						});
					}
				});
				continue;
			}
			if ("children" in node) {
				(node as { children: RootContent[] }).children = walk((node as { children: RootContent[] }).children);
			}
			out.push(node);
		}
		return out;
	};

	return { children: walk(children), count };
};

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
 *
 * The type decides what the reader gets. A multiple choice question turns its
 * ordered list into one input per option. A fill in the blank question turns
 * every blank marker into a text input. An open question gets a text area.
 */
export const renderQ = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const type = node.attributes?.type ?? "";
	const points = node.attributes?.points ?? "";
	const key = node.attributes?.answer ?? "";
	const multiple = node.attributes?.multiple === "true";
	const id = context.uniqueId("iw-quiz-answer");
	const group = context.uniqueId("iw-quiz-group");

	const children = node.children ?? [];
	const split = findAnswerBreak(children);
	let questionChildren = split < 0 ? children : children.slice(0, split);
	const answerChildren = split < 0 ? [] : children.slice(split + 1);

	let kind: QuestionKind = "none";
	let inputs: RootContent[] = [];

	if (type === "判断") {
		inputs = [1, 2].map(index =>
			context.element("label", { className: [OPTION_CLASS] }, [
				{
					type: "html",
					value: `<input type="radio" class="iw-quiz-choice" name="${group}" value="${index}">`
				},
				{ type: "text", value: ` ${index === 1 ? "正确" : "错误"}` }
			])
		);
		kind = "choice";
	} else if (type === "选择" || type === "多选") {
		const picked = extractOptions(context, questionChildren, group, multiple || type === "多选");
		questionChildren = picked.rest;
		inputs = picked.inputs;
		kind = picked.inputs.length > 0 ? "choice" : "none";
	} else if (type === "填空") {
		const filled = injectBlanks(questionChildren, group);
		questionChildren = filled.children;
		kind = filled.count > 0 ? "fill" : "none";
	} else if (type === "程序" || type === "解答") {
		kind = "essay";
	}

	const body: RootContent[] = [...questionChildren];
	if (inputs.length > 0) {
		body.push(context.element("div", { className: [OPTIONS_CLASS] }, inputs));
	}
	if (kind === "essay") {
		body.push({
			type: "html",
			value: `<textarea class="${ESSAY_CLASS}" rows="4" placeholder="写下你的答案"></textarea>`
		});
	}

	const answer = context.element("div", { className: [ANSWER_CLASS], id, hidden: true }, answerChildren);

	node.data = node.data ?? {};
	node.data.hName = "div";
	node.data.hProperties = {
		className: [QUESTION_CLASS],
		"data-type": type,
		"data-points": points,
		"data-kind": kind,
		...(key ? { "data-answer": key } : {})
	};
	node.children = [
		context.element("div", { className: [HEAD_CLASS] }, buildHead(context, id, type, points)),
		context.element("div", { className: ["iw-quiz-body"] }, body),
		answer
	];

	return true;
};

/**
 * Build the renderer of one set of questions.
 *
 * The set holds a title and the questions. The renderer boxes the set and
 * adds the buttons that show the answers and that grade the set. It does not
 * take over the children, so the dispatcher walks into the questions and
 * renders them too.
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
		value: `<button class="${SUBMIT_CLASS}" type="button">提交答案</button>`
	});
	children.push({
		type: "html",
		value: `<button class="${SHOW_ALL_CLASS}" type="button" aria-controls="${id}">全部显示答案</button>`
	});
	children.push(context.element("span", { className: [SCORE_CLASS], role: "status" }, []));

	node.data = node.data ?? {};
	node.data.hName = "div";
	node.data.hProperties = { className: ["iw-quiz"], id };
	// The questions follow the head of the set. The dispatcher walks into them.
	node.children = [context.element("div", { className: ["iw-quiz-bar"] }, children), ...(node.children ?? [])];

	return false;
};

export default renderQ;
