import { gradeChoice as gradeChoiceValues, gradeFill as gradeFillValues } from "./grade";

/**
 * The class of the button that shows one answer.
 */
const TOGGLE = "iw-quiz-toggle";

/**
 * The class of the button that shows every answer of one set.
 */
const SHOW_ALL = "iw-quiz-show-all";

/**
 * The class of the button that grades one set.
 */
const SUBMIT = "iw-quiz-submit";

/**
 * The class of the box that holds the answer.
 */
const ANSWER = "iw-quiz-answer";

/**
 * The class of the box that shows the score.
 */
const SCORE = "iw-quiz-score";

/**
 * The class of the box that holds one question.
 */
const QUESTION = "iw-quiz-q";

/**
 * The class of the input of a multiple choice question.
 */
const CHOICE = "iw-quiz-choice";

/**
 * The class of the input of a blank.
 */
const BLANK = "iw-quiz-blank";

/**
 * The class of the box that lets the reader mark an open question.
 */
const SELF_CHECK = "iw-quiz-self-check";

/**
 * The class of the input inside the self check box.
 */
const SELF_CHECK_INPUT = "iw-quiz-self-check-input";

/**
 * The label of the button while the answer is hidden.
 */
const HIDDEN_LABEL = "显示答案";

/**
 * The label of the button while the answer is shown.
 */
const SHOWN_LABEL = "隐藏答案";

/**
 * The label of the button of a set while every answer is hidden.
 */
const ALL_HIDDEN_LABEL = "全部显示答案";

/**
 * The label of the button of a set while every answer is shown.
 */
const ALL_SHOWN_LABEL = "全部隐藏答案";

/**
 * The points of one question.
 *
 * @param question - The question element.
 * @returns The points, or 0 when the question carries none.
 */
const pointsOf = (question: HTMLElement): number => {
	const value = Number(question.dataset.points ?? "0");
	return Number.isFinite(value) ? value : 0;
};

/**
 * Tell whether a question is graded by the client.
 *
 * @param question - The question element.
 * @returns True for the objective questions.
 */
const isObjective = (question: HTMLElement): boolean => {
	const kind = question.dataset.kind ?? "none";
	return kind === "choice" || kind === "fill";
};

/**
 * Grade a multiple choice question.
 *
 * @param question - The question element.
 * @returns True when the picked options match the key.
 */
const gradeChoice = (question: HTMLElement): boolean => {
	const inputs = Array.from(question.querySelectorAll<HTMLInputElement>(`input.${CHOICE}`));
	const picked = inputs.filter(input => input.checked).map(input => input.value);
	return gradeChoiceValues(picked, question.dataset.answer ?? "");
};

/**
 * Grade a fill in the blank question.
 *
 * @param question - The question element.
 * @returns True when every blank matches its key.
 */
const gradeFill = (question: HTMLElement): boolean => {
	const inputs = Array.from(question.querySelectorAll<HTMLInputElement>(`input.${BLANK}`));
	const values = inputs.map(input => input.value);
	return gradeFillValues(values, question.dataset.answer ?? "");
};

/**
 * Show or hide one answer.
 *
 * @param button - The button the reader pressed.
 * @param show - True shows the answer, false hides it.
 */
const setAnswer = (button: HTMLElement, show: boolean): void => {
	const id = button.getAttribute("aria-controls");
	if (!id) return;
	const answer = document.getElementById(id);
	if (!answer) return;

	if (show) {
		answer.removeAttribute("hidden");
	} else {
		answer.setAttribute("hidden", "");
	}
	button.setAttribute("aria-expanded", String(show));
	button.textContent = show ? SHOWN_LABEL : HIDDEN_LABEL;
};

/**
 * Show or hide every answer of one set.
 *
 * @param button - The button the reader pressed.
 * @param show - True shows the answers, false hides them.
 */
const setAllAnswers = (button: HTMLElement, show: boolean): void => {
	const set = button.closest<HTMLElement>(".iw-quiz");
	if (!set) return;

	for (const answer of set.querySelectorAll<HTMLElement>(`.${ANSWER}`)) {
		if (show) {
			answer.removeAttribute("hidden");
		} else {
			answer.setAttribute("hidden", "");
		}
	}
	for (const toggle of set.querySelectorAll<HTMLElement>(`.${TOGGLE}`)) {
		toggle.setAttribute("aria-expanded", String(show));
		toggle.textContent = show ? SHOWN_LABEL : HIDDEN_LABEL;
	}
	button.textContent = show ? ALL_SHOWN_LABEL : ALL_HIDDEN_LABEL;
};

/**
 * Mark one question with the points the reader got.
 *
 * @param question - The question element.
 * @param right - Whether the answer was right.
 * @param points - The points of the question.
 */
const markQuestion = (question: HTMLElement, right: boolean, points: number): void => {
	question.classList.remove("iw-quiz-q-right", "iw-quiz-q-wrong");
	question.classList.add(right ? "iw-quiz-q-right" : "iw-quiz-q-wrong");

	question.querySelector(".iw-quiz-mark")?.remove();

	const badge = document.createElement("span");
	badge.className = "iw-quiz-mark";
	badge.textContent = right ? `✓ ${points} 分` : "✗ 0 分";
	question.querySelector(".iw-quiz-head")?.appendChild(badge);
};

/**
 * Add the box that lets the reader mark an open question.
 *
 * @param question - The question element.
 * @param points - The points of the question.
 */
const addSelfCheck = (question: HTMLElement, points: number): void => {
	if (question.querySelector(`.${SELF_CHECK}`)) return;

	const box = document.createElement("label");
	box.className = SELF_CHECK;

	const input = document.createElement("input");
	input.type = "checkbox";
	input.className = SELF_CHECK_INPUT;

	const text = document.createElement("span");
	text.textContent = `我答对了（${points} 分）`;

	box.appendChild(input);
	box.appendChild(text);
	question.querySelector(".iw-quiz-head")?.appendChild(box);
};

/**
 * Write the score of one set.
 *
 * @param set - The set element.
 */
const renderScore = (set: HTMLElement): void => {
	const score = set.querySelector<HTMLElement>(`.${SCORE}`);
	if (!score) return;

	const questions = Array.from(set.querySelectorAll<HTMLElement>(`.${QUESTION}`));
	const objective = questions.filter(isObjective).reduce((sum, question) => sum + pointsOf(question), 0);
	const total = questions.reduce((sum, question) => sum + pointsOf(question), 0);
	const graded = Number(score.dataset.earned ?? "0");
	const self = Array.from(set.querySelectorAll<HTMLInputElement>(`input.${SELF_CHECK_INPUT}`)).reduce(
		(sum, input) => sum + (input.checked ? pointsOf(input.closest<HTMLElement>(`.${QUESTION}`) as HTMLElement) : 0),
		0
	);

	score.textContent = `客观题 ${graded} / ${objective} 分`;
	if (total > objective) score.textContent += ` ｜ 总分 ${graded + self} / ${total} 分`;
};

/**
 * Grade every question of one set and reveal the answers.
 *
 * @param button - The button the reader pressed.
 */
const gradeSet = (button: HTMLElement): void => {
	const set = button.closest<HTMLElement>(".iw-quiz");
	if (!set) return;

	const questions = Array.from(set.querySelectorAll<HTMLElement>(`.${QUESTION}`));
	let earned = 0;

	for (const question of questions) {
		const points = pointsOf(question);
		const kind = question.dataset.kind ?? "none";

		if (kind === "choice" || kind === "fill") {
			const right = kind === "choice" ? gradeChoice(question) : gradeFill(question);
			if (right) earned += points;
			markQuestion(question, right, points);
		} else if (kind === "essay") {
			addSelfCheck(question, points);
		}

		const toggle = question.querySelector<HTMLElement>(`.${TOGGLE}`);
		if (toggle) setAnswer(toggle, true);
	}

	const score = set.querySelector<HTMLElement>(`.${SCORE}`);
	if (score) {
		score.dataset.earned = String(earned);
		score.dataset.objective = String(earned);
	}

	const showAll = set.querySelector<HTMLElement>(`.${SHOW_ALL}`);
	if (showAll) showAll.textContent = ALL_SHOWN_LABEL;

	renderScore(set);
};

/**
 * Set up the quiz module on the current page.
 *
 * The listeners sit on the document, so a soft navigation keeps them. The
 * module only reads the markup, so the function is safe to call again.
 */
export const initQuiz = (): void => {
	const root = document.documentElement;
	if (root.dataset.iwQuizBound === "true") return;
	root.dataset.iwQuizBound = "true";

	document.addEventListener("click", event => {
		const target = event.target;
		if (!(target instanceof Element)) return;

		const toggle = target.closest<HTMLElement>(`.${TOGGLE}`);
		if (toggle) {
			const shown = toggle.getAttribute("aria-expanded") === "true";
			setAnswer(toggle, !shown);
			return;
		}

		const showAll = target.closest<HTMLElement>(`.${SHOW_ALL}`);
		if (showAll) {
			const shown = showAll.textContent === ALL_SHOWN_LABEL;
			setAllAnswers(showAll, !shown);
			return;
		}

		const submit = target.closest<HTMLElement>(`.${SUBMIT}`);
		if (submit) {
			gradeSet(submit);
		}
	});

	document.addEventListener("change", event => {
		const target = event.target;
		if (!(target instanceof HTMLInputElement)) return;
		if (!target.classList.contains(SELF_CHECK_INPUT)) return;

		const question = target.closest<HTMLElement>(`.${QUESTION}`);
		const set = question?.closest<HTMLElement>(".iw-quiz");
		if (!set) return;
		renderScore(set);
	});
};
