/**
 * The class of the button that shows one answer.
 */
const TOGGLE = "iw-quiz-toggle";

/**
 * The class of the button that shows every answer of one set.
 */
const SHOW_ALL = "iw-quiz-show-all";

/**
 * The class of the box that holds the answer.
 */
const ANSWER = "iw-quiz-answer";

/**
 * The label of a button while the answer is hidden.
 */
const HIDDEN_LABEL = "显示答案";

/**
 * The label of a button while the answer is shown.
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
 * Find the answer box that one button controls.
 *
 * @param button - The button the reader pressed.
 * @returns The answer box, or null when the button controls nothing.
 */
const findAnswer = (button: HTMLElement): HTMLElement | null => {
	const id = button.getAttribute("aria-controls");
	if (!id) return null;
	return document.getElementById(id);
};

/**
 * Show or hide one answer.
 *
 * @param button - The button the reader pressed.
 */
const toggleAnswer = (button: HTMLElement): void => {
	const answer = findAnswer(button);
	if (!answer) return;

	const shown = answer.hasAttribute("hidden");
	if (shown) {
		answer.removeAttribute("hidden");
	} else {
		answer.setAttribute("hidden", "");
	}
	button.setAttribute("aria-expanded", String(shown));
	button.textContent = shown ? SHOWN_LABEL : HIDDEN_LABEL;
};

/**
 * Show or hide every answer of one set.
 *
 * @param button - The button the reader pressed.
 */
const toggleAll = (button: HTMLElement): void => {
	const set = button.closest(".iw-quiz");
	if (!set) return;

	const answers = Array.from(set.querySelectorAll<HTMLElement>(`.${ANSWER}`));
	const show = answers.some(answer => answer.hasAttribute("hidden"));

	for (const answer of answers) {
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
			toggleAnswer(toggle);
			return;
		}

		const showAll = target.closest<HTMLElement>(`.${SHOW_ALL}`);
		if (showAll) {
			toggleAll(showAll);
		}
	});
};
