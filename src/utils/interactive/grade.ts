/**
 * The grading rules of the quiz module.
 *
 * The functions take the values the reader gave and the key of the question.
 * They hold no DOM, so they can be tested without a browser.
 */

/**
 * Split a key into its parts.
 *
 * A multiple choice key lists the option numbers, separated by commas. A
 * fill in the blank key lists the texts, separated by a pipe, because a
 * Chinese answer may hold a comma.
 *
 * @param key - The key of one question.
 * @param separator - The character that separates the parts.
 * @returns The parts of the key.
 */
const splitKey = (key: string, separator: string): string[] =>
	key
		.split(separator)
		.map(part => part.trim())
		.filter(Boolean);

/**
 * Grade a multiple choice question.
 *
 * @param picked - The values of the options the reader chose.
 * @param key - The key of the question.
 * @returns True when the chosen options match the key.
 */
export const gradeChoice = (picked: string[], key: string): boolean => {
	const expected = splitKey(key, ",").sort();
	const chosen = [...picked].sort();
	if (chosen.length === 0 || chosen.length !== expected.length) return false;
	return chosen.every((value, index) => value === expected[index]);
};

/**
 * Tell whether one blank matches its key.
 *
 * The match is exact after trimming, but a longer answer that contains the
 * key also counts. A reader who writes the full term is not wrong because
 * the key holds the short form.
 *
 * @param value - What the reader typed.
 * @param key - The key of the blank.
 * @returns True when the blank is right.
 */
const matchBlank = (value: string, key: string): boolean => {
	const typed = value.trim();
	if (typed === key) return true;
	return typed.includes(key);
};

/**
 * Grade a fill in the blank question.
 *
 * @param values - The texts the reader typed into the blanks.
 * @param key - The key of the question.
 * @returns True when every blank matches its part of the key.
 */
export const gradeFill = (values: string[], key: string): boolean => {
	const expected = splitKey(key, "|");
	if (values.length === 0 || values.length !== expected.length) return false;
	return values.every((value, index) => matchBlank(value, expected[index]));
};
