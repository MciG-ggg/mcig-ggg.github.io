/**
 * Set up the table module on the current page.
 *
 * The module adds a sort button to the header of every numeric column of a
 * sortable table. Clicking the button reorders the rows of the table in place,
 * so the markdown inside the table is never rendered again.
 */

/** The class of the button that sorts one column. */
const BUTTON_CLASS = "iw-sortable-button";

/** The class of the wrapper that the build-time renderer puts around the table. */
const ROOT_SELECTOR = ".iw-sortable";

/** The text of a cell, without the surrounding whitespace. */
const cellText = (cell: HTMLTableCellElement | undefined): string => (cell?.textContent ?? "").trim();

/**
 * Check whether a text holds a number.
 *
 * A text with a unit, such as "12 ms", is not a number.
 */
const isNumber = (text: string): boolean => text !== "" && Number.isFinite(Number(text));

/**
 * The indexes of the numeric columns.
 *
 * A column counts as numeric when every cell that has content holds a number.
 * The empty cells do not count. A column without any content is not numeric,
 * because there is nothing to sort.
 */
const numericColumnIndexes = (columns: string[][]): number[] => {
	const indexes: number[] = [];

	for (let index = 0; index < columns.length; index++) {
		const values = columns[index].filter(text => text !== "");
		if (values.length === 0) continue;
		if (!values.every(isNumber)) continue;
		indexes.push(index);
	}

	return indexes;
};

/**
 * Compare two cells of a column.
 *
 * A cell without a number goes to the end of the table, in both directions.
 */
const compareCells = (left: string, right: string, ascending: boolean): number => {
	const leftValue = left === "" ? Number.NaN : Number(left);
	const rightValue = right === "" ? Number.NaN : Number(right);

	if (Number.isNaN(leftValue) || Number.isNaN(rightValue)) {
		if (Number.isNaN(leftValue) && Number.isNaN(rightValue)) return 0;
		return Number.isNaN(leftValue) ? 1 : -1;
	}

	return ascending ? leftValue - rightValue : rightValue - leftValue;
};

/**
 * Sort the rows of a table by one column.
 *
 * The rows keep their nodes. Only their order in the table changes.
 */
const sortRows = (body: HTMLTableSectionElement, index: number, ascending: boolean): void => {
	const rows = Array.from(body.rows);

	rows.sort((a, b) => compareCells(cellText(a.cells[index]), cellText(b.cells[index]), ascending));

	for (const row of rows) body.append(row);
};

/**
 * Add the sort buttons to one sortable table.
 */
const prepare = (root: HTMLElement): void => {
	if (root.dataset.iwReady === "true") return;

	const table = root.querySelector("table");
	const header = table?.tHead?.rows[0];
	const body = table?.tBodies[0];
	if (!table || !header || !body) return;

	const rows = Array.from(body.rows);
	const columns = Array.from(header.cells, (_, index) => rows.map(row => cellText(row.cells[index])));

	for (const index of numericColumnIndexes(columns)) {
		const cell = header.cells[index];
		const name = cellText(cell);

		const button = document.createElement("button");
		button.type = "button";
		button.className = BUTTON_CLASS;
		button.dataset.iwColumn = String(index);
		button.dataset.iwName = name;
		button.setAttribute("aria-label", `按「${name}」列排序`);
		button.textContent = "↕";

		cell.append(button);
	}

	root.dataset.iwReady = "true";
};

/**
 * Sort the table that holds the button, and show the new direction.
 */
const onClick = (event: MouseEvent): void => {
	const target = event.target;
	if (!(target instanceof Element)) return;

	const button = target.closest<HTMLButtonElement>(`.${BUTTON_CLASS}`);
	if (!button) return;

	const table = button.closest("table");
	const body = table?.tBodies[0];
	if (!table || !body) return;

	const index = Number(button.dataset.iwColumn);
	if (!Number.isInteger(index)) return;

	// The first click sorts up, the next one flips the direction.
	const ascending = button.dataset.iwDirection !== "asc";

	sortRows(body, index, ascending);

	// Only the sorted column shows a direction. The other sortable columns go
	// back to the state they had before.
	for (const other of table.querySelectorAll<HTMLButtonElement>(`.${BUTTON_CLASS}`)) {
		const name = other.dataset.iwName ?? "";
		const sorted = other === button;

		other.dataset.iwDirection = sorted ? (ascending ? "asc" : "desc") : "";
		other.textContent = sorted ? (ascending ? "↑" : "↓") : "↕";
		other.setAttribute("aria-label", sorted ? `按「${name}」列${ascending ? "升序" : "降序"}排序` : `按「${name}」列排序`);
		other.parentElement?.setAttribute("aria-sort", sorted ? (ascending ? "ascending" : "descending") : "none");
	}
};

export const initTable = (): void => {
	for (const root of document.querySelectorAll<HTMLElement>(ROOT_SELECTOR)) {
		prepare(root);
	}

	// The click handler is bound to the document, so it survives the soft
	// navigation. The flag keeps it from being bound twice.
	const flag = document.documentElement;
	if (flag.dataset.iwTableBound === "true") return;
	flag.dataset.iwTableBound = "true";

	document.addEventListener("click", onClick);
};
