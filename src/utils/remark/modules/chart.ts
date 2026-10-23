/**
 * Build-time renderer for the chart module.
 *
 * The directive holds a markdown table. The first column gives the x value,
 * every other column gives one series. The renderer draws the chart as an SVG
 * and keeps the content of the directive, so the exact numbers stay readable.
 */
import type { RootContent, Table, TableCell, TableRow } from "mdast";
import { toString as mdastToString } from "mdast-util-to-string";

import type { ContainerDirectiveNode, InteractiveContext } from "../interactive";

/** The size of the drawing area of the chart, in user units. */
const VIEW_WIDTH = 640;
const VIEW_HEIGHT = 400;

/** The empty space around the drawing area. */
const MARGIN = { top: 46, right: 24, bottom: 52, left: 64 };

/** The number of series that have a color of their own. */
const SERIES_COLORS = 6;

/** One series of the chart. */
interface Series {
	/** The name of the series. It comes from the header of the table. */
	name: string;
	/** The value of every row. A missing value is null. */
	values: (number | null)[];
}

/** The data of the chart, read from the table of the directive. */
interface ChartData {
	/** The name of the x axis. It comes from the first cell of the header. */
	xTitle: string;
	/** The label of the x axis of every row. */
	xLabels: string[];
	/** The series of the chart. */
	series: Series[];
}

/**
 * Read the text of a cell.
 *
 * The cell can hold more than plain text, so the whole content is taken.
 */
const cellText = (cell: TableCell): string => mdastToString(cell).trim();

/**
 * Read a number of a cell.
 *
 * An empty cell has no value, and the chart leaves a gap there.
 */
const parseValue = (text: string): number | null => {
	if (text === "") return null;
	const value = Number(text);
	return Number.isFinite(value) ? value : null;
};

/**
 * Format a number for a label.
 *
 * The numbers of a table are short, so the label keeps them short too.
 */
const formatNumber = (value: number): string => String(Number(value.toFixed(2)));

/**
 * Round a step up to a number that people can read.
 *
 * The steps 1, 2 and 5 times a power of ten give axis labels that are easy to
 * read at a glance.
 */
const niceStep = (raw: number): number => {
	if (!(raw > 0)) return 1;
	const exponent = Math.floor(Math.log10(raw));
	const base = 10 ** exponent;
	const fraction = raw / base;
	const factor = fraction < 1.5 ? 1 : fraction < 3 ? 2 : fraction < 7 ? 5 : 10;
	return factor * base;
};

/**
 * Find the table inside the directive.
 *
 * The table plugin of the site wraps every table in a div, so the wrapper sits
 * next to the table.
 */
const findTable = (children: RootContent[]): Table | undefined => {
	for (const child of children) {
		if (child.type === "table") return child;
	}
	return undefined;
};

/**
 * Read the data of the chart from the table.
 *
 * The first column of the header names the x axis, every other cell names a
 * series. The first column of the body gives the x value of the row, every
 * other cell gives the value of the series.
 */
const readTable = (table: Table): ChartData => {
	const rows = table.children.filter((child): child is TableRow => child.type === "tableRow");
	const cells = (row: TableRow): TableCell[] => row.children.filter((child): child is TableCell => child.type === "tableCell");

	const header = rows[0] ? cells(rows[0]) : [];
	const xTitle = header[0] ? cellText(header[0]) : "";
	const series: Series[] = header.slice(1).map(cell => ({ name: cellText(cell), values: [] }));
	const xLabels: string[] = [];

	for (const row of rows.slice(1)) {
		const rowCells = cells(row);
		if (rowCells.length === 0) continue;
		xLabels.push(cellText(rowCells[0]));
		for (let index = 0; index < series.length; index++) {
			const cell = rowCells[index + 1];
			series[index].values.push(cell ? parseValue(cellText(cell)) : null);
		}
	}

	return { xTitle, xLabels, series };
};

/**
 * Prepare the markdown of the directive.
 *
 * The table of contents collects the headings of the page. A heading inside a
 * widget is a part of the widget, so it must not show up in the contents.
 *
 * The zoom of medium-zoom does not work inside a widget, so the images of the
 * widget do not take part in it.
 */
const prepareChildren = (children: RootContent[]): void => {
	for (const child of children) {
		if (child.type === "heading") {
			child.data = { ...child.data, hName: "p" };
			continue;
		}
		if (child.type === "image") {
			child.data = { ...child.data, hProperties: { ...child.data?.hProperties, "data-nozoom": "true" } };
			continue;
		}
		if ("children" in child) prepareChildren(child.children as RootContent[]);
	}
};

export const renderChart = (node: ContainerDirectiveNode, context: InteractiveContext): boolean => {
	const attributes = node.attributes ?? {};
	const type = attributes.type === "bar" ? "bar" : "line";
	const title = attributes.title?.trim() ?? "";
	const unit = attributes.unit?.trim() ?? "";

	const table = findTable(node.children);
	if (!table) return false;

	const data = readTable(table);
	if (data.xLabels.length === 0) return false;

	// A column without a number holds text or an image, so it is no series.
	const series = data.series.filter(item => item.values.some(value => value !== null));
	if (series.length === 0) return false;

	const numbers = series.flatMap(item => item.values.filter((value): value is number => value !== null));

	prepareChildren(node.children);

	// The drawing area of the chart.
	const plot = {
		left: MARGIN.left,
		right: VIEW_WIDTH - MARGIN.right,
		top: MARGIN.top,
		bottom: VIEW_HEIGHT - MARGIN.bottom
	};

	// The range of the y axis. A bar starts at zero, because the length of a
	// bar stands for the value. A line shows the change of the values, so it
	// uses the range of the data.
	const lowest = Math.min(...numbers);
	const highest = Math.max(...numbers);
	const rawMin = type === "bar" ? Math.min(0, lowest) : lowest;
	let rawMax = type === "bar" ? Math.max(0, highest) : highest;
	if (rawMax === rawMin) rawMax = rawMin + 1;
	const step = niceStep((rawMax - rawMin) / 4);
	const yMin = Math.floor(rawMin / step) * step;
	let yMax = Math.ceil(rawMax / step) * step;
	// The top of the axis stays above the highest value, so the value does not
	// touch the frame.
	if (yMax <= highest) yMax += step;
	const yOf = (value: number): number => plot.bottom - ((value - yMin) / (yMax - yMin)) * (plot.bottom - plot.top);

	// The positions of the x axis. Numbers are placed by their value, text is
	// placed in the middle of its own band.
	const xNumbers = data.xLabels.map(label => parseValue(label));
	const numericX = xNumbers.every(value => value !== null);
	const band = (plot.right - plot.left) / data.xLabels.length;
	const positions: number[] = [];

	if (numericX) {
		const values = xNumbers as number[];
		const gaps: number[] = [];
		for (let index = 1; index < values.length; index++) gaps.push(Math.abs(values[index] - values[index - 1]));
		const gap = gaps.length > 0 ? Math.min(...gaps) : 1;
		// The first and the last value keep half a step of room.
		const from = Math.min(...values) - gap / 2;
		const to = Math.max(...values) + gap / 2;
		for (const value of values) positions.push(plot.left + ((value - from) / (to - from)) * (plot.right - plot.left));
	} else {
		for (let index = 0; index < data.xLabels.length; index++) positions.push(plot.left + band * (index + 0.5));
	}

	// The width of the group of bars of one x value. The group leaves room
	// between the groups, and every series of the group gets the same width.
	let groupWidth = band * 0.7;
	if (numericX && positions.length > 1) {
		const gaps: number[] = [];
		for (let index = 1; index < positions.length; index++) gaps.push(Math.abs(positions[index] - positions[index - 1]));
		groupWidth = Math.min(...gaps) * 0.7;
	}
	// A bar keeps a sane width, also when the chart holds few x values.
	const barWidth = Math.min(groupWidth / series.length, 64);
	const barGroupWidth = barWidth * series.length;
	const baseline = yOf(0);
	const xOf = (index: number): number => positions[index];

	// The grid, the ticks and the titles of the axes.
	const frame: RootContent[] = [];
	const rule = (className: string, x1: number, y1: number, x2: number, y2: number): RootContent =>
		context.element("line", { className: [className], x1, y1, x2, y2, fill: "none", stroke: "currentColor" });
	const label = (className: string, x: number, y: number, anchor: string, value: string): RootContent =>
		context.element("text", { className: [className], x, y, textAnchor: anchor, dominantBaseline: "middle", fill: "currentColor" }, [
			{ type: "text", value }
		]);

	// The ticks of the y axis. The count comes from the range, so the loop does
	// not drift with the step.
	const tickCount = Math.max(1, Math.round((yMax - yMin) / step));
	for (let index = 0; index <= tickCount; index++) {
		const value = yMin + index * step;
		const y = yOf(value);
		frame.push(rule("iw-chart-grid", plot.left, y, plot.right, y));
		frame.push(label("iw-chart-tick", plot.left - 10, y, "end", formatNumber(value)));
	}
	frame.push(rule("iw-chart-axis", plot.left, plot.bottom, plot.right, plot.bottom));
	data.xLabels.forEach((name, index) => {
		frame.push(label("iw-chart-tick", xOf(index), plot.bottom + 20, "middle", name));
	});
	// The unit stands above the y axis. The x axis title stands at the end of
	// the x axis.
	if (unit) frame.push(label("iw-chart-axis-title", plot.left - 10, plot.top - 22, "end", unit));
	if (data.xTitle) frame.push(label("iw-chart-axis-title", plot.right, plot.bottom + 40, "end", data.xTitle));

	// The series of the chart. Every data point gets a group with a title, so
	// the browser shows the value when the reader hovers over the point.
	const seriesNodes = series.map((item, index) => {
		const color = `iw-chart-series-${(index % SERIES_COLORS) + 1}`;
		const points: RootContent[] = [];

		if (type === "line") {
			let path = "";
			let open = false;
			item.values.forEach((value, position) => {
				if (value === null) {
					open = false;
					return;
				}
				path += `${open ? "L" : "M"}${xOf(position).toFixed(2)} ${yOf(value).toFixed(2)} `;
				open = true;
			});
			if (path !== "") {
				const attributes = { className: ["iw-chart-line"], d: path.trim(), fill: "none", stroke: "currentColor" };
				points.push(context.element("path", attributes));
			}
		}

		item.values.forEach((value, position) => {
			if (value === null) return;
			const text = `${item.name}，${data.xLabels[position]}：${formatNumber(value)}${unit ? ` ${unit}` : ""}`;
			const shape =
				type === "bar"
					? context.element("rect", {
							className: ["iw-chart-bar"],
							x: xOf(position) - barGroupWidth / 2 + index * barWidth,
							y: Math.min(yOf(value), baseline),
							width: barWidth * 0.9,
							height: Math.abs(yOf(value) - baseline),
							fill: "currentColor"
						})
					: context.element("circle", { className: ["iw-chart-dot"], cx: xOf(position), cy: yOf(value), r: 3, fill: "currentColor" });
			const title = context.element("title", {}, [{ type: "text", value: text }]);
			points.push(context.element("g", { className: ["iw-chart-point"] }, [title, shape]));
		});

		return context.element("g", { className: ["iw-chart-series", color] }, points);
	});

	const svg = context.element(
		"svg",
		{
			className: ["iw-chart-svg"],
			dataChart: true,
			viewBox: `0 0 ${VIEW_WIDTH} ${VIEW_HEIGHT}`,
			width: "100%",
			height: "auto",
			role: "img",
			ariaLabel: title || series.map(item => item.name).join("、") || "图表"
		},
		[...frame, ...seriesNodes]
	);

	// The legend names the series. It shows up together with the title.
	const legend = title
		? context.element(
				"div",
				{ className: ["iw-chart-legend"] },
				series.map((item, index) =>
					context.element("span", { className: ["iw-chart-legend-item"] }, [
						context.element("span", { className: ["iw-chart-swatch", `iw-chart-series-${(index % SERIES_COLORS) + 1}`] }),
						{ type: "text", value: item.name }
					])
				)
			)
		: null;

	const children: RootContent[] = [];
	if (title) children.push(context.element("figcaption", { className: ["iw-chart-caption"] }, [{ type: "text", value: title }]));
	children.push(svg);
	if (legend) children.push(legend);
	children.push(...node.children);

	node.data = {
		...node.data,
		hName: "figure",
		hProperties: { className: ["iw-widget", "iw-chart"] }
	};
	node.children = children;

	return true;
};

export default renderChart;
