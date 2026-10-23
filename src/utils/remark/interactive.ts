import type { Plugin } from "unified";
import type { Root, RootContent } from "mdast";
import type { Properties } from "hast";
import type { Data } from "unist";
import { SKIP, visit } from "unist-util-visit";

import { escapeHtml } from "./escape";
import { renderAnnotate } from "./modules/annotate";
import { renderChart } from "./modules/chart";
import { renderCompare } from "./modules/compare";
import { renderQ, renderQuiz } from "./modules/quiz";
import { renderSlider } from "./modules/slider";
import { renderSortable } from "./modules/sortable";
import { renderStep } from "./modules/step";
import { renderSteps } from "./modules/steps";

// The markdown pipeline reads the tag name and the attributes from the data
// of a node. The `hProperties` field is declared by `src/utils/remark/attr.ts`.
declare module "unist" {
	interface Data {
		hName?: string;
	}
}

/**
 * A container directive as mdast produces it.
 *
 * The type comes from `mdast-util-directive`, which is not a direct
 * dependency, so the shape is declared here.
 */
export interface ContainerDirectiveNode {
	type: "containerDirective";
	name: string;
	attributes?: Record<string, string | null | undefined> | null;
	data?: Data;
	children: RootContent[];
}

/**
 * The tools a module uses to build its output.
 *
 * A module never writes a full HTML string. It builds mdast nodes and sets
 * `data.hName` and `data.hProperties` on them. The markdown pipeline then
 * renders those nodes as the requested HTML elements, and the markdown inside
 * the widget keeps its normal rendering: KaTeX, tables, code highlighting and
 * image optimization all keep working.
 */
export interface InteractiveContext {
	/** Escape a value for use in HTML text or in an attribute value. */
	escapeHtml(value: string): string;
	/**
	 * Build a node that renders as the given HTML element.
	 *
	 * @param tag - The HTML tag name, for example `div` or `input`.
	 * @param properties - The attributes. Use `className` with an array for classes.
	 * @param children - The child nodes.
	 */
	element(tag: string, properties?: Properties, children?: RootContent[]): RootContent;
	/** Build an id that is unique inside the current page. */
	uniqueId(prefix: string): string;
}

/**
 * Transform one directive node.
 *
 * The renderer mutates the node in place. It returns `true` when it took
 * ownership of the node, which stops the dispatcher from walking into the
 * children.
 */
export type ModuleRenderer = (node: ContainerDirectiveNode, context: InteractiveContext) => boolean;

const renderers: Record<string, ModuleRenderer> = {
	annotate: renderAnnotate,
	chart: renderChart,
	compare: renderCompare,
	q: renderQ,
	quiz: renderQuiz,
	slider: renderSlider,
	sortable: renderSortable,
	step: renderStep,
	steps: renderSteps
};

/**
 * Build the tool set for one page.
 *
 * The counter is per page, so the ids stay unique inside a page and stay
 * short. Two widgets of the same kind on one page get different numbers.
 */
const createContext = (): InteractiveContext => {
	let counter = 0;

	return {
		escapeHtml,
		element: (tag: string, properties: Properties = {}, children: RootContent[] = []): RootContent =>
			({
				type: "paragraph",
				data: { hName: tag, hProperties: properties },
				children
			}) as unknown as RootContent,
		uniqueId: (prefix: string): string => {
			counter += 1;
			return `${prefix}-${counter}`;
		}
	};
};

/**
 * Remark plugin that turns the container directives of the interactive
 * modules into HTML widgets.
 *
 * The plugin runs after `remark-directive`, which creates the
 * `containerDirective` nodes, and after the reading time plugin, so the word
 * count still sees the markdown source of the widgets.
 */
const remarkInteractive: Plugin<[], Root> = () => {
	const context = createContext();

	return (tree: Root) => {
		visit(tree, "containerDirective", (node: ContainerDirectiveNode) => {
			const renderer = renderers[node.name];
			if (!renderer) return;
			if (renderer(node, context)) return SKIP;
		});
	};
};

export default remarkInteractive;
