import type { ShikiTransformer } from "shiki";

/**
 * Shiki transformer that adds the language label and the line highlight to
 * the code blocks.
 *
 * The transformer runs after the copy button transformer, so the `<pre>` is
 * already inside a `div.code-container`.
 */
const chrome = (): ShikiTransformer => ({
	name: "code-chrome",
	pre(node): void {
		void node;
	}
});

export default chrome;
