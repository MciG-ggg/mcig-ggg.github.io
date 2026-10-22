/**
 * Render one markdown file through the pipeline of the site and print the HTML.
 *
 * The script uses the markdown configuration of `astro.config.ts`, so the
 * output matches the built page. It runs in about a second, which makes it
 * the fast check for the interactive modules.
 *
 * Usage: `pnpm exec tsx scripts/test-interactive.ts <file.md>`
 */
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { createMarkdownProcessor, type AstroMarkdownProcessorOptions } from "@astrojs/markdown-remark";
import config from "../astro.config";

const target = process.argv[2];
if (!target) {
	console.error("usage: pnpm exec tsx scripts/test-interactive.ts <file.md>");
	process.exit(1);
}

const file = resolve(process.cwd(), target);
const source = await readFile(file, "utf8");

const processor = await createMarkdownProcessor(config.markdown as AstroMarkdownProcessorOptions);
const { code, metadata } = await processor.render(source);

console.log(code);
console.error(`headings: ${metadata.headings.map(heading => `${heading.depth}:${heading.slug}`).join(", ") || "none"}`);
