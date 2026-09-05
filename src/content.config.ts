import { glob } from "astro/loaders";
import { defineCollection, z } from "astro:content";

/**
 * Preserve the full relative path as entry id (e.g. "zh-cn/2025总结/index").
 * Astro 5's default generateId slugifies each path segment and occasionally
 * collapses the locale prefix, breaking multi-locale routing. Override it to
 * keep the locale prefix intact so that `entry.id.split("/")[0]` always returns
 * the language code in multi-locale mode.
 */
function entryId({ entry }: { entry: string }) {
	return entry.replace(/\.[^.]+$/, "").replace(/\/index$/, "");
}

/**
 * Note collection configuration
 * Represents main blog articles with comprehensive metadata
 */
const note = defineCollection({
	// Load all markdown files from note directory
	loader: glob({
		pattern: ["**/*.md", "!**/_*.md", "!**/_*/*.md"],
		base: "./src/content/note",
		generateId: entryId
	}),
	schema: z.object({
		title: z.string(), // Post title (required)
		timestamp: z.date(), // Publication date (required)
		series: z.string().optional(), // Series name for grouped posts
		tags: z.array(z.string()).optional(), // Array of topic tags
		description: z.string().optional(), // Post description/excerpt
		sensitive: z.boolean().default(false), // Marks content as sensitive
		toc: z.boolean().default(true), // Whether to show table of contents
		top: z.number().int().nonnegative().default(0), // Top priority for sorting (higher is more important)
		draft: z.boolean().default(false) // Draft status (excludes from public listing)
	})
});

/**
 * Jotting collection configuration
 * Represents shorter posts, quick thoughts, or micro-blog entries
 */
const jotting = defineCollection({
	// Load all markdown files except those starting with underscore
	loader: glob({
		pattern: ["**/*.md", "!**/_*.md", "!**/_*/*.md"],
		base: "./src/content/jotting",
		generateId: entryId
	}),
	schema: z.object({
		title: z.string(), // Jotting title (required)
		timestamp: z.date(), // Publication date (required)
		tags: z.array(z.string()).optional(), // Array of topic tags
		description: z.string().optional(), // Brief description
		sensitive: z.boolean().default(false), // Marks content as sensitive
		top: z.number().int().nonnegative().default(0), // Top priority for sorting (higher is more important)
		draft: z.boolean().default(false) // Draft status
	})
});

/**
 * Preface collection configuration
 * Represents introductory content, site announcements, or special pages
 */
const preface = defineCollection({
	// Load all markdown files
	loader: glob({ pattern: "**/*.md", base: "./src/content/preface", generateId: entryId }),
	schema: z.object({
		timestamp: z.date() // Creation timestamp
	})
});

/**
 * Information collection configuration
 * Represents static content like about pages, policies, or site information
 */
const information = defineCollection({
	// Load both markdown and YAML files for mixed content types
	loader: glob({
		pattern: "**/*.{md,mdx,yaml}",
		base: "./src/content/information",
		generateId: entryId
	})
});

export const collections = { note, jotting, preface, information };
