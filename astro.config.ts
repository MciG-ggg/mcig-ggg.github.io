// @ts-check
import { defineConfig, fontProviders } from "astro/config";
import yaml from "@rollup/plugin-yaml";
import mdx from "@astrojs/mdx";
import sitemap from "@astrojs/sitemap";
import svelte from "@astrojs/svelte";
import tailwindcss from "@tailwindcss/vite";
import swup from "@swup/astro";
import githubLight from "shiki/themes/github-light.mjs";
import darkPlus from "shiki/themes/dark-plus.mjs";

import GFM from "remark-gfm";
import ins from "remark-ins";
import mark from "remark-flexible-markers";
import CJK from "remark-cjk-friendly";
import CJKStrikethrough from "remark-cjk-friendly-gfm-strikethrough";
import math from "remark-math";
import gemoji from "remark-gemoji";
import footnote from "remark-footnotes-extra";
import { remarkExtendedTable as table, extendedTableHandlers as tableHandler } from "remark-extended-table";
import directive from "remark-directive";
import ruby from "remark-ruby-directive";
import directiveTextGuard from "./src/utils/remark/directive-text-guard";
import alerts from "remark-github-blockquote-alert";
import { rehypeHeadingIds as ids } from "@astrojs/markdown-remark";
import anchor from "rehype-autolink-headings";
import links from "rehype-external-links";
import katex from "rehype-katex";
import sectionize from "@hbsnow/rehype-sectionize";
import rehypeCardlink from "./src/utils/rehype-cardlink";

import spoiler from "./src/utils/remark/spoiler";
import attr from "./src/utils/remark/attr";
import abbr from "./src/utils/remark/abbr";
import wrapper from "./src/utils/remark/table-wrapper";
import copy from "./src/utils/code-copy";
import chrome from "./src/utils/code-chrome";
import reading from "./src/utils/remark/reading";
import figure from "./src/utils/remark/figure";
import cardlink from "./src/utils/remark/cardlink";
import interactive from "./src/utils/remark/interactive";

import siteConfig from "./site.config";
import { ZeoSevenFonts } from "./src/fonts/config";

// https://astro.build/config
export default defineConfig({
	site: "https://mcig-ggg.github.io/",
	trailingSlash: "never",
	// The code review tutorial was published as a directory page plus one
	// page per chapter, then merged into a single page. The chapter URLs are
	// already indexed, so they redirect to the merged page.
	redirects: {
		"/lecture/CodeReview教程_AI时代下的CodeReview/第 1 章 Code Review 在流程中的位置": "/lecture/CodeReview教程_AI时代下的CodeReview",
		"/lecture/CodeReview教程_AI时代下的CodeReview/第 2 章 审查清单与沟通心态": "/lecture/CodeReview教程_AI时代下的CodeReview",
		"/lecture/CodeReview教程_AI时代下的CodeReview/第 3 章 读一个模板解析库的代码": "/lecture/CodeReview教程_AI时代下的CodeReview",
		"/lecture/CodeReview教程_AI时代下的CodeReview/第 4 章 设计层的分歧：装饰器、interface 与一致性":
			"/lecture/CodeReview教程_AI时代下的CodeReview",
		"/lecture/CodeReview教程_AI时代下的CodeReview/第 5 章 工程化收尾：覆盖率、CI，以及 AI 之后剩下的问题":
			"/lecture/CodeReview教程_AI时代下的CodeReview",
		"/lecture/CodeReview教程_AI时代下的CodeReview/第 6 章 判断力：把工作和知识解耦": "/lecture/CodeReview教程_AI时代下的CodeReview",
		// The Auto-Drive-Intro note follows the 2026 courseware now. Five
		// chapter URLs from the old six-chapter layout are gone, so they
		// redirect to the chapter that took over their content.
		"/lecture/Auto-Drive-Intro/第 2 章 无人车基本框架": "/lecture/Auto-Drive-Intro/第 1 章 智能驾驶概述",
		"/lecture/Auto-Drive-Intro/第 3 章 视觉感知": "/lecture/Auto-Drive-Intro/第 2 章 环境感知",
		"/lecture/Auto-Drive-Intro/第 4 章 雷达感知": "/lecture/Auto-Drive-Intro/第 2 章 环境感知",
		"/lecture/Auto-Drive-Intro/第 5 章 路径规划": "/lecture/Auto-Drive-Intro",
		"/lecture/Auto-Drive-Intro/第 6 章 地图与定位": "/lecture/Auto-Drive-Intro/第 3 章 高精度地图",
		// The note now stops at chapter 5. The four pages after it are gone,
		// so their URLs redirect to the directory page.
		"/lecture/Auto-Drive-Intro/第 7 章 控制理论": "/lecture/Auto-Drive-Intro",
		"/lecture/Auto-Drive-Intro/第 8 章 强化学习": "/lecture/Auto-Drive-Intro",
		"/lecture/Auto-Drive-Intro/附录 A 路径规划": "/lecture/Auto-Drive-Intro",
		"/lecture/Auto-Drive-Intro/课后实践：APOLLO 平台大作业": "/lecture/Auto-Drive-Intro"
	},
	i18n: {
		...siteConfig.i18n,
		routing: {
			redirectToDefaultLocale: false,
			prefixDefaultLocale: false
		}
	},
	markdown: {
		remarkPlugins: [
			[GFM, { singleTilde: false }],
			ins,
			mark,
			spoiler,
			attr,
			CJK,
			[CJKStrikethrough, { singleTilde: false }],
			math,
			gemoji,
			footnote,
			abbr,
			[table, { colspanWithEmpty: true }],
			wrapper,
			cardlink,
			directive,
			ruby,
			directiveTextGuard,
			[alerts, { legacyTitle: true }],
			reading,
			// The interactive modules run last, so the reading time still
			// counts the markdown source of the widgets.
			interactive
		],
		remarkRehype: {
			footnoteLabel: null,
			footnoteLabelTagName: "p",
			footnoteLabelProperties: {
				className: ["hidden"]
			},
			handlers: {
				...tableHandler
			}
		},
		rehypePlugins: [
			ids,
			[anchor, { behavior: "wrap" }],
			[links, { target: "_blank", rel: ["nofollow", "noopener", "noreferrer"] }],
			katex,
			rehypeCardlink,
			figure,
			sectionize
		],
		smartypants: false,
		shikiConfig: {
			themes: {
				light: {
					...githubLight,
					colorReplacements: {
						"#fff": "var(--block-color)"
					}
				},
				dark: {
					...darkPlus,
					colorReplacements: {
						"#1e1e1e": "var(--block-color)"
					}
				}
			},
			// Emit `light-dark()` instead of one hard-coded theme plus `--shiki-dark`.
			// The page sets `color-scheme`, so the code block follows the theme.
			defaultColor: "light-dark()",
			transformers: [
				copy({
					duration: 1500
				}),
				chrome()
			]
		}
	},
	vite: {
		// @ts-expect-error
		plugins: [yaml(), tailwindcss()]
	},
	integrations: [
		svelte(),
		mdx(),
		sitemap(),
		swup({
			globalInstance: true,
			preload: false,
			smoothScrolling: false,
			progress: true
		})
	],
	experimental: {
		fonts: [
			{
				name: "Noto Serif",
				provider: fontProviders.google(),
				weights: [400, 700],
				fallbacks: ["serif"],
				cssVariable: "--font-noto-serif"
			},
			{
				name: "Noto Serif SC",
				provider: fontProviders.google(),
				weights: [400, 700],
				fallbacks: ["serif"],
				cssVariable: "--font-noto-serif-sc"
			},
			{
				name: "Noto Serif JP",
				provider: fontProviders.google(),
				weights: [400, 700],
				fallbacks: ["serif"],
				cssVariable: "--font-noto-serif-jp"
			},
			{
				name: "Playwrite MX",
				provider: fontProviders.google(),
				weights: [100],
				display: "block",
				fallbacks: ["serif"],
				cssVariable: "--font-playwrite-mx"
			},
			{
				name: "Maple Mono NF CN",
				provider: ZeoSevenFonts(),
				fallbacks: ["monospace"],
				cssVariable: "--font-maple-mono-nf-cn"
			},
			{
				name: "The Peak Font Plus",
				provider: ZeoSevenFonts(),
				fallbacks: ["serif"],
				cssVariable: "--font-the-peak-font-plus"
			}
		]
	}
});
