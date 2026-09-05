import siteConfig from "./src/utils/config";

const config = siteConfig({
	title: "MciG",
	prologue: "If you need a website\nthat loads fast and has great SEO, then Astro is for you.",
	author: {
		name: "MciG",
		email: "mcigggg399026@gmail.com",
		link: "https://mcig-ggg.github.io"
	},
	description: "A modern Astro theme focused on content creation.",
	copyright: {
		type: "CC BY-NC-ND 4.0",
		year: "2025"
	},
	i18n: {
		locales: ["zh-cn"],
		defaultLocale: "zh-cn"
	},
	feed: {
		section: "*",
		limit: 20
	},
	latest: "*"
});

export const monolocale = Number(config.i18n.locales.length) === 1;

export default config;
