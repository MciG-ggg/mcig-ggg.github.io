import siteConfig from "./src/utils/config";

const config = siteConfig({
	title: "MciG",
	prologue: "Undergraduate learning embodied intelligence, robotics, and generative models.",
	author: {
		name: "MciG",
		email: "mcigggg399026@gmail.com",
		link: "https://mcig-ggg.github.io"
	},
	description: "MciG-ggg 的个人主页：本科生，关注具身智能、机器人、VLA 与生成式模型。",
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
