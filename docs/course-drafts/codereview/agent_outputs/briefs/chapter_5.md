第 5 章 工程化收尾：覆盖率、CI，以及 AI 之后剩下的问题（V2 13:42-15:15）

- 正式项目还需要什么：测试是有的，但没有 coverage，不知道测试写得够不够 [13:42]
- GitHub workflow 只有一个 publish（上传 wheel），没有测试、没有 lint、没有 format；程序出错压根不知道；想做成正经项目，CI 要做起来 [14:00]
- AI 之后小结：编程小细节上的问题越来越少了，缩进、变量名格式这类琐碎问题基本消失，代码看起来都挺靠谱 [14:30]
- 但从项目角度看，依然有大量可以 review 的问题：design、interface、CI [14:45]
- 这些东西 AI 能做吗。或许能做，但它肯定不是自动做的：不会说「写个这个项目」就把我说的那些全都做了 [14:55]
- 你脑子里得有这根弦：需要不断跟它沟通、不断跟它较劲，它在这方面可能有有价值的见解 [15:05]
- 有没有这个意识去跟它沟通，是作为程序员值得培养的能力 [15:15]


---

## 写作指令（writer agent 必读）

产出文件：`docs/course-drafts/codereview/sections/section_5.md`

正文从 `## 第 5 章 工程化收尾：覆盖率、CI，以及 AI 之后剩下的问题` 开始，写完整章节，不是摘要。写到最后一个知识点就停，不写章末小结、不写总结、不写过渡到下一章的话。

材料：
- 本章要点：`docs/course-drafts/codereview/agent_outputs/briefs/chapter_5.md`（上面这一节）
- 本章字幕切片（带时间戳）：`docs/course-drafts/codereview/slices/chapter_5.txt`
- 术语表：`docs/course-drafts/codereview/agent_outputs/glossary.md`（专有名词与英文写法一律照它来）
补充材料：CI 配置在 source/temparse_publish.yml，内容是 `on: release: types: [published]` 加三步 `actions/checkout` / `astral-sh/setup-uv` / `uv build` / `uv publish`。仓库根目录的 workflow 只有这一个发布流程。

本章可用的图（放在本章目录的 `images/` 下，正文里单独成段引用，alt 文本写图注，路径写 `images/xxx.png`）：
- `images/fig-repo-tree.png` — temparse 的 GitHub 仓库首页：.github/workflows 只有一个发布流程（视频 14:40）

硬性要求：
1. 溯源链接只用时间戳一种形状，链接文本写成 `[MM:SS]`，URL 用 `https://www.bilibili.com/video/BV1RfaX6sESp/?t=<秒数>`，秒数 = 分×60+秒。至少 3 处，放在知识点后面，不要单独成行。
2. 正文里不要出现这些词（本博客的 lint 会报错）：本节将介绍、接下来、值得注意的是、本章小结、要点回顾、总结、综上所述、总而言之、显然、易证、众所周知、不难看出、大家可以自己验证。
3. 不写 `##` 以外的一级标题，不写「小结 / 总结」类标题。
4. 讲师原话用引用块保留，每一行都以 `>` 开头。引用块里可以出现上面禁用的词。
5. 代码用围栏代码块并标语言。代码前一句说明这段代码在做什么，代码后不要重复叙述代码里已经能看得出的东西。
6. 章节开头直接进入问题，不写「大家好」「本节将介绍」「接下来我们来看」这类开场白。
7. 中文里夹 English 术语（`code review`、`interface`、`t-string`、`type check`、`CI`、`coverage`）。句子短，陈述句优先。不写「不是……而是……」这种转折。
8. 不留 `[cite]`、`TODO`、待补之类的占位符。不保留原始逐字稿。
9. 公式极少用；确需公式时用 `$...$`，且公式前先用一句中文说明它在表达什么。
