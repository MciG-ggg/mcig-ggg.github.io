# 讲义体系改造方案

> 范围：`drive-render-pdf` / `review-notes-pdf` / `bilibili-render-pdf` / `youtube-render-pdf` 四个 skill，
> 以及 `mcig-blog` 的可视化与交互能力。
>
> 决策来源：11 轮 grilling + 3 条 workflow（29 agent，9.2M token）。

---

## 1. 根决策

**四个 skill 从「产出 LaTeX → PDF」改为「产出 blog 原生 Markdown，落进 mcig-blog 的 `lecture` collection」。**
PDF 不再由 skill 直接生成，而是从 blog 页面打印导出。

这条决策决定了后面全部细节。

### 为什么

| 事实 | 来源 |
|---|---|
| 四个 skill 现在全是 LaTeX 单交付，模板高度重复 | 审计实测 |
| 视频模板字节相同（md5 `7df4c42e…`），drive 只多 3 行，review 是独立分叉 | 审计实测 |
| 所有**增加**长度的规则都有脚本检查，所有**减少**长度的规则一条都不查 | 审计实测 |
| 真实交付里 5 个「本章小结」占 1814/22650 CJK 字符 = **8.0%**，纯复述 | 审计实测 |
| blog 已装 remark-math + rehype-katex + shiki + mermaid + medium-zoom + @astrojs/mdx | 调研实测 |
| Chrome headless 打印的 PDF **无 outline 书签树**，CJK 被嵌成 Type3 字体 | 审计实测 |
| LaTeX 走 hyperref 有 3 项 outline，字体嵌入正常 | 审计实测 |

最后一条是唯一对「blog → PDF」不利的证据，但可以用脚本补（见 §6）。

---

## 2. 内容决策

### 2.1 读者与定位

自学者深读。假设读者是三个月后回来看这篇的人。

### 2.2 砍哪些废话

只砍两类，其余保留：

| 砍 | 不砍 |
|---|---|
| 开场铺垫与过渡句（"本节将介绍…"） | 原文引用块 |
| 章末总结复述（`\subsection{本章小结}`、Entry recap、Exit handoff） | 公式前的直觉解释 |

理由：这两类是纯结构复述，实测占 8.0%。原文引用、公式直觉铺垫是信息本身，不是废话。

### 2.3 章节结构

**按知识结构重组，不照视频章节 1:1。** 视频章节按讲课顺序，学习顺序未必最优。

### 2.4 篇幅

**只去小结，不设字数上限。** 字数上限会逼模型砍公式或砍图，与「加可视化」直接冲突。

### 2.5 溯源元数据（全保留）

- 行内可点击时间戳 `[12:34](https://…&t=754s)` → 点开跳到视频那一秒
- 文首 `cardlink` 块：原视频链接卡片（带封面）
- `series` 归组
- 原文引用块：关键段落保留讲师原话

### 2.6 原始转写

**不保留全文。** 正文精简，不附逐字稿。

---

## 3. 输出契约

四个 skill 共用一份写作契约，各自 SKILL.md 只写平台特有的部分。

### 3.1 落盘位置

```
src/content/lecture/zh-cn/<可读标题>/
├── index.md          # blog 原生 Markdown
└── images/           # 视频关键帧 PNG
```

新 collection 名 `lecture`，与 `note` / `jotting` 平行，样式大体相同。

### 3.2 frontmatter

```yaml
---
title: <标题>
timestamp: 2026-10-21 14:30:00+08:00
series: <课程或系列名>
tags: [cuda, inference]
description: <一句话摘要>
source_url: https://www.youtube.com/watch?v=...
toc: true
draft: true          # 生成后先 draft，确认再改 false
---
```

`draft` 默认 `true`——生成完不自动公开。

### 3.3 使用的 blog 原生语法

| 用途 | 语法 |
|---|---|
| 公式 | `$...$` / `$$...$$`（remark-math + rehype-katex） |
| 代码 | 标准围栏，Shiki 高亮 + 自带复制按钮 |
| 图 | `![caption](images/x.png)` 单独成段 → 自动转 `<figure>` + `<figcaption>` |
| 视频卡片 | ` ```cardlink ` YAML 块 |
| 告警 | `> [!NOTE]` / `[!TIP]` / `[!WARNING]` / `[!IMPORTANT]` |
| 表格 | 标准 markdown，自动包 `.table-wrapper` |
| 强调 | `==text==` |
| 术语 | 文末 `*[ABBR]: Definition` |

### 3.4 图的要求

| 类型 | 来源 | 落地 |
|---|---|---|
| 视频关键帧 | 抽帧 + 目视筛选 | `images/*.png`，单独成段引用 |
| 结构/流程/概念图 | figure agent 手写 | **内联 SVG**，用 `currentColor` 跟随主题 |

**每节一图，触发式**：只在图能替代三句以上正文、或是流程/结构/时间线时才画。不设图数上限。

### 3.5 交互模块

作者可用 `:::` 容器指令调用（见 §5）。

### 3.6 读者可见的来源标记

文首标一行来源（原视频链接 + 「本文由视频整理」）。

---

## 4. Agent 拓扑

**保留并行分章 writer。** 深度最高，代价是 master 要写过渡语去粘——但过渡语正是要砍的东西，所以 §2.2 的规则同时消掉了这个代价。

```
[0] master      取源、建目录、切片、集成、commit
[1] outline     知识结构大纲 + 术语表 + 章节→切片映射
[2] figure      抽帧筛选 + 内联 SVG 生成 → images/ + figures_manifest
[3] writers     每个 writer 写一个 index.md 分片（按知识结构，不按视频章节）
[4] consistency 术语一致性、重复定义、图引用完整性、溯源完整性
[5] master      集成 → src/content/lecture/zh-cn/<标题>/index.md → git commit
```

**writer 直读 blog 现有笔记**（`src/content/note/zh-cn/`）学语气和图用法。

**agent 间用磁盘文件传递**，不贴长文本。

### 4.1 图 agent 的产出

`figures_manifest.json`：

```json
{
  "figures": [
    {"id": "f1", "type": "frame", "path": "images/frame-1234.png",
     "caption": "…", "interval": "12:34-13:02", "section": "ch3"},
    {"id": "f2", "type": "svg", "inline": "<svg…>", "section": "ch3"}
  ]
}
```

---

## 5. blog 侧改造

### 5.1 视觉 token（3h）

**只换 token，不改布局。** 但加动作条和 chip 行。

`src/styles/global.css`：
- 三份重复调色板（`:root` / `@media prefers-color-scheme:dark` / `[data-theme=dark]`）合并成一份，用 `light-dark()`
- 补 `[data-theme="light"]` —— **修现有 bug**：系统偏好深色时用户选浅色压不住 media query 里的 `:root`
- ChatGPT 色阶：surface `#fff`/`#f9f9f9`（light）、`#212121`/`#303030`（dark）；text `#0d0d0d`/`#fff`；muted `#5d5d5d`/`#cdcdcd`；faint `#8f8f8f`/`#afafaf`；border `#0000001a`/`#ffffff1a`
- alpha 阶梯 05/08/10/12/16/20/25
- radius 阶梯 2/4/6/8/10/12/16/20/24px
- 4px 间距单位
- 新增 `--accent-color: #3566f0` + `--accent-soft`；全局 `:focus-visible` 焦点环

正文保留现在的 serif 文学感，UI chrome 与内联控件改 sans。

同步改：`astro.config.ts`（shiki `defaultColor: "light-dark()"` + dark 侧 `colorReplacements`）、`src/layouts/header/ThemeSwitcher.svelte`（派发 `theme:change` 事件 + 不再无条件覆盖用户选择）、`public/feed.css`（独立复制了一份调色板）、`src/graph/content.ts`（OG 图写死 `#fffffd`）。

### 5.2 文章动作条 + chip 行（2.5h）

`src/components/ArticleActions.astro`（新建，纯 SSR + 内联 onclick）：复制链接 / 复制为 Markdown / 分享。
标题下元信息改成 ChatGPT 风格的 chip 行（日期/系列/标签/字数，12px muted，hairline 描边）。

### 5.3 mermaid 修复（2h）

**修 bug，不是加模块。** 现状三个洞：

1. 判定逻辑是「按代码首词猜」的兜底，只覆盖固定 14 种关键字，`xychart`/`sankey`/`treemap`/`timeline` 永远不渲染。**且会误判**——以 `graph = torch.cuda.CUDAGraph()` 开头的 Python 块被当成图表。
2. 主题写死 `dark`——浅色页上是深色块，深色页上也是深色。可见对比度缺陷。
3. 整包进每个页面的 Base chunk（490KB / 140KB gzip），首页也加载。

改法：
- 判定改为 `pre[data-language="mermaid"]`（Shiki 已写该属性，dist 产物可见）
- 主题跟 `documentElement.dataset.theme`，监听 `theme:change` 重绘
- SVG 注入 `pre` 内部，**不动容器**（现在 `outerHTML` 整块替换把复制按钮一起删了）
- `await import("mermaid")` 动态加载 → Base chunk 从 140KB gzip 降到 3.5KB

33 篇笔记只有 2 篇用 mermaid，其余页面直接省 490KB。

### 5.4 交互模块库（~19h）

全部走 `remark-directive` 容器指令（`:::`），构建期 remark 插件解析，输出 raw HTML（照 `src/utils/remark/cardlink.ts` 的形状，它自带 `escapeHtml` 先例）。

| 模块 | 语法 | 实现 | 工时 |
|---|---|---|---|
| **chart** | `:::chart{type="line" title="…"}` + markdown 表格 | 构建期生成内联 SVG + `<title>` 悬停；class 取色走 CSS 变量；只做 line + bar | 4.5h |
| **slider** | `:::slider{label min max step value unit}` + 内容 | 运行时写 `--v` / `--value` 两个 CSS 变量；document 级事件委托（swup 换页免重绑） | 2h |
| **compare** | `:::compare{leftLabel rightLabel}` + 两图 | range 承担分隔线，键盘可达 | 2.5h |
| **steps / tabs** | `::::steps{title}` + `:::step{title}` | radio + 纯 CSS 兄弟选择器显隐，零 JS | 2.5h |
| **annotate** | `:::annotate{title}` + 图 + `:pin{x y label}` | CSS hover/focus 展开气泡；百分比坐标 | 3h |
| **可排序表格** | 自动（数值列） | 客户端给表头加按钮，排序 DOM 行 | 3h |
| **代码块 chrome** | 自动 | 语言标签 + 终端外框；ChatGPT 签名元素 | 1.5h |

**联动实现**：`src/utils/remark/interactive.ts`（指令分派）+ `src/utils/interactive.ts`（客户端）+ `src/styles/interactive.css`。

**行高亮**（围栏 meta，如 ` ```python {1,3} `）：20 行加进现有 shiki transformer——**不需要**单独 rehype 插件（实测 `this.options.meta.__raw` 在 pre 与 line hook 都拿得到）。

### 5.5 三个必须处理的既有约束

| 约束 | 处理 |
|---|---|
| `global.css` 的 `svg { fill: currentColor }` 会给图表几何填上文字色 | 图表内显式 `fill:none` 或 class 覆盖 |
| `medium-zoom` 绑 `.markdown img:not([data-nozoom])`，控件内图片仍会点开大图盖住交互 | 控件内图加 `data-nozoom`（逃生舱仓库已有） |
| 控件内 `###` 仍被收集进 headings，TOC 会多出指向隐藏面板的条目 | TOC 过滤，或禁控件内 h2/h3 |

### 5.6 MDX

只给需要交互的篇用 `.mdx`。实测 `.mdx` 里 `client:load` 的 Svelte 组件**可行**：能构建、能 SSR 出 island、能 hydration、swup 软导航后仍可用，Shiki/表格/分区/阅读字数都不丢。

**注意**：`attr.ts` 的 `{...}` 属性语法与 MDX **互斥**——`## H {#id}` 直接让构建失败（acorn 解析 JSX 报错）。MDX 篇里不能用属性语法。

### 5.7 `@media print`

一条 print 块（约 30 行 CSS，**不做 JS 状态机**）：
- 展开所有折叠面板
- 隐藏 range / 排序按钮 / 分隔线
- 图表 `break-inside: avoid`
- 滑条复位到作者设定的初值（读者自己拖出来的末态不具复现性）—— 这一条需要一小段 JS，排在 CSS 之后

### 5.8 不做

- **笔记关系图**：`graph.png.ts` 是 Open Graph 社交分享图（satori + sharp），不是关系图；数据极稀疏（35 篇 / 14 条内链 / 23 个 tag），逐篇连线图不成立
- **侧栏 artifact**：OpenAI 已于 2026-05-28 退役 Canvas，2026-10-07 的 Intelligent UI 收敛到「正文内嵌 artifact」
- **滚动驱动动画**：延后

---

## 6. PDF 导出

**手动跑脚本。** skill 不自动 build（每次重 build 整个博客太慢）。

```
pnpm build
node scripts/export-pdf.mjs <note-id>
```

Chrome headless `--print-to-pdf`。

### 两个已知降级

| 问题 | 处理 |
|---|---|
| Chrome 出的 PDF **无 outline 书签树**（xelatex+hyperref 有 3 项） | 接受，或脚本用 pypdf 补 outline。**未实测**哪个更省事 |
| CJK 被嵌成 Type3 字体（60KB vs LaTeX 的 4.9KB） | 接受，或在 print CSS 里指定系统字体栈。**未实测** |

这两条是「从 blog 页面打印」相对 LaTeX 的唯一劣势。**未实测**——需要跑一次真实导出才知道代价。

---

## 7. 旧资产处置

| 对象 | 处置 |
|---|---|
| 四个 skill 的 `.tex` 模板 | **删掉**（git 历史里还有） |
| `review-notes-pdf/scripts/` 9 个脚本 | **删掉**（`lint_handout.py` / `build_handout.py` 是 LaTeX 结构硬耦合的） |
| `review-notes-pdf/references/` 6 个 md | 重写为 blog 版写作契约 |
| 旧 PDF 产物 | **保留在磁盘**，不再生成新的 |
| 存量讲义（`/Users/mcig/Documents/pku-ai-infra/Video-Course/` 14 篇 + `review-notes-20260922/` 1 篇） | **不迁移** |

---

## 8. 要修的既有缺陷

| 缺陷 | 位置 | 修法 |
|---|---|---|
| **Google API key 硬编码** | `drive-render-pdf/SKILL.md:106` | 移到环境变量。**这是公开 web key，不是私密凭据，但仍该移** |
| `soffice` 是指向不存在路径的坏 shim | `/opt/homebrew/bin/soffice` | 删 shim 或重装 LibreOffice。当前 skills 不用它，但会误导后续 agent |
| mermaid 主题写死 dark + 关键词误判 + 整包常驻 | `src/layouts/Base.astro` | 见 §5.3 |
| `[data-theme="light"]` 缺失 | `src/styles/global.css` | 见 §5.1 |
| TOC `addEventListener` 每次换页累积 | `src/components/TOC.astro` | 注册一次 |
| `Position.astro` scroll 监听泄漏 | `src/components/Position.astro` | 换页先 remove 再 add |
| `Note.svelte` / `Jotting.svelte` 内层 `<main>` 嵌套 | 两个组件 | 内层改 `<div>`（swup `containers: ['main']` 查询有歧义） |

---

## 9. 旧 LaTeX 侧的两个真 bug（如果留着参考要修）

| bug | 位置 | 修法 |
|---|---|---|
| tcolorbox 标题宏未加花括号，标题含 ASCII 逗号会报错并截断 | 视频模板 `title=#1` | 改 `title={#1}` |
| review 模板硬依赖本机 `Noto Serif CJK SC`，缺失则整份编译失败 | `review-handout-template.tex:23` | 加 fallback |

---

## 10. 工作量

| 阶段 | 内容 | 工时 |
|---|---|---|
| A | blog token + 两个 bug + 动作条 + chip 行 | 5.5h |
| B | mermaid 修复 | 2h |
| C | 交互模块库（6 个模块） | 19h |
| D | `lecture` collection + 列表页 + 路由 | 3h |
| E | 四个 skill 重写（写作契约 + SKILL.md） | 4h |
| F | PDF 导出脚本 | 2h |
| | **合计** | **约 35.5h** |

---

## 11. 未实测 / 待定

- **PDF 导出的两个降级**（无 outline、CJK Type3）——需要跑一次真实导出才知道实际代价
- **Chrome headless 打印时 mermaid 是否渲染完成**——mermaid 是客户端异步渲染，无头快照可能在渲染完成前取样。兜底：把 mermaid 源码放图下方的 `<details>` 里
- **`chart` / `slider` / `compare` / `annotate` 的真实作者成本**——调研认为 slider 和 compare 在现有 33 篇里零用例，但决定做，所以要在第一篇真实文章上验证作者成本可接受
- **双语翻倍成本**——同一篇内容在 `zh-cn/` 和 `en/` 各有一份，需逐图标注的模块（annotate、compare）作者要写两遍
- **reading-time 失真**——`reading.ts` 用 `mdast-util-to-string` 统计，chart 把表格换成 SVG 后字数会掉

---

## 12. 执行顺序

1. **A** blog token + bug 修复（纯赚，互不耦合）
2. **B** mermaid 修复（ROI 最高的一步）
3. **D** `lecture` collection + 列表页
4. **E** 四个 skill 重写
5. **C** 交互模块库（chart → steps/tabs → 代码块 chrome → slider → compare → annotate → 排序表格）
6. **F** PDF 导出脚本
7. 拿一篇真实视频跑通全链路验证
