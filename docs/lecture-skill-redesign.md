# 讲义体系改造：as-built 记录

> 范围：`drive-render-pdf` / `review-notes-pdf` / `bilibili-render-pdf` / `youtube-render-pdf` 四个 skill，
> 以及 `mcig-blog` 的可视化与交互能力。
>
> 决策来源：11 轮 grilling + 3 条 workflow（29 agent，9.2M token）。
>
> 本文记录的是**已经建成的东西**，不是计划。每节标状态：**已完成** / **部分完成** / **未开始**，
> 后面写清还差什么。所有数字都带测法，测法写不出来的地方就写「未实测」。

本文覆盖的提交区间：`8326366`（feat(ui): ChatGPT-style tokens, article actions, metadata chips）→ `335fd28`（note: rebuild the RL-Kernel note from the lecture slides），主线 21 个提交。

---

## 1. 根决策

**状态：已完成。**

四个 skill 从「产出 LaTeX → PDF」改为「产出 blog 原生 Markdown，落进 `mcig-blog` 的 `lecture` collection」。PDF 不再由 skill 直接生成，而是从 blog 页面打印。

**交付物只有 blog 原生 Markdown。** 没有 skill 产出 HTML，也没有 skill 产出 `.tex` 或 PDF。

证据：

| 事实 | 来源 |
|---|---|
| 共享契约写明「LaTeX 与 PDF 不是交付物」 | `_shared/blog-note-contract.md:12-13` |
| 视频 skill 写明「不产出 `.tex`，不产出 PDF。PDF 由人需要时从 blog 页面打印」 | `youtube-render-pdf/SKILL.md:20`、`bilibili-render-pdf/SKILL.md:20` |
| 课程 skill 写明「产出只有 blog 原生 Markdown」 | `review-notes-pdf/SKILL.md:20` |
| 会议 skill 写明「Do not deliver a `.tex` file or a PDF. A human prints the page」 | `drive-render-pdf/SKILL.md:480` |
| 四个 skill 目录里已无 `.tex` 模板 | `find /Users/mcig/.agents/skills -name "*.tex"` 只命中 `exam-paper-latex/assets/chinese-exam-template.tex`（另一个 skill，不在本次范围） |
| blog 侧仍装着 remark-math + rehype-katex + shiki + mermaid + medium-zoom + @astrojs/mdx | `astro.config.ts` import 段 |
| 存量讲义 PDF 仍在磁盘上（14 篇 + 1 篇），没有新生成 | `/Users/mcig/Documents/pku-ai-infra/Video-Course/*.pdf` 14 个；`/Users/mcig/review-notes-20260922/讲义.pdf` 8.2 MB |

**原计划里没落地的部分：** PDF 导出脚本（`scripts/export-pdf.mjs`）始终没写，`src/styles/` 里也没有任何 `@media print` 规则。所以「从页面打印」这条路目前只是契约里的说法，没有任何实现支撑。见 §11。

### 为什么

| 事实 | 来源 |
|---|---|
| 四个 skill 改造前全是 LaTeX 单交付 | 审计实测 |
| 视频模板字节相同（md5 `7df4c42e…`），drive 只多 3 行，review 是独立分叉 | 审计实测 |
| 所有**增加**长度的规则都有脚本检查，所有**减少**长度的规则一条都不查 | 审计实测 |
| 真实交付里 5 个「本章小结」占 1814/22650 CJK 字符 = **8.0%**，纯复述 | 审计实测 |
| Chrome headless 打印的 PDF **无 outline 书签树**，CJK 被嵌成 Type3 字体 | 审计实测 |
| LaTeX 走 hyperref 有 3 项 outline，字体嵌入正常 | 审计实测 |

最后一条是唯一对「blog → PDF」不利的证据。**至今没有实测过**：本轮没有导出过一次 PDF，两条降级都还是审计时的旧测量。

---

## 2. 内容决策

**状态：大部分已完成；章节组织规则被改写（见 §2.3）。**

### 2.1 读者与定位

**已完成。** 契约 frontmatter 写的是「目标读者是三个月后回来看这篇的人自己」，`pedagogical-flow.md` 展开成「读者脑子是脏的，不是空的」：他要的是推导链条完整、术语定义准确、坑被标出来，不是从头听一遍。

### 2.2 砍哪些废话

**已完成，而且从「散文规矩」变成「脚本门禁」。**

契约 §3 保留/删掉对照表落地成 `review-notes-pdf/scripts/lint_note.py`（309 行共享契约 + 309 行 lint）的确定性检查：frontmatter 八个字段、cardlink 块、禁用措辞、不许小结/总结标题、引用形状单一且至少 3 处、每张图有 alt、每章最多 3 个提示框、`$$` 配对、无 TODO 残留。

实测（`python3 lint_note.py --note <路径>`）：

| 笔记 | 结果 |
|---|---|
| `计算智能_进化算法与优化方法` | `lint_note: OK` |
| `vLLM小课堂（十八）RL-Kernel_大模型RL后训练的一致性实践` | `lint_note: OK` |
| `vLLM小课堂十八_RL-Kernel训推一致性`（已被 335fd28 合并掉的中间版本） | 1 处错误：第 285 行「接下来」 |

**还差：** 契约 §3 明写「不写公式速查表、符号表这类查表内容」，但门禁不查这个。两篇笔记里已经看不到跨章速查表和跨章符号表（见 §2.3），可是**第 4 章（DE）里还留着一个 `### 术语速查` 小节，17 条**。它挂在章内，不是跨章表，lint 也不管。要不要禁，属于契约没定死的地方。

### 2.3 章节组织

**状态：规则被改写，已完成。**

计划 §2.3 定的规矩是**一条**：「按知识结构重组，不照视频章节 1:1」。这条对视频成立，对课程材料不成立——复习的人要把「第 7 章」对到笔记和试卷上，课程章节顺序就是教学大纲。

最终契约 `_shared/blog-note-contract.md:74-113` 写的是**三条**，按源材料类型分：

| 源材料 | 章节切分 | 落在哪 |
|---|---|---|
| 课程材料（pptx/pdf 文件夹） | **一个课件文件一章**，章节号照抄文件里的编号，文件缺号笔记也缺号，不合并不拆不重排 | `blog-note-contract.md:77-92` |
| 视频讲课，带 slides | **按幻灯片切分**，幻灯片切换处是章节边界，每章写清覆盖哪几张 | `blog-note-contract.md:94-105` |
| 视频讲课，不带 slides | **按知识结构重组**，可以自由重排 | `blog-note-contract.md:107-113` |

**旧的固定七段式章节骨架没了。** 老 LaTeX 讲义模板 `review-notes-20260922/讲义-template.tex:163-170` 每章固定七段（前置补丁 / 动机 / 直觉与术语 / 完整不跳步的推导 / 阶梯例题 / 易错点 / 自测题）。新契约把它列为反模式：`review-notes-pdf/references/structure-guide.md:180`「❌ 固定 7 段式章节 → 凑出来的段落是填充物」。`pedagogical-flow.md` 保留了一个 7 步写作顺序，但标明「顺序可调，元素可缺」；`structure-guide.md` 的章节元素表也标「按需取用，不是固定顺序，不是固定条数」。

**落地证据（两篇笔记实测）：**

| 笔记 | 章节 | 小节 | 依据 |
|---|---|---|---|
| `计算智能_进化算法与优化方法` | 12 个 `##`（第 0–10 章、第 12 章，**没有第 11 章**） | 102 个 `###` + 74 个 `####` | 课件目录 `Nextcloud/School/Compute-Intelligence/课件/` 有 12 个 pptx（第 0–10、12），**没有第 11 个**。笔记的 12 章与 12 个文件一一对应，缺号留着缺号 |
| `vLLM小课堂（十八）RL-Kernel_大模型RL后训练的一致性实践` | 14 个 `##` | 53 个 `###` | 335fd28 提交说明：「The note follows the slide order now, with 14 chapters and 53 subsections」；旧版是 17 个平铺章，按视频内容写的，读者对不回 deck |

**章节索引**（契约要求超过 5 章就在开头放一章一行）：两篇都有。计算智能 12 行，RL-Kernel 14 行。

### 2.4 篇幅

**已完成。** 不设字数上限。契约没有长度条款，`pedagogical-flow.md` 只说「推导的每一步都留，图不能因为太长被砍」。实测两篇笔记 333,011 B 和 31,635 B。

### 2.5 溯源元数据

**状态：部分完成。**

| 要素 | 状态 | 证据 |
|---|---|---|
| 行内可点击时间戳 | 已完成 | RL-Kernel 笔记 115 处 `[MM:SS](url?t=秒)`；旧版只有 36 处 |
| 文首 cardlink 块 | 已完成 | 两篇笔记文首都有 ```cardlink 块；计算智能的 cardlink 在 69aaf35 重建时丢过，a7236ce 补回 |
| `series` 归组 | 已完成 | 两篇 frontmatter 都有 `series` |
| 原文引用块 | 已完成 | 计算智能 58 行 `>` 引用；RL-Kernel 2 行 |

**还差：** 契约 §4 规定文档引用标签固定为 `<文件名> · p.<页码>`。计算智能笔记里 **499 处** 页码链接，只有 **89 处**用这个形状，其余 **410 处**是裸 `[p. N]`。`note_links.py` 的 `DOC_LINK_RE` 只认带 `·` 的形状，所以 lint 和 `build_source_manifest.py` 只能看见那 89 处。

### 2.6 原始转写

**已完成。** 两篇笔记都不含逐字稿；契约 §10 检查清单里「没有原始逐字稿」。

---

## 3. 输出契约

**状态：已完成，除了图的两条。**

### 3.1 落盘位置

**已完成。** `src/content/lecture/zh-cn/<可读标题>/`，下面是 `index.md` 和 `images/`。实测两篇都在这个位置。

collection 定义在 `src/content.config.ts:43-62`（`base: "./src/content/lecture"`，`generateId` 保留完整相对路径），注册在 `collections` 里（`:111`）。`Section` 类型加了 `"lecture"`（`src/utils/config.ts:9`），导航加了入口（`Navigator.svelte:19`），中英文 i18n 都有 `home.lecture` / `navigation.lecture`（`src/i18n/{zh-cn,en}/index.yaml:17,29`）。

### 3.2 frontmatter

**已完成。** 契约 §2 定八个字段。实测两篇的 frontmatter 都是 title / timestamp / series / tags / description / source_url / toc / draft 八项。

`draft` 默认 `true`；实测两篇已发布笔记都是 `draft: false`（计算智能在 808db5d 发布，RL-Kernel 在 02731aa 发布）。

### 3.3 使用的 blog 原生语法

**已完成。** 公式 `$...$` / `$$...$$`（remark-math + rehype-katex）、代码围栏（shiki + 自带复制按钮）、图单独成段自动转 `<figure>`、视频卡片 ```cardlink、告警 `> [!NOTE]` 等、表格（`table-wrapper`）、`==text==`（mark）、文末 `*[ABBR]: Definition`（abbr）——插件都在 `astro.config.ts` 里挂着。

### 3.4 图的要求

**状态：部分完成。**

| 计划里的两类图 | 落地情况 |
|---|---|
| 视频关键帧 PNG | 做了，但落成 **JPG**：RL-Kernel 笔记 12 张 `fig_001.jpg`–`fig_012.jpg`，分辨率 1536×817，按 335fd28 的说明是从 1920×1080 源切下来的，不是旧版 480 像素采样 |
| 课件页截图 PNG | 做了：计算智能笔记 65 张 `fig_001.png`–`fig_065.png`，2000×1500 |

**还差：** 计划 §3.4 说「结构/流程/概念图 → 内联 SVG，用 `currentColor` 跟随主题」。三篇笔记里 `<svg` 出现 **0 次**，全用位图。`global.css:195` 预留了 `svg[data-chart] { fill: none; }`，但那是给将来的 chart 模块用的。

### 3.5 交互模块

**状态：未开始（见 §5.4、§11）。** 契约和四个 SKILL.md 都留着 `:::` 容器指令的口子，但 `src/content/` 里 `^:::` 命中 **0 次**。

### 3.6 读者可见的来源标记

**已完成。** 两篇笔记文首都有 cardlink 块（视频带 `image` 封面，课程材料指向课件目录）。

---

## 4. Agent 拓扑

**状态：拓扑已落进 skill 文件；课程材料的产物留在工作目录里。**

四个 skill 的拓扑写在各自的 SKILL.md：

| skill | 拓扑 |
|---|---|
| `youtube-render-pdf` | master → outline → figure → 五个 writer（按视频长度切 2–7 段）→ consistency → master 集成（`SKILL.md:27-34`） |
| `bilibili-render-pdf` | 同上，多一节「AI 字幕优先」（`SKILL.md:64-77`） |
| `review-notes-pdf` | master → outline → figure → writers（**按课件文件切分**）→ **lint** → consistency → master 集成（`SKILL.md:93-101`） |
| `drive-render-pdf` | 自己的一套 10 步流程（`SKILL.md:24-70`） |

课程材料那条链多了一个 lint 阶段，写者产出 `sections/section_NN.md`，master 集成成 `index.md`。

**产物落盘位置**（AList 分享上的工作目录 `Nextcloud/School/Compute-Intelligence/`）：

| 产物 | 现状 |
|---|---|
| `agent_outputs/chapter_map.json`（54 KB）、`outline.md`（40 KB）、`glossary.md`（56 KB）、`consistency_report.md`（32 KB）、`writer_briefs.md`（7.4 KB） | 在 |
| `build/materials.json`、`build/source-manifest.json`、`build/figures_manifest.json`、`build/cache-manifest.json` | 在 |
| `blog/section_08.md`（2026-10-21 14:19）、`blog/section_10.md`（20:06）、`blog/images/fig_052.png`–`fig_065.png` | 在（课程笔记写者的分片与图） |
| `sections/section_NN.tex` | 在（2026-09-22 的 LaTeX 那轮留下的） |

---

## 5. blog 侧改造

### 5.1 视觉 token

**状态：大部分已完成；「ChatGPT 色阶」和「4px 间距单位」没做。**

| 计划项 | 状态 | 证据 |
|---|---|---|
| 三份重复调色板合并成一份 `light-dark()` | 已完成 | `src/styles/global.css:8-50`（`color-scheme: light dark`） |
| 补 `[data-theme="light"]` | 已完成 | `global.css:52-54` |
| `--accent-color` / `--accent-soft` | 已完成 | `global.css:26-27`（`light-dark(#3566f0,#4b83f5)`） |
| alpha 阶梯 05/08/10/12/16/20/25 | 已完成 | `global.css:30-37` |
| radius 阶梯 2/4/6/8/10/12/16/20/24 | 已完成 | `global.css:39-47` |
| 全局 `:focus-visible` 焦点环 | 已完成 | `global.css:199-201` |
| 「正文保留 serif，UI chrome 改 sans」 | 未做 | `src/styles/` 与 `src/layouts/` 里没有 sans 字体变量 |
| 「4px 间距单位」 | 未做 | `global.css` 没有间距刻度变量 |
| 「ChatGPT 色阶：surface/text/muted/faint/border 取 #fff #f9f9f9 #0d0d0d #5d5d5d #8f8f8f #0000001a」 | 未做 | 站点原色板保留（`--background-color: light-dark(#fffffd,#0e0e0c)`、`--primary-color: light-dark(#2a2a28,#f2f2f0)`、`--remark-color: light-dark(#757575,#8f8f8c)`、`--weak-color: light-dark(#9f9f9c,#6b6b68)`、`--shadow-color: light-dark(#cdcdcc,#333331)`），只加了 accent 与两级阶梯 |

同步改动也都做了：shiki `defaultColor: "light-dark()"` + dark 侧 `colorReplacements`（`astro.config.ts:120-127`）；`ThemeSwitcher.svelte` 派发 `theme:change`（`:17`）且只在用户没选过时跟随系统（`:47-52`）；`public/feed.css` 同样换成 `light-dark()`（6 处）；`src/graph/content.ts` 的 OG 图底色 `#fffffd`→`#ffffff`、次要色 `#555555`→`#5d5d5d`（8326366）。

顺带修的：表格边框 3px→1px（`markdown.css:19`）、引用块 4px→2px（`markdown.css:232`）、四个监听器泄漏（TOC、Position、medium-zoom、Note/Jotting 内层 `<main>`）。

### 5.2 文章动作条 + chip 行

**状态：已完成。**

`src/components/ArticleActions.astro`：复制链接 / 复制 Markdown 原文 / 分享（`navigator.share` 不可用时退回复制链接）。chip 行在 note、jotting、lecture 三个详情页都有，样式是 `.chip`（`global.css:370-379`，1px `--surface-10` 描边、`--surface-05` 底、12px muted）。

### 5.3 mermaid 修复

**状态：已完成。三条坑都填了。**

| 坑 | 处理 | 证据 |
|---|---|---|
| 判定按代码块首词猜，只覆盖 14 个关键字，`xychart`/`sankey`/`treemap`/`timeline` 永远不渲染；**且会误判**——Python 块以 `graph = torch.cuda.CUDAGraph()` 开头时被当成图表 | 改读 `pre[data-language="mermaid"]`。Shiki 把语言写在 `<pre>` 的 `data-language` 上 | `src/utils/mermaid.ts:11`；构建产物实测：python 块 `data-language="python"`，mermaid 块 `data-language="mermaid"` |
| 主题写死 `dark`，浅色页上是深色块 | 改成 `theme: "base"`，颜色全部取站点 CSS 变量；明暗由解析出的背景色亮度判定（`isDark()`，YIQ 权重 299/587/114）；`theme:change` 时重画 | `mermaid.ts:109-145`、`:174-189` |
| 整包 mermaid 常驻每个页面的 Base chunk | 改 `await import("mermaid")` 动态加载，只有含图的页面才拉 | `mermaid.ts:93` |

**误判不是假想。** 仓库里就有两个 Python 块首词是 `graph = torch.cuda.CUDAGraph()`：`src/content/note/zh-cn/CUDA Graph 实战入门.md:52` 和 `src/content/note/zh-cn/MiniMind-O CUDA Graph优化实录.md:232`。旧逻辑按首词猜，这两个块会被当成图表。

**实测体积（构建产物）：**

| chunk | 原始 | gzip |
|---|---|---|
| `Base.astro_astro_type_script_index_0_lang.*.js` | 12,400 B | **4,298 B** |
| `mermaid.core.*.js`（独立 chunk，按需加载） | 480 KB | 137,554 B |

计划 §5.3 预测「140KB gzip 降到 3.5KB」。实测 Base chunk 是 4.3KB gzip（略高于预测），mermaid 本体 137.5KB gzip 变成独立 chunk，只在有图的页面加载。

**复制按钮保住了：** 旧代码用 `outerHTML` 整块替换容器，复制按钮一起没了。现在 SVG 注入 `pre` 内部（`mermaid.ts:161`），容器不动，按钮还在（实测产物里复制按钮存在）。

### 5.4 交互模块库

**状态：未开始（在提交历史里）。**

计划 §5.4 列了 7 个模块（chart / slider / compare / steps / annotate / 可排序表格 / 代码块 chrome）和约 19h 工时，落在 `src/utils/remark/interactive.ts` + `src/utils/interactive.ts` + `src/styles/interactive.css`。

**当前工作区里有在做，但没提交，而且不干净。** 实测（`git status --short`）：

```
 M astro.config.ts
 M src/layouts/Base.astro
 M src/utils/remark/directive-text-guard.ts
?? docs/interactive-modules.md
?? scripts/test-interactive.ts
?? src/styles/interactive.css
?? src/styles/modules/
?? src/utils/code-chrome.ts
?? src/utils/interactive.ts
?? src/utils/interactive/
?? src/utils/remark/escape.ts
?? src/utils/remark/interactive.ts
?? src/utils/remark/modules/
```

未提交文件里有 `src/utils/remark/modules/{chart,slider,compare,annotate,sortable,step,steps}.ts`、`src/utils/interactive/{chart,slider,compare,annotate,code,steps,table}.ts`、`src/styles/modules/*.css`，还有一份契约文档 `docs/interactive-modules.md`（写明「模块改 mdast 节点，不写 HTML 字符串」）。

**`pnpm check` 现在报 4 个类型错**，全在这批未提交文件里：

```
scripts/test-interactive.ts:26       'fileURL' does not exist in type 'MarkdownProcessorRenderOptions'
scripts/test-interactive.ts:24       'shikiConfig' is not assignable to parameter of type 'AstroMarkdownProcessorOptions'
src/utils/remark/directive-text-guard.ts:32   No overload matches this call
src/utils/remark/interactive.ts:117           No overload matches this call
```

`pnpm build` 仍能过（20:29 跑通，57 页），因为类型错不影响打包。

### 5.5 三个必须处理的既有约束

**状态：部分完成。**

| 约束 | 状态 |
|---|---|
| `global.css` 的 `svg { fill: currentColor }` 会给图表几何填文字色 | 预留了出口但没模块用：`global.css:195` 的 `svg[data-chart] { fill: none; }`；`svg { fill: currentColor }` 本身仍在（`global.css:187`） |
| medium-zoom 绑 `.markdown img:not([data-nozoom])`，控件内图会点开大图 | 逃生舱 `data-nozoom` 仓库里本来就有（`Linkroll.astro:50`），选择器也是计划里的写法（`Base.astro:34`）。但目前没有任何控件，约束没被触发 |
| 控件内 `###` 会被收进 TOC | 未做。`docs/interactive-modules.md` 提的解法（把组件内标题节点改成 `data.hName = "p"`）还没落地 |

### 5.6 MDX

**状态：未使用；计划里那条坑仍然成立。**

仓库里唯一的 `.mdx` 是 `src/content/information/zh-cn/linkroll.mdx`，`lecture` collection 里没有 `.mdx`。

计划 §5.6 写「`attr.ts` 的 `{#id}` 属性语法与 MDX 互斥——`## H {#id}` 直接让构建失败（acorn 解析 JSX 报错）」。**实测复现：** 临时放一个 `src/content/information/zh-cn/_probe-attr.mdx`，内容是 `## Heading with attr {#probe-id}`，`pnpm build` 报：

```
[@mdx-js/rollup] Could not parse expression with acorn
file: .../_probe-attr.mdx:3:23
  Unexpected token
```

探针已删。

### 5.7 `@media print`

**状态：未开始。** `src/styles/` 里没有任何 `@media print`。计划 §5.7 的一页 print CSS（展开折叠面板、隐藏 range 与排序按钮、图表 `break-inside: avoid`、滑条复位）一行没写。

### 5.8 不做

**状态：维持。** 笔记关系图没做（`src/graph/content.ts` 仍是 satori 生成的 Open Graph 图）；侧栏 artifact 没做；滚动驱动动画没做。

---

## 6. PDF 导出

**状态：未开始。**

计划 §6 写「手动跑脚本：`pnpm build` 然后 `node scripts/export-pdf.mjs <note-id>`，Chrome headless `--print-to-pdf`」。

**实际：`scripts/` 里只有 `new.ts`（和一个未提交的 `test-interactive.ts`）。`scripts/export-pdf.mjs` 不存在，`src/styles/` 里没有 print CSS。**

契约和四个 skill 现在的说法是「PDF 由人需要时从 blog 页面打印」——**没有任何实现支撑这句话**：没有 print 样式，页面直接打印会带上导航、侧栏、目录。

**两个已知降级仍未实测**（审计时的旧数据，不是本轮测的）：

| 问题 | 计划里的处理 | 现状 |
|---|---|---|
| Chrome headless 出的 PDF 无 outline 书签树（xelatex+hyperref 有 3 项） | 接受，或用 pypdf 补 outline，**未实测**哪个省事 | 未实测 |
| CJK 被嵌成 Type3 字体（60KB vs LaTeX 4.9KB） | 接受，或在 print CSS 里指定系统字体栈，**未实测** | 未实测 |

计划 §11 还列了一条「Chrome headless 打印时 mermaid 是否渲染完成」——mermaid 现在是客户端异步渲染 + 动态 import，headless 快照要在图注入后才取样，这条更悬，仍未实测。

---

## 7. 旧资产处置

**状态：已完成，四项都有落地证据。**

| 对象 | 计划里的处置 | 实际 |
|---|---|---|
| 四个 skill 的 `.tex` 模板 | 删掉 | 已删。四个 skill 目录里没有 `.tex` |
| `review-notes-pdf/scripts/` 9 个脚本 | 删掉 | 实际是**换成 blog 版等价物，不是全删**：`lint_handout.py` → `lint_note.py`，`build_handout.py` → `note_links.py`（TASKS.md 记的是这个换法）。现在目录里 8 个 `.py`：`build_cache.py`、`build_source_manifest.py`、`diff_source_manifest.py`、`extract_figure.py`、`lint_note.py`、`note_links.py`、`publish_alist.py`、`sync_alist.py` |
| `review-notes-pdf/references/` 6 个 md | 重写为 blog 版写作契约 | 目录仍是 6 个 md，但内容按课程材料重写：`structure-guide.md`（9.0 KB，2026-10-22 17:53 改成按课件文件组织）、`pedagogical-flow.md`（4.6 KB）、`figure-policy.md`（6.8 KB）、`multi-agent-workflow.md`（15 KB）、`material-change-workflow.md`（12 KB）、`alist-api.md`（6.7 KB）。**共享契约搬到了 `_shared/blog-note-contract.md`（319 行）**，四个 SKILL.md 各自只写平台特有的部分 |
| 旧 PDF 产物 | 保留在磁盘，不再生成新的 | 保留。`Video-Course/` 14 个 PDF；`review-notes-20260922/讲义.pdf` 8.2 MB；AList 上 `讲义/计算智能_期末自学讲义.pdf` 17 MB |
| 存量讲义 14 篇 + `review-notes-20260922` 1 篇 | 不迁移 | 没迁移。`Video-Course/` 14 个 PDF 只在磁盘上；两篇新讲义是重写的，不是搬的 |

---

## 8. 要修的既有缺陷

**状态：7 项里 6 项已修，1 项（soffice）没修，而且它现在是死链的根因。**

| 缺陷 | 状态 | 证据 |
|---|---|---|
| Google API key 硬编码在 `drive-render-pdf/SKILL.md` | **已修** | 不再硬编码，改为从 yt-dlp 源码里 grep 出来（`drive-render-pdf/SKILL.md:112-120`） |
| `soffice` 是指向不存在路径的坏 shim（`/opt/homebrew/bin/soffice`） | **未修** | 见下面 §9.3 |
| mermaid 主题写死 dark + 关键词误判 + 整包常驻 | **已修** | 见 §5.3 |
| `[data-theme="light"]` 缺失 | **已修** | `global.css:52-54` |
| TOC `addEventListener` 每次换页累积 | **已修** | TOC / Position / Base / ArticleActions 四个脚本都用 `window.__xxxRegistry` + `bound` 标志，整个会话只注册一次 |
| `Position.astro` scroll 监听泄漏 | **已修** | `Position.astro:96-99`，同一套 registry 模式 |
| `Note.svelte` / `Jotting.svelte` 内层 `<main>` 嵌套 | **已修** | 改成 `<div>`（`Note.svelte:106`、`Jotting.svelte:77`），注释写明 swup `containers: ['main']` 找第一个 main 会歧义 |

计划外还修了两个（记在 TASKS.md）：

| 新发现 | 状态 | 证据 |
|---|---|---|
| `remark-directive` 把 `[03:10](url)` 里的冒号当指令标记，链接文字丢掉第一个冒号之后的内容 | **已修** | 新增 `src/utils/remark/directive-text-guard.ts`，并挂进 `astro.config.ts` |
| i18n 的 `home.note` 缩进错误（被解析成 `home.jotting` 的子键） | **已修** | `src/i18n/zh-cn/index.yaml` 缩进已平 |

---

## 9. 施工过程中发现的问题

这一节记的是计划里没有、干活时才发现的事。

### 9.1 Astro content store 会把上一版笔记发出去

**现象：** 提交历史里，RL-Kernel 笔记被重建过一次（335fd28：把重写后的笔记搬进旧目录名，删掉旧版）。CI 上构建出来的页面有时是**上一版**的正文。

**机制（读源码 + 复现）：**

1. Astro 的 content store（`node_modules/.astro/data-store.json`）**每个 collection item 存一条**，key 是 id（这里 id 由文件路径生成），value 里带 `body` 和 `digest`。实测这个文件 9.0 MB，3068 个顶层 key，lecture collection 下 3 条。
2. glob loader（`node_modules/astro/dist/content/loaders/glob.js:83-91`）在 `digest` 对得上时**直接复用缓存里的 body**，不重新解析。
3. 笔记改名或换目录时 id 变了，但**上一版条目还占着 collection map 的 key**。loader 再看到同一个 id 就报 `store.has(id)`，打印 `Duplicate id "..." found in ...`（`glob.js:106-109`）。

**复现（本地实测）：**

```bash
cp /tmp/data-store.backup.json node_modules/.astro/data-store.json   # 还原重建前的 store
pnpm build
```

输出：

```
[WARN] [glob-loader] Duplicate id "zh-cn/vLLM小课堂（十八）RL-Kernel_大模型RL后训练的一致性实践"
found in .../src/content/lecture/zh-cn/vLLM小课堂（十八）RL-Kernel_大模型RL后训练的一致性实践/index.md.
Later items with the same id will overwrite earlier ones.
```

再把 store 里那条的 body 换成带 `STALE-MARKER` 的版本，重建，**构建出的 HTML 里有 `STALE-MARKER`，源文件里没有**；换回干净的 store 再重建，标记消失。

**修复（已落在 CI）：** `.github/workflows/deploy.yaml` 现在先恢复 astro 缓存（`path: node_modules/.astro`，`key: astro-cache-${{runner.os}}-${{github.sha}}`，`restore-keys` 回退到 `astro-cache-${{runner.os}}-`），紧接着 `rm -rf node_modules/.astro/data-store.json .astro`，再 `pnpm build`。字体文件在 `node_modules/.astro/fonts` 里，随缓存一起留下——字体服务当时取不到，缓存不能丢。

顺带一个 CI 修复（86b0240）：`pnpm/action-setup` 和 `package.json` 都写了 pnpm 版本号，action 拒绝运行；改成只由 `package.json` 决定（`pnpm@9.15.9`）。

### 9.2 TOC 的 `scrollIntoView` 会把整页顶走

**现象：** 目录高亮切到新标题时，页面会跳一下。

**原因：** 旧代码对高亮项调 `activeItem.scrollIntoView({behavior:"smooth", block:"nearest"})`。`scrollIntoView` 会滚动**所有可滚祖先**，包括 body，所以整页跟着动。

**现在：** TOC 自己判断当前章节（`currentSection()`，取视口 25% 线以上最后一个标题；页面滚到底时取最后一个），然后**只改目录面板的 `scrollTop`**（`TOC.astro:279-319`）。顺带修了两个同源问题：观察器的 `rootMargin: "-10% 0px -70% 0px"` 让标题滚出窄带就失去高亮（改成自己算）；`.toc-nav` 没有高度约束，长目录把 sticky 容器撑出视口（加 `flex: 1; min-height: 0`）。

### 9.3 课件公式是 MathType 对象，文本抽取读不出来

**现象：** 笔记第 10、12 章起初只有课件标题，没有公式。

**原因：** 课件里的公式是 **MathType 对象，以 WMF 位图存在 pptx 里**。文本抽取只读文本层，公式那层读不到。

**处理（335fd28 之前，b29a157）：** 解 MathType 的 WMF 记录流——记录长度按 16 位字计、`META_EXTTEXTOUT` 用紧凑负载、`META_MOVETO` 先出行后列——用 `Symbol.ttf` 的 cmap 还原希腊字母与运算符，重画后逐条对课件 media 核对。修掉 4 处错：DOC 目标函数里多出的平方根、MOMMOP 缩放因子 η 未声明为自选符号、WeB 加权和「落在 0 与 1 之间」的说法错、MCOP 约束下标被说成课件原文。

**实测（同一文件逐章数 `$$` 块）：**

| 章节 | 69aaf35（按课件重建时） | 现在 |
|---|---|---|
| 第 9 章 | 0 | 7 |
| 第 10 章 | 0 | 14 |
| 第 12 章 | 0 | 19 |

第 9 章是 2f356a6 单独做的，第 10、12 章是 b29a157 做的。

### 9.4 `extract_materials.py` 走 soffice，对每个 pptx 返回 0 个 block

**现象：** 抽课件文本时每个 deck 都是 0 block。

**原因链：**

1. `extract_materials.py:217-236` 的 `extract_presentation()` 先把 pptx 转 PDF：`convert_with_soffice(path, converted_dir, "pdf")`。
2. `convert_with_soffice()`（`:126-141`）跑 `soffice --headless --convert-to pdf`。
3. `/opt/homebrew/bin/soffice` 是个指向 `/opt/homebrew/Caskroom/libreoffice/26.8.0/.homebrew-command-wrappers/soffice` 的符号链接，那个 wrapper 内容是 `exec "/Applications/LibreOffice.app/Contents/MacOS/soffice" "$@"`——**`/Applications/LibreOffice.app` 不存在**。

**实测：**

```bash
soffice --headless --version
# /opt/homebrew/bin/soffice: line 2: /Applications/LibreOffice.app/Contents/MacOS/soffice: No such file or directory
# exit=126
```

用真课件跑抽取：

```bash
python3 ~/.agents/skills/open-book-index/scripts/extract_materials.py \
  --input ".../课件/Chapter 1_ Introduction to EAs.pptx" \
  --output-workdir /tmp/em-test-workdir
# {"blocks": 0}
# summary.json: processed_files 0, skipped_files 1, block_count 0
#   reason: "very little extractable slide text; visual inspection or OCR may be required"
```

**绕法（已用）：** 改用 python-pptx 直读 pptx 文本层，不再依赖 soffice 转换（本地有 python-pptx 1.0.2）。

**还没修：** 那个坏 shim 本身还在 `/opt/homebrew/bin/soffice`，`convert_with_soffice()` 这条路仍然会失败。

### 9.5 AList 分享从 /hdd 搬到 /Nextcloud

**现象：** 源页链接指向一个不存在的路径。

**处理（b29a157）：** 笔记里所有源页链接与 `source_url` 从 `http://10.188.31.55:5244/hdd/School/...` 改成 `.../Nextcloud/School/...`。实测当前笔记：501 处 `/Nextcloud/`，0 处 `/hdd/`。

**但换完还是没通。** 分享上 `converted/` 目录只有**第 0–8 章**的 9 个 PDF，而笔记引用了第 9、10、12 章的渲染 PDF：

| 被引用的渲染 PDF | 笔记里引用次数 | 分享上是否存在 |
|---|---|---|
| 第 9 章 | 112 | **不存在** |
| 第 10 章 | 40 | **不存在** |
| 第 12 章 | 36 | **不存在** |
| 第 0–8 章 | 311 | 存在 |

合计 499 处里 **188 处指向分享上没有的文件**。这 188 处正好对应 9.4 里那条：`soffice` 坏了，第 9/10/12 章的 pptx 从没渲染成 PDF 传上去。课件原件（`课件/` 下的 12 个 pptx）是齐的。

**测法说明：** 分享用 WebDAV 挂在 `/Volumes/dav`（`http://10.188.31.55:5244/dav/`），可以直接 `ls` 目录。**不能用 HTTP 判存活**——`http://10.188.31.55:5244/Nextcloud/.../pdf` 对存在和不存在的文件都返回 200 加同一份 3,905 字节的 SPA 页面；`/d/` 直链要签名（`expire missing`）。`/api/fs/list` 对未登录用户返回 401。

### 9.6 两条同源的笔记目录同时存在

`src/content/lecture/zh-cn/` 下先后出现过三个目录：`vLLM小课堂十八_RL-Kernel训推一致性`（重写版）、`vLLM小课堂（十八）RL-Kernel_大模型RL后训练的一致性实践`（旧版）、`计算智能_进化算法与优化方法`。335fd28 把重写版搬进旧目录名并删掉旧版，**现在只剩两个笔记目录**，与 §2.3 的章节统计对得上。中间态是 §9.1 那次撞车的来源。

---

## 10. 章节与笔记的最终状态

**状态：两篇笔记都已发布，构建通过。**

| 项 | 计算智能 | RL-Kernel |
|---|---|---|
| 目录 | `计算智能_进化算法与优化方法` | `vLLM小课堂（十八）RL-Kernel_大模型RL后训练的一致性实践` |
| frontmatter `draft` | false | false |
| `##` 章节 | 12 | 14 |
| `###` / `####` 小节 | 102 / 74 | 53 / 0 |
| 引用 | 499 处页码链接（89 处带 `文件名 · p.N` 形状，410 处裸 `[p. N]`） | 115 处时间戳 |
| 图 | 65 张 PNG（2000×1500） | 12 张 JPG（1536×817） |
| 章节索引 | 12 行 | 14 行 |
| 原文引用块 | 58 行 | 2 行 |
| 正文体积 | 333,011 B | 31,635 B |
| `lint_note.py` | OK | OK |

**构建与检查：**

```bash
pnpm check     # 52 个文件，0 errors / 0 warnings / 0 hints（干净树上，2026-10-22 18:56）
pnpm build     # 57 页，Complete!（2026-10-22 20:29）
```

`pnpm check` 的 0 错是**在 §5.4 那批未提交文件落地之前**测的；现在同一命令报 4 个错，全在未提交的交互模块文件里。

---

## 11. 未开始 / 未实测 / 还没做完

按「能不能收尾」排序。

### 11.1 交互模块库（未开始）

计划的 7 个模块（chart / slider / compare / steps / annotate / 可排序表格 / 代码块 chrome）一行没进提交历史。工作区里有一批未提交的实现（见 §5.4），配套契约 `docs/interactive-modules.md` 写明了「模块改 mdast 节点，不写 HTML 字符串」的渲染模型、`InteractiveContext` 的三个共享工具、每模块只改自己那几个文件的分工。现在的状态：

- 未提交，且 `pnpm check` 报 4 个类型错（`scripts/test-interactive.ts:24,26`、`src/utils/remark/directive-text-guard.ts:32`、`src/utils/remark/interactive.ts:117`）。
- `src/content/` 里 `^:::` 命中 0 次，没有任何笔记用上这些模块。
- §5.5 的两条约束因此也没被触发。

### 11.2 PDF 导出脚本（未开始）

`scripts/export-pdf.mjs` 不存在；`src/styles/` 里没有 `@media print`。契约里「PDF 由人需要时从 blog 页面打印」目前没有实现支撑。连带未实测的：

- Chrome headless 出的 PDF 无 outline 书签树（对比 xelatex+hyperref 3 项）——计划里就写「未实测哪个更省事」。
- CJK 被嵌成 Type3 字体（60KB vs LaTeX 4.9KB）——同样未实测。
- headless 打印时 mermaid 是否渲染完成（客户端异步渲染 + 动态 import）。
- `reading-time` 失真：`reading.ts` 仍用 `mdast-util-to-string`。将来 chart 模块把表格换成 SVG 时字数会掉。

### 11.3 计划里被砍掉或改写的部分

| 计划里的东西 | 现在的状态 |
|---|---|
| 「按知识结构重组」当唯一章节规则 | 换成三条按源材料分类的规则（§2.3） |
| 固定七段式章节骨架 | 删掉，列为反模式（§2.3） |
| 公式速查表、跨章符号表、易混对清单 | 从笔记里删掉，契约列为不写。**残留**：计算智能第 4 章还有一个 `### 术语速查`（17 条），lint 不查（§2.2） |
| 引用标签固定 `<文件名> · p.<页码>` | 499 处里只有 89 处是这个形状，410 处是裸 `[p. N]`；lint 和 `build_source_manifest.py` 只看得见 89 处（§2.5） |
| 内联 SVG 示意图（`currentColor` 跟随主题） | 没有内联 SVG，全是位图（§3.4） |
| ChatGPT 色阶的具体色值、4px 间距单位、UI chrome 改 sans | 都没做（§5.1） |
| 交互模块 7 个 + 代码块 chrome | 未开始（§11.1） |
| `@media print` 一页 CSS | 未做（§5.7） |
| `scripts/export-pdf.mjs` | 未做（§11.2） |
| 旧 LaTeX 两个 bug：tcolorbox 标题宏未加花括号、review 模板硬依赖 `Noto Serif CJK SC` | 模板删了，bug 随模板作废。老 `讲义-template.tex` 里 `title={#1}` 5 处、`Noto Serif CJK SC` 1 处仍在磁盘上，但已经不在任何 skill 的路径上 |

### 11.4 工具链里还坏着的东西

- **`/opt/homebrew/bin/soffice` 坏 shim 仍在**（`/Applications/LibreOffice.app` 不存在，`soffice --headless --version` 退出码 126）。课程文本抽取已经改走 python-pptx 直读，但 `convert_with_soffice()` 这条路还坏着。**连带后果**：第 9/10/12 章的渲染 PDF 从没上传分享，笔记里 188 处页码链接指向分享上没有的文件（§9.4、§9.5）。
- **MDX + `{#id}` 仍然互斥**（`## H {#id}` 构建即失败，acorn `Unexpected token`）。只要讲义还可能用 MDX 就得记住这条（§5.6）。
- **`draft` 默认值与发布流程没有脚本兜底**：契约说生成后保持 `draft: true`、人确认后再改 false 并单独提交。实际两篇都是人工改的（808db5d、02731aa），没有门禁。

### 11.5 任务记录本身的欠账

**已修（本轮一并处理）。** 改写本文之前，`TASKS.md` 的「下一步」段还停在两个已经完成的项上：阶段 B mermaid 修复（b5843ef 已做）、阶段 C 提交第 10、12 章（b29a157 已做）；阶段 D/E/F 三段不存在，§9.1 的 content store 发现、坏 soffice、188 处死链、89/410 的引用标签形状缺口都没有登记。

现在 `TASKS.md` 的「下一步」段改成六段：B、C 标注已完成并注上提交号，D（交互模块库）、E（PDF 导出）、F（工具链欠账）写清每一步的可执行动作。

**仍欠：** 阶段 F 第 3 条（引用标签统一成 `<文件名> · p.<页码>`）要动两篇笔记的正文，是内容编辑而不是脚本改动，没法在本文改写这一轮里顺手做掉。

---

## 12. 执行顺序

**状态：实际执行顺序如下。**

| # | 提交 | 内容 |
|---|---|---|
| 1 | 8326366 / 98c8d03 | blog token 合并、`[data-theme="light"]`、ArticleActions、chip 行、四个监听器泄漏、Note/Jotting 内层 `<main>` |
| 2 | 361ff90 | `lecture` collection：详情页、列表页、导航入口、`Section` 类型加 `"lecture"` |
| 3 | c052e07 → 02731aa | 第一篇讲义（RL-Kernel）落盘并发布 |
| 4 | 209d6fa → 808db5d | 第二篇讲义（计算智能）落盘并发布 |
| 5 | 8bde276 | TOC 三处修复：页面被顶、滚到底不跟随、目录太长 |
| 6 | 69aaf35 | 按课件文件重建计算智能笔记：12 章对 12 个课件文件，横向对照并进第 7 章，速查章整章删除 |
| 7 | f4bef41 → 6999789 | CI：清 content store、留字体缓存 |
| 8 | 86b0240 | CI：pnpm 版本只由 package.json 决定 |
| 9 | 1f552e9 | 第 0 章内容从第 1 章移出，引用改指本课件 |
| 10 | a7236ce | 补回被重建丢掉的 cardlink 与 frontmatter |
| 11 | 2f356a6 | 第 9 章公式从课件补回 |
| 12 | b5843ef | mermaid 三处修复 |
| 13 | b29a157 | 第 10、12 章公式从课件补回；AList 路径 /hdd → /Nextcloud |
| 14 | 75205d4 | RL-Kernel 时间戳逐条校正 |
| 15 | 335fd28 | RL-Kernel 笔记按 slide 顺序重排：14 章 53 小节，115 处时间戳，图改从 1920×1080 源切 |

计划 §12 排的顺序是 A→B→D→E→C→F。实际：blog 侧（A）在最前，collection（D）第二，skill 重写（E）与两篇笔记交错进行，mermaid（B）在中段，交互模块（C）与 PDF 导出（F）**都没做**。
