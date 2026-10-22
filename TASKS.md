# TASKS

## 已完成

### 阶段 A：blog token + 动作条 + chip 行

- 合并三份重复调色板为一份 `light-dark()`，修「系统深色时用户选浅色无效」的 bug
- Shiki 改用 `defaultColor: "light-dark()"`，代码块跟随主题，删掉两处 `!important` 覆盖
- 新增 `--accent-color`、alpha 阶梯 `--surface-05..25`、radius 阶梯 `--radius-2..24`、全局 `:focus-visible`
- 新增 `ArticleActions.astro`：复制链接 / 复制 Markdown 原文 / 分享
- note 与 jotting 详情页改用 ChatGPT 风格 chip 元信息行
- 表格边框 3px → 1px，引用块 4px → 2px
- 修四个监听器泄漏：TOC 与 Position 的 scroll 监听、medium-zoom 每次换页叠加 document 监听器、Note/Jotting 内层 `<main>` 与 swup `containers: ['main']` 冲突

### `lecture` collection

- 新增 collection、详情页、列表页、导航入口
- `Note` 列表组件加 `base` 属性，两页共用
- `Section` 类型加 `"lecture"`

### 两个 skill 改造

四个 skill 从「产出 LaTeX → PDF」改为「产出 blog 原生 Markdown，落进 `lecture` collection」。

- 共享写作契约 `_shared/blog-note-contract.md`（落盘位置、frontmatter、章节组织、语气、图的要求、检查清单）
- `youtube-render-pdf` 161 行、`bilibili-render-pdf` 202 行、`drive-render-pdf` 485 行、`review-notes-pdf` 299 行
- 删掉全部 `.tex` 模板和 LaTeX 耦合脚本；`lint_handout.py` → `lint_note.py`，`build_handout.py` → `note_links.py`
- `final-review-suite` 同步更新
- 修 `drive-render-pdf/SKILL.md` 里硬编码的 Google API key（改为从 yt-dlp 源码读）
- alist 地址从 `10.188.31.124`（已下线）改成 `10.188.31.55`

### 两篇笔记

- `vLLM小课堂（十八）RL-Kernel 训推一致性`：B 站 AI 字幕（`ai-zh`，未用 Whisper），15 章、17 张图、36 个可点击时间戳
- `计算智能：进化算法与优化方法`：按课件文件编排，12 章（第 0-10、12 章，课件无第 11 章）、111 小节、65 张图、440 处源页链接、0 死链

### 课程讲义按课件编排

讲义原来只覆盖 12 个课件里的 9 个：开篇被并进第 1 章，约束优化那 3 个课件没写，第 9 章「五大算法横向对照」是讲义自己综合的、没有课件撑着。

- 章节号照抄课件文件编号，文件缺号时笔记也缺号
- 一个课件文件一章，不合并、不丢弃
- 不写没有课件撑着的章：横向对照内容并进第 7 章（算法比较方法），机制对照表移到前置区
- 速查章整章删除：公式速查表、跨章符号表、易混对清单
- 提取课件文本用 python-pptx 直读。`extract_materials.py` 走 soffice，而 soffice 是指向不存在路径的坏 shim，所以它对 pptx 返回 0 个 block

### 顺带修的 blog bug

- 新增 `src/utils/remark/directive-text-guard.ts`：`remark-directive` 会把 `[03:10](url)` 里的冒号当指令标记，链接文字丢掉第一个冒号之后的内容
- 修 i18n 的 `home.note` 缩进错误（被解析成 `home.jotting` 的子键）

### 目录（TOC）三个问题

- **页面被顶**：`scrollIntoView` 会滚动所有祖先容器，包括 body。改成只滚目录面板
- **滚到底不跟随**：`rootMargin: "-10% 0px -70% 0px"` 让观察带只有视口 10%~30%，标题滚出这条带就没有可见项。改成取「视口线上方最后一个标题」，并在页面到底时取最后一个
- **目录太长**：`.toc-nav` 没有高度约束，159 项全部铺开。给它加 `flex: 1; min-height: 0`，父容器加 `min-h-0`

### 课程讲义结构

「按知识结构重组」这条规则只适用于视频，不适用于课程材料。课程章节顺序就是教学大纲，复习的人要把「第 7 章」对到笔记和试卷上。

- 共享契约分两种源类型：视频按知识结构重组，课程保持章节顺序 1:1
- `##` 是章节（带课程章节号），`###` 是小节，章节标题不许丢
- 章节超过五个时开头放章节索引
- 笔记里不写公式速查表、符号表、易混对清单、自测题清单、考前冲刺清单
- `计算智能` 那篇重排成 9 章，速查章删掉

### 第 0 章插入后的第 1 章清理

第 0 章「开篇」按课件 26 页插入后，第 1 章仍带着 9 处指向 `Chapter 0_ 开篇.pdf` 的引用。

- 逐处核对 `Chapter 0_ 开篇.json`（26 页）与 `Chapter 1_ Introduction to EAs.json`（13 页），能对上第 1 章课件页的改指第 1 章，只第 0 章才讲的整段删掉
- 删掉第 1 章的「八类复杂优化问题」一节：第 0 章「课程的九个方向」已完整讲这八类，`可行域 (feasible region)` 英文注记补回第 0 章
- `fig_007`（第 0 章第 7 页课程主线图）和 `fig_006`（第 0 章第 12 页约束优化图）随内容移到第 0 章，图片文件与编号不动
- 改完第 1 章只剩第 1 章课件的引用，字符数 14266 → 12824

### 第 10、12 章公式复原与核对

- 解码 MathType 的 WMF 记录流（记录长度按 16 位字计、`META_EXTTEXTOUT` 紧凑负载、`META_MOVETO` 先行后列），用 `Symbol.ttf` 的 cmap 还原希腊字母与运算符，重画后逐条核对课件 media
- 修掉 4 处：DOC 目标函数里多出的平方根、MOMMOP 缩放因子 $\eta$ 未声明为自选符号、WeB 加权和"落在 0 与 1 之间"的说法错、MCOP 约束下标被说成课件原文

### vLLM小课堂十八 RL-Kernel 训推一致性

- 按讲者的 slide 顺序重排：14 章，章节边界对着 slide 标题逐帧核对，不按知识结构重排
- 12 张图全部从 `video.mp4` 按整秒切（1920×1080 原始分辨率），裁掉播放器的摄像头窗口和底部控制条，逐张目视确认内容完整
- 115 处可点击时间戳，格式统一 `[MM:SS](url?t=秒)`，超过一小时用 `H:MM:SS`
- ASR 纠错：`VARM`/`WIM`/`威尔姆`/`魏母` → vime；`our kernel` → RL-Kernel；`训退` → 训推；`V0.1.0` → v0.1.0；`MC tr`/`麦克TRO` → Megatron-LM；`rock com` → ROCm；`KA` → CUDA；`LMEL` → lm-eval；`BENCHUART` → benchmark
- 逐条核对引用的秒数落在字幕行上；只落在画面上的（如 GitHub 面板的 RFC 验收条款）单独标注
- 补 `###` 小节：14 个 `##` 章各切 2 到 9 个小节，共 53 个 `###`；正文一字未改，115 处时间戳、12 处图引用原样保留，文件只增 889 字符（全部来自标题行与其空行）

### vLLM小课堂（十八）RL-Kernel 训推一致性：按 slide 重排章节

- 把原来按视频内容写的 17 个平铺 `##` 章合并成 12 章（按用户给的 13 段 slide 序列，实际落 13 章），章节边界逐帧核对 8 张 contact sheet
- 每章下把旧章标题降成 `###` 小节，131 行正文一字未改，17 处图引用、36 处时间戳原样保留
- 补上章首索引（13 行，一章一行），frontmatter 与 `cardlink` 块保持原样

### 文档：讲义体系改造方案改写为 as-built 记录

- `docs/lecture-skill-redesign.md` 从「计划」改写成「已建成什么」的记录：每节标已完成/部分完成/未开始
- 新增「施工过程中发现的问题」一节：Astro content store 撞车发旧版、TOC 的 `scrollIntoView` 顶页、MathType 公式读不出、`extract_materials.py` 走坏 soffice 返回 0 block、AList 从 /hdd 搬到 /Nextcloud
- 下一件事：交互模块库仍未提交（`pnpm check` 报 4 个类型错），PDF 导出脚本与 `@media print` 未写

## 下一步

### 阶段 B：mermaid 修复（已完成，见 b5843ef）

1. `src/layouts/Base.astro` 的 mermaid 判定：读 `pre[data-language="mermaid"]`，删掉 14 词首词猜测
2. 主题跟 `document.documentElement.dataset.theme`，监听 `theme:change` 重绘
3. SVG 注入 `pre` 内部，不动容器（现在 `outerHTML` 整块替换删掉了复制按钮）
4. `await import("mermaid")` 动态加载，Base chunk 从 140KB gzip 降到 3.5KB

### 阶段 C：提交第 10、12 章（已完成，见 b29a157）

1. `git add -A src/content/lecture/zh-cn/计算智能_进化算法与优化方法/index.md && git commit -m "note: expand chapters 10 and 12 with the formulas recovered from the slides"`

### 阶段 D：交互模块库（未提交，`pnpm check` 报 4 个类型错）

1. 修 `scripts/test-interactive.ts:24,26`、`src/utils/remark/directive-text-guard.ts:32`、`src/utils/remark/interactive.ts:117` 的类型错
2. 落地 7 个模块：chart / slider / compare / steps / annotate / 可排序表格 / 代码块 chrome
3. 补 `src/styles/` 的 `@media print` 一页 CSS，展开折叠面板、隐藏 range 与排序按钮、图表 `break-inside: avoid`

### 阶段 E：PDF 导出（未开始）

1. 写 `scripts/export-pdf.mjs`，或先定「人从页面打印」这条路要什么 print CSS
2. 实测两条降级：Chrome headless 的 PDF 无 outline 书签树、CJK 被嵌成 Type3 字体
3. 实测 headless 打印时 mermaid 是否渲染完成

### 阶段 F：工具链欠账

1. 修 `/opt/homebrew/bin/soffice` 坏 shim，或让 `extract_materials.py` 彻底不走 soffice
2. 第 9、10、12 章的渲染 PDF 没上传 AList，笔记里 188 处页码链接指向分享上没有的文件
3. 引用标签统一成 `<文件名> · p.<页码>`，现在 499 处里只有 89 处是这个形状，lint 与 `build_source_manifest.py` 只看得见这 89 处
