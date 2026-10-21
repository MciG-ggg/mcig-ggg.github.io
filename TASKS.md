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
- `计算智能：进化算法与优化方法`：从 LaTeX 讲义转换，10 章、65 张图、650 处源页链接

### 顺带修的 blog bug

- 新增 `src/utils/remark/directive-text-guard.ts`：`remark-directive` 会把 `[03:10](url)` 里的冒号当指令标记，链接文字丢掉第一个冒号之后的内容
- 修 i18n 的 `home.note` 缩进错误（被解析成 `home.jotting` 的子键）

## 下一步

### 阶段 B：mermaid 修复

1. 改 `src/layouts/Base.astro` 的 mermaid 判定：读 `pre[data-language="mermaid"]`，删掉 14 词首词猜测
2. 主题跟 `document.documentElement.dataset.theme`，监听 `theme:change` 重绘
3. SVG 注入 `pre` 内部，不动容器（现在 `outerHTML` 整块替换删掉了复制按钮）
4. `await import("mermaid")` 动态加载，Base chunk 从 140KB gzip 降到 3.5KB
