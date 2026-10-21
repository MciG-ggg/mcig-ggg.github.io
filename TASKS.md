# TASKS

## 已完成

### 阶段 A：blog token + 动作条 + chip 行（分支 `feat/chatgpt-ui-tokens`）

- 合并三份重复调色板为一份 `light-dark()`，修「系统深色时用户选浅色无效」的 bug
- Shiki 改用 `defaultColor: "light-dark()"`，代码块跟随主题，删掉两处 `!important` 覆盖
- 新增 `--accent-color`、alpha 阶梯 `--surface-05..25`、radius 阶梯 `--radius-2..24`、全局 `:focus-visible`
- 新增 `ArticleActions.astro`：复制链接 / 复制 Markdown 原文 / 分享
- note 与 jotting 详情页改用 ChatGPT 风格 chip 元信息行
- 表格边框 3px → 1px，引用块 4px → 2px
- 修四个既有 bug：TOC 与 Position 的监听器泄漏、medium-zoom 每次换页叠加 document 监听器、Note/Jotting 内层 `<main>` 与 swup `containers: ['main']` 冲突

## 下一步

### 阶段 B：mermaid 修复（2h）

1. 改 `src/layouts/Base.astro` 的 mermaid 判定：读 `pre[data-language="mermaid"]`，删掉 14 词首词猜测
2. 主题跟 `document.documentElement.dataset.theme`，监听 `theme:change` 重绘
3. SVG 注入 `pre` 内部，不动容器（现在 `outerHTML` 整块替换删掉了复制按钮）
4. `await import("mermaid")` 动态加载，Base chunk 从 140KB gzip 降到 3.5KB
