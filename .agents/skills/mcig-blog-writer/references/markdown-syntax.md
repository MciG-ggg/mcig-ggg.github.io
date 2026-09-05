# Markdown 扩展语法 (Markdown Syntax)

本博客通过 `src/utils/remark/` 下的 remark 插件提供以下扩展语法。

**重要**：参考 skill（grt-blog-writer）的 `::: gallery / callout / timeline / chat-history / year-card / link-card` 在**本博客不支持**。不要照搬。

---

## 1. Abbreviation（缩写）

**插件**：`abbr.ts`

**语法**：在文末用 `*[ABBR]: Definition` 定义，正文里首次出现的 ABBR 会自动加 `<abbr>` tooltip。

```markdown
*[HTML]: HyperText Markup Language
*[MDP]: Markov Decision Process

正文里写 HTML 第一次出现会自动 tooltip，第二次开始仍然是 HTML。
```

**注意**：定义行会被插件自动从树中移除，不会渲染出来。

---

## 2. Attributes（HTML 属性）

**插件**：`attr.ts`

**语法**：在元素后面或标题末尾接 `{...}`：

```markdown
## 二级标题 {#custom-id .highlight}

[外部链接](https://example.com){target=_blank rel=noopener}

![替代文字](images/foo.png){.responsive width=500}

**加粗文本**{.red}
```

支持的属性语法：
- `#id` — 设置 id
- `.class` — 添加 class（可多个）
- `key=value` / `key="value with spaces"` — 任意 HTML 属性

---

## 3. Cardlink（卡片链接）

**插件**：`cardlink.ts`

**语法**：用 ` ```cardlink ` 围栏代码块，内填 YAML：

````markdown
```cardlink
url: https://example.com/article
title: "文章标题"
description: "一句话摘要"
host: example.com
favicon: https://example.com/favicon.ico
image: https://example.com/og.png
```
````

**渲染效果**：变成带 favicon、host、标题、描述、缩略图的卡片链接（`class="card-link"`）。

**典型用法**：在正文里推另一篇文章 / 资源 / GitHub repo。

---

## 4. Auto Figure（自动 figure）

**插件**：`figure.ts`（rehype）

**语法**：单独一段只放一张图片，**必须**有 `alt`：

```markdown
![架构图：CUDA Graph capture 流程](images/cuda-graph-flow.png)
```

**自动转**：

```html
<figure>
  <img src="images/cuda-graph-flow.png" alt="架构图：CUDA Graph capture 流程" />
  <figcaption>架构图：CUDA Graph capture 流程</figcaption>
</figure>
```

**注意**：
- 段落里**只能**有一张图片（不能跟文字混排）
- 图片**必须**有 `alt`（否则不会转换）
- 图放在同级 `images/` 目录

---

## 5. Reading Time（自动阅读时间）

**插件**：`reading.ts`

**无需手动写**。插件自动计算词数并通过 `frontmatter.words` 暴露给 Astro 组件。

---

## 6. Spoiler（剧透/折叠）

**插件**：`spoiler.ts`

**语法**：用 `!!text!!` 包裹：

```markdown
普通文本 !!这是剧透内容，点击查看!! 继续普通文本。
```

**支持嵌套强调**：

```markdown
!!**加粗的剧透**!!
!!*斜体剧透*!!
```

**渲染**：变成 `<span class="spoiler">text</span>`，需要 CSS 的 hover/click 行为才显示（看博客主题配置）。

**注意**：`!!` 两侧不能紧跟 `!` 或空格，会被识别失败。

---

## 7. Table Wrapper（表格自动包裹）

**插件**：`table-wrapper.ts`

**无需特殊语法**。任何标准 markdown 表格都会被自动包进 `<div class="table-wrapper">`，方便响应式样式。

```markdown
| 维度 | A | B |
|------|---|---|
| 性能 | 7.51× | 5.20× |
| 易用 | 中 | 低 |
```

自动变成：

```html
<div class="table-wrapper">
  <table>...</table>
</div>
```

---

## 8. GitHub Alerts（引用块告警）

**插件**：`remark-github-blockquote-alert`（`astro.config.ts` 里 `[alerts, { legacyTitle: true }]`）

**语法**：引用块首行写 `[!TYPE]`，支持 `NOTE` / `TIP` / `WARNING` / `CAUTION` / `IMPORTANT`。**本博客实际只用了 `NOTE`**（在 5 篇已发布文章中出现 17 次）。

```markdown
> [!NOTE] 这里是一条注意
> 详细内容可以跨多行，都是引用块的普通内容。

> [!TIP]
> 给个建议。

> [!WARNING]
> 小心这里的坑。
```

**典型用法**：给关键段落带色块的提示（在技术文章里当 callout 用）。`legacyTitle` 模式下 `[!TYPE] 标题字` 会把「标题字」当标题块显示。

---

## 9. Math（数学公式）

**插件**：`remark-math` + `rehype-katex`

**本博客：30 篇使用**。

**行内**：`$公式$`

```markdown
损失函数 $\mathcal{L}_{\text{FM}}(\theta)$ 定义如下：
```

**块级**：`$$公式$$`（单独成段）

```markdown
$$\mathcal{L}_{\text{FM}}(\theta) = \mathbb{E}_{t \sim \text{Unif}, x \sim p_t} [\|u_t^{\theta}(x) - u_t^{\text{target}}(x)\|^2]$$
```

**典型用法**：ML / RL / 机器人类技术文章的公式推导。渲染基于 KaTeX。

---

## 标准 Markdown（全部支持）

除此之外所有标准 markdown 语法都支持：
- 标题、段落、列表、引用、代码块（带语言高亮）
- 链接（自动转 card-link 是不会的，要用上面的 ` ```cardlink ` 块）
- 图片（单独成段时触发 figure）
- **GitHub Alert**（`> [!NOTE]` 等，见第 8 节）
- **Math**（`$...$` / `$$...$$`，见第 9 节）
- 表格、引用、脚注（`[^1]` 语法由 `remark-footnotes-extra` 支持，参考 astro.config.ts）
- 任务列表 `- [x]`

---

## 速查表

| 想要的效果 | 写法 |
|---|---|
| 术语 tooltip | `*[MDP]: Markov Decision Process` |
| 给标题加 id | `## 标题 {#my-id}` |
| 给链接开新 tab | `[text](url){target=_blank}` |
| 卡片链接 | ` ```cardlink ` YAML 块 |
| 图片带 caption | `![caption](images/x.png)` 单独成段 |
| 告警提示 | `> [!NOTE]` 引用块 |
| 数学公式 | `$...$` 或 `$$...$$` |
| 剧透 | `!!隐藏内容!!` |
| 表格 | 标准 markdown，**不用**手动包 div |
