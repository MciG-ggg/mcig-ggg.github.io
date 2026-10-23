# 交互模块契约

blog 的讲义笔记需要交互。当前实现有 7 个模块：chart、slider、compare、steps、annotate、sortable、code。

每个模块一个 agent。所有人读这份文档，然后只改自己那几个文件。

## 1. 渲染模型

**模块不写 HTML 字符串。** 模块改 mdast 节点，让 markdown 流水线去渲染。

`mdast-util-to-hast` 的每个 handler 都会读节点上的 `data.hName` 和 `data.hProperties`：

- `data.hName` 换标签名。`paragraph` 节点加 `data.hName = "div"` 就渲染成 `<div>`。
- `data.hProperties` 加属性。`className` 用数组，`ariaLabel` 渲染成 `aria-label`，`dataFoo` 渲染成 `data-foo`。

这样写有三个好处：

1. 组件里的 markdown 走正常渲染。KaTeX、表格、代码高亮、图片优化都还在。
2. 图片留在 mdast 里。`remarkCollectImages` 收得到本地图片，Astro 才会复制和压缩图片。写成 HTML 字符串的图片会丢。
3. 组件里的 markdown 不进 TOC。`rehypeHeadingIds` 只收 `h1` 到 `h6`。组件里的标题节点改成 `data.hName = "p"` 就不会出现在右侧目录里。

### 共享工具

`src/utils/remark/interactive.ts` 导出 `InteractiveContext`。模块的第二个参数就是这个对象。

| 成员 | 作用 |
|---|---|
| `escapeHtml(value)` | 转义文本和属性值。用数字字符引用，不要用命名实体。 |
| `element(tag, properties?, children?)` | 造一个渲染成指定 HTML 元素的 mdast 节点。 |
| `uniqueId(prefix)` | 造一个页面内唯一的 id。同页两个同类组件拿到不同编号。 |

### 模块接口

模块文件在 `src/utils/remark/modules/<name>.ts`。

```ts
export const renderXxx = (node: ContainerDirective, context: InteractiveContext): boolean => {
	// 改 node
	return true;
};
```

返回值 `true` 表示模块接管了这个节点，流水线不再走进它的子节点。返回 `false` 表示模块没处理，流水线继续往下走。

`node.attributes` 是指令的属性对象，例如 `{type: "line", title: "标题"}`。`node.children` 是组件里的 markdown。

## 2. 文件归属

**每个 agent 只改自己那几个文件。** 别的文件一律不动。

| 模块 | 构建期 | 客户端 | 样式 | 样例 |
|---|---|---|---|---|
| chart | `src/utils/remark/modules/chart.ts` | `src/utils/interactive/chart.ts` | `src/styles/modules/chart.css` | `docs/interactive-samples/chart.md` |
| slider | `src/utils/remark/modules/slider.ts` | `src/utils/interactive/slider.ts` | `src/styles/modules/slider.css` | `docs/interactive-samples/slider.md` |
| compare | `src/utils/remark/modules/compare.ts` | `src/utils/interactive/compare.ts` | `src/styles/modules/compare.css` | `docs/interactive-samples/compare.md` |
| steps | `src/utils/remark/modules/steps.ts`、`step.ts` | `src/utils/interactive/steps.ts` | `src/styles/modules/steps.css` | `docs/interactive-samples/steps.md` |
| annotate | `src/utils/remark/modules/annotate.ts` | `src/utils/interactive/annotate.ts` | `src/styles/modules/annotate.css` | `docs/interactive-samples/annotate.md` |
| sortable | `src/utils/remark/modules/sortable.ts` | `src/utils/interactive/table.ts` | `src/styles/modules/table.css` | `docs/interactive-samples/sortable.md` |
| code | `src/utils/code-chrome.ts` | `src/utils/interactive/code.ts` | `src/styles/modules/code.css` | `docs/interactive-samples/code.md` |
| quiz | `src/utils/remark/modules/quiz.ts` | `src/utils/interactive/quiz.ts` | `src/styles/modules/quiz.css` | `docs/interactive-samples/quiz.md` |

这些文件不许改：

- `src/utils/remark/interactive.ts`、`src/utils/interactive.ts`、`src/styles/interactive.css`
- `astro.config.ts`、`src/layouts/Base.astro`
- `src/utils/remark/directive-text-guard.ts`、`src/utils/remark/escape.ts`

`src/utils/code-chrome.ts` 已经注册进 `astro.config.ts` 的 shiki transformer 列表。code 模块只改这个文件。

## 3. 通用规则

### 类名

一律 `iw-` 前缀。组件根节点用 `iw-<模块名>`，子元素用 `iw-<模块名>-<作用>`。

### 颜色

不写死颜色值。只用站点变量：`--primary-color`、`--secondary-color`、`--remark-color`、`--background-color`、`--block-color`、`--shadow-color`、`--accent-color`、`--surface-05` 到 `--surface-25`、`--radius-*`。

需要新颜色就在 `src/styles/interactive.css` 里加 `--iw-*` 变量，那个文件归文档作者管。**在自己的 CSS 文件里不要改 `:root`。**

图表的系列颜色用类名取色：`.iw-chart-series-1` 到 `.iw-chart-series-6`，颜色写在 `src/styles/modules/chart.css` 里。

### 明暗主题

站点用 `light-dark()` 加 `color-scheme`。组件只用上面的变量，就自动跟主题。不要写 `@media (prefers-color-scheme: dark)`。

### 客户端 JS

1. **事件委托到 `document`。** 不要给每个元素单独 `addEventListener`。swup 换页后 `document` 不换，委托还在。
2. **幂等。** 每次 `astro:page-load` 都会重新调用 `init<模块名>()`。每个处理过的元素打一个标记，例如 `data-iw-ready="true"`，重复调用时跳过。
3. 不引入新的 npm 依赖。
4. 客户端代码放 `src/utils/interactive/<name>.ts`，导出 `init<Name>(): void`。

### 无障碍

- 可操作的元素必须键盘可达，焦点可见。站点的 `:focus-visible` 已经有轮廓。
- 需要说明的元素给 `aria-label`。
- 纯装饰的元素给 `aria-hidden="true"`。

### 打印

每个模块在自己的 CSS 文件里写 `@media print` 规则：

- 交互控件（range 滑条、排序按钮、对比分隔线）打印时隐藏。
- 折叠内容（steps 的面板）打印时全部展开。
- 图表和图片 `break-inside: avoid`。

### 标题

组件里的 `##` 和 `###` 节点改成 `data.hName = "p"`，这样右侧目录不会收进去。

## 4. 模块规格

### chart

```
:::chart{type="line" title="吞吐随 batch 变化" unit="req/s"}
| batch | vLLM | SGLang |
|---|---|---|
| 1 | 100 | 80 |
| 8 | 700 | 520 |
:::
```

- `type`：`line`（默认）或 `bar`。
- 表格第一列是 x 值。数字就按数值画，文字就当分类标签。
- 表格其余每一列是一个系列。系列名是表头文字。
- `title` 是图标题，也是 SVG 的 `aria-label`。
- `unit` 是数值单位，显示在 y 轴标题或图例里。

输出：

- 根节点 `figure.iw-chart`。
- `svg[data-chart]`。站点 CSS 已经给 `svg[data-chart]` 设了 `fill: none`，图表几何必须带这个属性。
- `viewBox` 加 `width="100%"` 加 `height="auto"`。图片和图表一样宽。
- `role="img"` 加 `aria-label`。
- 每个数据点一个 `<g>`，里面放 `<title>`。浏览器原生提示就是它。
- 系列颜色用类名 `.iw-chart-series-1` 到 `.iw-chart-series-6`。
- 图例在 `title` 存在时显示。
- 客户端 JS 不需要。`initChart()` 保持空实现。

### slider

```
:::slider{label="温度" min="0" max="1" step="0.01" value="0.7"}
内容
:::
```

- `label` 是控件名，也是 `aria-label`。
- `min`、`max`、`step`、`value` 都有默认值：`min=0`、`max=100`、`step=1`、`value=min`。
- 组件内容在滑条下面，内容里的 markdown 正常渲染。

输出：

- 根节点 `div.iw-slider`。
- `input[type="range"].iw-slider-range`。
- 读数元素 `span.iw-slider-readout`。
- 客户端在 `input` 事件里写两个 CSS 变量到根节点：`--v` 是 0 到 1 的小数，`--value` 是原始数值。同时更新读数文字。小数位数跟 `step` 一致。
- 打印时把滑条复位到 `value`。作者设的初值才有复现性，读者拖出来的末态没有。
- 控件内图片加 `data-nozoom`。

### compare

```
:::compare{leftLabel="训练" rightLabel="推理"}
![训练时的分布](images/a.png)
![推理时的分布](images/b.png)
:::
```

- 容器里的前两张图片是左图和右图。
- `leftLabel` 和 `rightLabel` 是两角的标签。

输出：

- 根节点 `div.iw-compare`。
- 舞台 `div.iw-compare-stage`，`position: relative`。
- 右图铺满舞台。左图放在裁剪层 `div.iw-compare-clip` 里，裁剪宽度由 `--v` 控制。
- `input[type="range"].iw-compare-range` 覆盖整个舞台。range 本身透明，键盘可达。
- 分隔线 `div.iw-compare-divider`，位置由 `--v` 控制。
- 两张图片都加 `data-nozoom`。medium-zoom 会抢走点击。
- 客户端在 `input` 事件里写 `--v`。

### steps

```
::::steps{title="三步走"}
:::step{title="第一步"}
第一步的内容
:::
:::step{title="第二步"}
第二步的内容
:::
::::
```

**外层用 4 个冒号，内层用 3 个。** 3 个冒号时 micromark 会把内层当成外层的一部分，嵌套就散了。

- 纯 CSS，零 JavaScript。`initSteps()` 保持空实现。
- 用 radio 加兄弟选择器显隐。radio 的 `name` 用 `context.uniqueId("iw-steps")`，同页两个 steps 组件的 radio 组不能重名。

输出：

- 根节点 `div.iw-steps`。
- N 个 `input[type="radio"]`，第一个 `checked`。`id` 用 `uniqueId`。
- 标签栏 `div.iw-steps-tabs`，N 个 `label`，`for` 指向对应 radio。`title` 是组标题，`step` 的 `title` 是单步标题。
- 面板 `div.iw-step`，N 个。
- CSS 用 `:has()` 加 `:nth-of-type` 控制显隐。**CSS 里写 1 到 12 的规则**，超过 12 步不显示，样例里说明这个上限。
- 打印时全部展开。

### annotate

```
:::annotate{title="前向传播的数据流"}
![结构图](images/x.png)

:pin{x="23" y="41" label="QKV 投影"}
:pin{x="67" y="55" label="注意力输出"}
:::
```

- 容器里的第一张图片是底图。
- `:pin{...}` 是文本指令。`x` 和 `y` 是百分比，0 到 100。`label` 是气泡文字。
- `title` 是图标题。

输出：

- 根节点 `figure.iw-annotate`。
- 舞台 `div.iw-annotate-stage`，`position: relative`。
- 每个 pin 一个 `button.iw-pin`，绝对定位，用 `--x` 和 `--y` 定位。`button` 键盘可达。
- 气泡用 CSS 的 `:hover` 和 `:focus-visible` 展开。气泡文字写进 `data-label`，CSS 用 `content: attr(data-label)`。
- 底图加 `data-nozoom`。
- 客户端 JS 不需要。`initAnnotate()` 保持空实现。

### sortable

```
:::sortable
| 名称 | 延迟 |
|---|---|
| A | 12 |
| B | 30 |
:::
```

- 组件包住一张表格。表格本身照常渲染。
- **只有数值列加排序按钮。** 数值列的判断是自动的：一列里去掉空值后全是数字，就算数值列。
- 客户端给表头加一个按钮。点击后按该列排序 DOM 行，不重新渲染表格。
- 按钮要有 `aria-label`，写清按哪一列排序。
- 打印时隐藏排序按钮。
- 表格在窄屏要能横向滚动。站点的 `div.table-wrapper` 已经处理了。

### code

code 模块不是指令。它作用于所有围栏代码块。

- 实现是 shiki transformer，文件 `src/utils/code-chrome.ts`。
- **语言标签。** 代码块左上角显示语言名。Astro 的 shiki transformer 已经给 `<pre>` 写了 `data-language`，CSS 可以用 `pre::before { content: attr(data-language) }`。标签不能跟着代码横向滚动。
- **行高亮。** 围栏 meta 写行号：

  ```` ```python {1,3} ````

  高亮第 1 行和第 3 行。meta 的原始字符串在 `this.options.meta.__raw`。
- **不要动 mermaid。** `pre[data-language="mermaid"]` 是图表，图表自己渲染。语言标签和行高亮都要跳过 mermaid。
- **不要动复制按钮。** `src/utils/code-copy.ts` 的 transformer 已经在做复制按钮。两个 transformer 的顺序是 `copy` 在前、chrome 在后。chrome 的 `root` hook 拿到的节点里，`<pre>` 已经包在 `div.code-container` 里。

shiki 的 hook：

| hook | 参数 | 说明 |
|---|---|---|
| `root` | 根节点 | 根节点的 `children[0]` 是 `<pre>`，或者 `copy` 造出来的 `div.code-container`。 |
| `pre` | `<pre>` 元素 | `properties.dataLanguage` 是语言名。 |
| `code` | `<code>` 元素 | |
| `line` | 行元素 | 第二个参数是行号，从 1 开始。 |

### quiz

一组题，每道题带一个「显示答案」的按钮。

````markdown
::::quiz{title="第一卷"}
:::q{type="选择" points="2"}
题干

1. 选项一
2. 选项二

---

答案与解析
:::

:::q{type="程序" points="14"}
程序题的题干

---

```python
print("答案")
```

:::
::::
````

- **外层用 4 个冒号，内层用 3 个。** 外层的冒号要比内层多一个，否则内层指令会把外层提前关掉。
- `type` 是题型，显示在题头的徽标上：选择、填空、程序、解答。
- `points` 是分值，显示在题型后面。
- **答案和题干之间用一个 `---` 分隔。** `---` 前后都要空一行，否则 `---` 会被当成上一段的标题下划线。
- 没有 `---` 的题目没有答案，按钮也不出现。

输出：

- 根节点 `div.iw-quiz`，带标题栏和一个「全部显示答案」按钮。
- 每道题是 `div.iw-quiz-q`，带 `data-type` 和 `data-points`。
- 题头是 `div.iw-quiz-head`，里面是题型徽标、分值和「显示答案」按钮。
- 答案是 `div.iw-quiz-answer`，带 `hidden` 属性，按钮的 `aria-controls` 指向它的 id。
- 客户端在 `document` 上委托点击：单题按钮切换自己的答案，整卷按钮切换这一卷的所有答案。

## 5. 自测

模块写完后跑这个命令。它走站点自己的 markdown 配置，输出 HTML。

```bash
pnpm exec tsx scripts/test-interactive.ts docs/interactive-samples/<name>.md
```

看输出的 HTML，确认：

1. 组件的结构对。
2. 组件里的 markdown 正常渲染（表格、公式、代码、图片都在）。
3. 图片的 `src` 没有丢。

样例文件写在 `docs/interactive-samples/<name>.md`。样例要覆盖模块的所有属性。

不要跑 `pnpm build`。多个 agent 同时跑会互相覆盖 `dist/`。构建由集成 agent 跑一次。

## 6. 交付清单

- [ ] 只改了自己那几个文件
- [ ] `pnpm exec tsx scripts/test-interactive.ts docs/interactive-samples/<name>.md` 的输出符合规格
- [ ] CSS 只用站点变量，没有写死颜色
- [ ] 明暗主题都成立
- [ ] 客户端 JS 幂等，事件委托到 `document`
- [ ] 键盘可达，有 `aria-label`
- [ ] 自己的 CSS 文件里有 `@media print` 规则
- [ ] 组件里的标题节点改成了 `data.hName = "p"`
- [ ] 注释和标识符用英文，用户可见文字用中文
- [ ] 代码风格：tab 缩进，双引号，分号，行宽 150
