# compare 样例

`compare` 把两张图叠在一起，拖动中间的滑块切换。

## 完整属性

:::compare{leftLabel="训练" rightLabel="推理"}
![训练时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
![推理时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
:::

## 只有左标签

:::compare{leftLabel="训练"}
![训练时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
![推理时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
:::

## 没有标签

:::compare
![训练时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
![推理时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
:::

## 组件里的 markdown

组件里的表格、公式和代码照常渲染。组件里的标题不进右侧目录。

:::compare{leftLabel="左" rightLabel="右"}
![训练时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
![推理时的分布](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)

### 组件里的标题

这个标题渲染成段落，不进目录。

| 方法 | 精度 | 耗时 |
|---|---|---|
| 遗传算法 | 0.91 | 12 ms |
| 粒子群 | 0.88 | 9 ms |

$$
f(x) = \sum_{i=1}^{n} x_i
$$

```python
print("compare")
```
:::
