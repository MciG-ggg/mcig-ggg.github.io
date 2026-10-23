# Slider

滑条把一个数交给读者。内容在滑条下面，读者的选择通过 CSS 变量 `--v`（0 到 1 的位置）和 `--value`（原始数值）传给组件里的样式。

## 全部属性

:::slider{label="温度" min="0" max="1" step="0.01" value="0.7"}
内容在滑条下面，里面的 markdown 照常渲染。

### 组件里的标题

组件里的标题不进右侧目录。

| 温度 | 状态 |
| --- | --- |
| 0.20 | 低温 |
| 0.70 | 适中 |
| 0.95 | 高温 |

$$
T = 20 + 25 v
$$

```python
def temperature(v):
    return 20 + 25 * v
```

![温度与体感](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
:::

## 只有 label

没有 `min`、`max`、`step`、`value` 时，取默认值 `min=0`、`max=100`、`step=1`、`value=min`。

:::slider{label="权重"}
默认值是 0 到 100，步长 1，初始值 0。
:::

## 整数步长

`step="1"` 时读数不带小数位。

:::slider{label="迭代次数" min="1" max="20" step="1" value="5"}
迭代次数从 1 到 20。
:::

## 步长带两位小数

`step="0.01"` 时读数保留两位小数，初始值 0.7 写成 0.70。

:::slider{label="增益" min="0" max="2" step="0.01" value="0.7"}
增益范围 0 到 2。
:::
