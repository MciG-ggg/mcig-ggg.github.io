# annotate

`:::annotate` 把一张图和若干个标注点放在同一个舞台里。标注点是按钮，鼠标悬停或键盘聚焦时弹出气泡。

```
:::annotate{title="前向传播的数据流"}
![结构图](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)

:pin{x="23" y="41" label="QKV 投影"}
:pin{x="67" y="55" label="注意力输出"}
:::
```

## 属性

| 属性 | 说明 | 默认值 |
|---|---|---|
| `title` | 图标题，显示在图下方 | 无标题 |
| `x` | 标注点的横坐标，百分比，0 到 100 | 必填 |
| `y` | 标注点的纵坐标，百分比，0 到 100 | 必填 |
| `label` | 气泡里的文字 | 空 |

## 示例

:::annotate{title="前向传播的数据流"}
![结构图](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)

:pin{x="23" y="41" label="QKV 投影"}
:pin{x="67" y="55" label="注意力输出"}
:pin{x="45" y="78" label="残差连接"}
:::

坐标超出 0 到 100 时会被截断到边界。缺少 `x` 或 `y` 的标注点会被丢掉。

## 没有标题

`title` 是可选的。没有标题时，舞台下面不出现说明文字。

:::annotate
![结构图](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)

:pin{x="50" y="30" label="编码器"}
:pin{x="50" y="70" label="解码器"}
:::
