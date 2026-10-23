# steps

`steps` 把一段讲解分成几步。读者点上面的标签切换步骤，每一步的内容是一段普通 Markdown。

## 用法

外层用 4 个冒号，内层用 3 个冒号。

::::steps{title="自动控制原理的一个例子"}
:::step{title="第一步：建模"}
把电机和负载写成一个二阶系统。

$$
m\ddot{\theta} + b\dot{\theta} + ku = e(t)
$$

| 符号 | 含义 | 单位 |
| --- | --- | --- |
| $m$ | 转动惯量 | $\mathrm{kg\cdot m^2}$ |
| $b$ | 阻尼系数 | $\mathrm{N\cdot m\cdot s/rad}$ |

```python
def step(theta, u, m, b, k, dt):
    acc = (u - b * theta["omega"] - k * theta["angle"]) / m
    return {"angle": theta["angle"] + theta["omega"] * dt, "omega": theta["omega"] + acc * dt}
```

![系统框图](../../src/content/lecture/zh-cn/计算智能_进化算法与优化方法/images/fig_001.png)
:::
:::step{title="第二步：设计控制器"}
用状态反馈把极点放到 $-\zeta \pm j\omega_d$。

- 期望阻尼比 $\zeta = 0.7$
- 期望自然频率 $\omega_d = 10\ \mathrm{rad/s}$

### 这一步里的标题不进目录

组件里的 `##` 和 `###` 渲染成普通段落，所以右侧目录不收它们。
:::
:::step{title="第三步：验证"}
在阶跃输入下看响应。
:::
::::

同页第二个 `steps` 的 radio 组换了一个名字，两个组件互不干扰。`step` 不写 `title` 时，标签上写“第 N 步”。

::::steps{title="自动驾驶的一个例子"}
:::step{title="第一步：采集"}
摄像头和激光雷达各出一路数据。
:::
:::step
用卡尔曼滤波融合两路数据。
:::
::::

## 步骤上限

样式表只写了 1 到 12 的规则，所以一个 `steps` 最多显示 12 步。第 13 步起的标签和面板都不显示。

## 属性

| 指令 | 属性 | 说明 |
| --- | --- | --- |
| `steps` | `title` | 整组步骤的标题 |
| `step` | `title` | 一步的标题，也是标签上的文字 |
