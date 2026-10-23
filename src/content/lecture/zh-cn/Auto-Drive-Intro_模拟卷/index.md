---
title: Auto-Drive-Intro 模拟卷
timestamp: 2026-10-23T15:05:00+08:00
series: Auto-Drive-Intro
tags: [课内, Auto-Drive-Intro]
description: 《智能驾驶导论》期末模拟卷三套，每套一百分，含选择、填空、程序、解答四类题。答案藏在每道题的按钮后面，可以单题展开，也可以整卷展开。
source_url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/
toc: true
draft: false
---

```cardlink
url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/
title: "Auto-Drive-Intro · 课程课件与作业"
description: "《智能驾驶导论》的六讲课件、感知基础讲义与四次 APOLLO 平台作业"
host: 10.188.31.55:5244
```

三套卷，每套一百分，题型是选择、填空、程序、解答。**答案在每道题右上角的「显示答案」按钮后面。** 做完一整卷再一次性展开对答案，也可以一题一题看。

配套复习资料在[《Auto-Drive-Intro 代码题复习》](/lecture/Auto-Drive-Intro_代码题复习)。三套卷的取材范围分别是感知与传统方法、Apollo 工具链与规划、控制与参数。

| 卷 | 范围 | 选择 | 填空 | 程序 | 解答 | 合计 |
|---|---|---|---|---|---|---|
| 第一卷 | 感知与传统方法 | 10 分 | 12 分 | 28 分 | 50 分 | 100 分 |
| 第二卷 | Apollo 工具链与规划 | 10 分 | 12 分 | 28 分 | 50 分 | 100 分 |
| 第三卷 | 控制与参数 | 10 分 | 12 分 | 28 分 | 50 分 | 100 分 |

## 第一卷 · 感知与传统方法

### 答题卡

| 题号 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 得分 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 类型 | 选 | 选 | 选 | 选 | 选 | 填 | 填 | 填 | 程 | 程 | 解 | 解 | |
| 满分 | 2 | 2 | 2 | 2 | 2 | 4 | 4 | 4 | 14 | 14 | 25 | 25 | 100 |

### 一、单项选择题

::::quiz{title="第一卷"}
:::q{type="选择" points="2" answer="2"}
`cv2.imread` 读进来的彩色图像，通道顺序是：

1. RGB
2. BGR
3. GBR
4. YUV

（本章取材自 [第二章 无人驾驶感知基础.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=10)）

---

**答案：2（BGR）**

OpenCV 的默认通道序是蓝绿红。转灰度必须写 `cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)`，写成 `RGB2GRAY` 得到的灰度图红蓝互换，边缘位置会偏。

:::

:::q{type="选择" points="2" answer="3"}
Canny 边缘检测的三个步骤中，负责把边缘压成一像素宽的是：

1. Sobel 梯度计算
2. 高斯平滑
3. 非极大值抑制
4. 双阈值滞后

（见 [第二章 无人驾驶感知基础.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=10)）

---

**答案：3（非极大值抑制）**

Sobel 给出梯度幅值和方向，非极大值抑制沿梯度方向只保留最大的那个像素，双阈值滞后决定哪些弱边缘能连上强边缘。

:::

:::q{type="选择" points="2" answer="2"}
在图像坐标系里（y 轴向下），左侧车道线的斜率符号是：

1. 正
2. 负
3. 零
4. 随机

（见 [第二章 无人驾驶感知基础.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=14)）

---

**答案：2（负）**

左车道线从画面左下走向右上，$x$ 增大时 $y$ 减小，斜率 $k = \Delta y / \Delta x < 0$。所以代码里判左线用 `slope < -MIN_ABS_SLOPE`。

:::

:::q{type="选择" points="2" answer="2"}
毫米波雷达测量目标径向速度的原理是：

1. 飞行时间
2. 多普勒效应
3. 相位差
4. 三角测量

---

**答案：2（多普勒效应）**

目标相对雷达有速度时，回波频率会偏移，测出频率差就得到径向速度。飞行时间是激光雷达的测距原理。

:::

:::q{type="选择" points="2" answer="2"}
在概率霍夫变换里，把 `threshold` 调大会导致：

1. 检出的线段变多
2. 检出的线段变少
3. 线段长度变长
4. 断裂处被连起来

---

**答案：2（检出的线段变少）**

`threshold` 是投票门槛。票数要达到门槛才算一条线，门槛提高后只有足够长的共线点集才能通过，线段变少，远处弱车道线容易漏检。

:::

### 二、填空题

:::q{type="填空" points="4" answer="弱边缘候选的门槛|强边缘的门槛|1:3"}
Canny 的双阈值在代码里写成 `CANNY_LOW, CANNY_HIGH = 50, 150`。低阈值是 {{blank}}，高阈值是 {{blank}}，两者的数值关系通常取 {{blank}}。

---

**答案：高阈值是强边缘的门槛，低阈值是弱边缘候选的门槛，弱边缘还要连着强边缘才被保留（滞后判定）。两者通常取 1:3。**

代码里 50 和 150 正好 1:3。调高高阈值会让弱边缘被提升为强边缘，噪点变多；调低则只剩最强边缘，噪点少但可能漏掉对比度低的车道线。

:::

:::q{type="填空" points="4" answer="累加器投票门槛|线段的最短像素长度|允许的最大断裂"}
概率霍夫变换的三个参数 `HOUGH_THRESHOLD`、`HOUGH_MIN_LEN`、`HOUGH_MAX_GAP` 分别是 30、20、20。`HOUGH_THRESHOLD` 是 {{blank}}，`HOUGH_MIN_LEN` 是 {{blank}}，`HOUGH_MAX_GAP` 是 {{blank}}。（见 [第二章 无人驾驶感知基础.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=12)）

---

**答案：`HOUGH_THRESHOLD` 是累加器投票门槛，一条直线上要有足够多的点投票才算一条线；`HOUGH_MIN_LEN` 是线段的最短像素长度，短于此的线段丢弃；`HOUGH_MAX_GAP` 是允许的最大断裂，相距不超过这个像素数的两个点算同一条线上的。**

调大 `HOUGH_MAX_GAP` 有风险：斜率本来不同的两组点可能被连成一条线。

:::

:::q{type="填空" points="4" answer="稀疏|无序"}
点云有四个特点，请写出其中三个：简单、精确、{{blank}} 和 {{blank}}。

---

**答案：稀疏、无序。**

点云就是 $n \times 3$ 的矩阵（$x, y, z$，有的还带强度 $i$），几何上点的顺序不影响它对整体形状的表示，所以是无序的；激光雷达只在表面上打点，所以是稀疏的。点与点之间的关联性、刚性变换下的旋转性也是常考的两条。

:::

### 三、程序题

:::q{type="程序" points="14"}
写出从 BGR 原图提取边缘的完整三行代码，并说明每行做了什么。

---

```python
gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
edges = cv2.Canny(blurred, 50, 150)
```

第一行把三通道转成单通道，OpenCV 的通道序是 BGR 所以用 `COLOR_BGR2GRAY`。第二行是 5×5 高斯平滑，sigma 填 0 表示由核大小自动推算（$\sigma \approx 1.1$），作用是把单像素噪声摊开而不移动边缘位置。第三行是 Canny：Sobel 梯度 → 非极大值抑制 → 双阈值滞后，输出的是只有 0 和 255 的二值边缘图。

:::

:::q{type="程序" points="14"}
写出「按斜率把霍夫线段分成左右两组，各组取斜率与截距均值」的代码片段，要求跳过竖直线和斜率绝对值小于 0.5 的水平噪声线。

---

```python
MIN_ABS_SLOPE = 0.5
left_fit, right_fit = [], []
for x1, y1, x2, y2 in lines.reshape(-1, 4):
    if x1 == x2:
        continue
    slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)
    if slope < -MIN_ABS_SLOPE:
        left_fit.append((slope, intercept))
    elif slope > MIN_ABS_SLOPE:
        right_fit.append((slope, intercept))

left_slope, left_intercept = np.average(left_fit, axis=0)
right_slope, right_intercept = np.average(right_fit, axis=0)
```

`HoughLinesP` 的返回形状是 $(N, 1, 4)$，要 `reshape(-1, 4)` 压成 $(N, 4)$。`x1 == x2` 是竖直线，除以斜率会除零，必须先跳。图像 y 轴向下，左线斜率为负、右线为正，所以用两个严格不等号把中间那段斜率小的留给噪声线。`np.average(fit, axis=0)` 对每列（斜率、截距）分别取平均。

:::

### 四、解答题

:::q{type="解答" points="25"}
这套传统车道线检测方法在真实场景里不够用。请至少写出三条原因，并说明每条原因对应流水线上的哪一步。（课件用 LaneNet 这类深度学习方法做对比，见 [第二章 无人驾驶感知基础.pdf · p.25](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=25)）

---

**三条原因，对应三步：**

**一、假设车道线是直线。** 对应第五步「按斜率分组后取斜率截距均值」。这一步把每条车道线压成一条直线 $y = kx + b$，弯道上真实车道线是曲线，用直线去拟合必然偏出去，弯道越急偏得越多。用曲线拟合（多项式）或者样条能缓解，但成本上去了。

**二、依赖手工指定的 ROI。** 对应第三步「三角掩码」。掩码的顶点在宽度的 45%、高度的 60% 处，这个位置是给某几张特定图片手调的。换一个俯仰角、换一个坡道、换一个分辨率，三角形就罩不住真正的路面了，漏检和误检同时出现。深度学习方法的 ROI 是学出来的，不用手调。

**三、只用灰度信息，丢掉了颜色和纹理。** 对应第一步「灰度」。车道线在图上靠的是和路面的亮度差，但有的路面标线和沥青亮度接近，靠颜色区分。灰度化之后这些线索全没了，只能靠 Canny 的边缘去碰运气。

**补充一条：** 整条流水线是逐帧独立处理的，没有时序信息。抖动、遮挡、天气变化都没有平滑机制。深度学习车道线检测用时序信息做稳定，这是传统方法结构上给不了的。

:::

:::q{type="解答" points="25"}
详细说明 Canny 边缘检测的三个步骤，并解释双阈值滞后这一机制为什么能抑制噪声。

---

**第一步：Sobel 梯度。** 用两个 3×3 卷积核分别算水平和竖直方向的偏导，得到 $G_x$ 和 $G_y$，合成梯度幅值 $G = \sqrt{G_x^2 + G_y^2}$ 和方向 $\theta = \arctan2(G_y, G_x)$。

**第二步：非极大值抑制。** 沿梯度方向取两个相邻像素，只保留梯度最大的那个，其余置零。梯度方向上的两个邻居是沿边缘切线方向取的，所以这一步把宽边缘压成一像素宽，同时把噪声点压掉大半。

**第三步：双阈值滞后。** 梯度大于 `CANNY_HIGH` 的像素直接判为边缘；梯度在 `CANNY_LOW` 和 `CANNY_HIGH` 之间的判为候选；小于 `CANNY_LOW` 的丢弃。然后做连通性检查：候选像素只要和强边缘八连通，就算边缘；否则丢掉。

**为什么滞后能抑制噪声。** 单独的弱边缘可能是噪声，随手一个高阈值会把它误当边缘；单独一个低阈值会把大片噪声拉进来。滞后用的是「弱证据要依附强证据」这条规则：噪声点几乎不可能刚好和一群强边缘连成一片，所以孤立的弱边缘被丢掉；而真实的弱边缘（车道线磨损、被树影遮了一段、远处对比度低）虽然弱，但它连着强边缘那部分，会被整条保留下来。阈值比取 1:3 是在「保住弱边缘」和「压住噪声」之间取平衡。

:::
::::

## 第二卷 · Apollo 工具链与规划

### 答题卡

| 题号 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 得分 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 类型 | 选 | 选 | 选 | 选 | 选 | 填 | 填 | 填 | 程 | 程 | 解 | 解 | |
| 满分 | 2 | 2 | 2 | 2 | 2 | 4 | 4 | 4 | 14 | 14 | 25 | 25 | 100 |

### 一、单项选择题

::::quiz{title="第二卷"}
:::q{type="选择" points="2" answer="2"}
启动 Apollo 的 Dreamview 交互界面，正确的命令是：

1. `bash scripts/dreamview.sh`
2. `bash scripts/bootstrap_neo.sh`
3. `apollo start dreamview`
4. `python3 dreamview.py`

（见 [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx)）

---

**答案：2（`bash scripts/bootstrap_neo.sh`）**

这条命令一键拉起整套 Apollo 环境，包括 cyber 和 dreamview。终端出现 `Dreamview is running at http://localhost:8888` 就说明 web 端起来了。

:::

:::q{type="选择" points="2" answer="1"}
回放一个离线数据包并让它循环播放的命令是：

1. `cyber_recorder play -f demo_3.5.record -l`
2. `cyber_recorder info -f demo_3.5.record`
3. `cyber_monitor --loop demo_3.5.record`
4. `apollo replay demo_3.5.record -l`

---

**答案：1（`cyber_recorder play -f demo_3.5.record -l`）**

`-f` 指文件，`-l` 是 loop 循环。`cyber_recorder list` 看包里有哪些通道，`cyber_recorder info` 看包的信息。

:::

:::q{type="选择" points="2" answer="3"}
要做开放空间规划（泊车），Dreamview 的 Module Controller 里需要启动的模块是：

1. 只启 Planning
2. 启 Planning 和 Prediction
3. 启 Planning、Prediction、Routing、TaskManager
4. 启 Planning、Prediction、Routing、Localization

（见 [作业二：基于百度apollo平台的多场景下路径规划.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx)）

---

**答案：3（Planning、Prediction、Routing、TaskManager）**

路径规划只需要 Planning、Prediction、Routing。开放空间规划是路径规划的特例，多出来的任务调度由 TaskManager 负责，所以要多启它。

:::

:::q{type="选择" points="2" answer="2"}
在 ST 图（横轴 $s$ 纵轴 $t$）上，一辆车以速度 $v$ 匀速行驶，它画出来的线斜率是：

1. $v$
2. $1/v$
3. $v^2$
4. 与 $v$ 无关

---

**答案：2（$1/v$）**

斜率是 $\frac{dt}{ds}$。走同样的距离，车越快用时越少，所以斜率与速度成反比。车越快，这条线在图上越平。

:::

:::q{type="选择" points="2" answer="2"}
cyber_monitor 和 PNC Monitor 的区别是：

1. 前者看曲线，后者看数据流
2. 前者看数据流，后者看曲线
3. 两者完全一样
4. 前者只能离线用，后者只能在线用

---

**答案：2（前者看数据流，后者看曲线）**

cyber_monitor 看的是 topic 上的消息内容本身，PNC Monitor 看的是某个物理量随时间变化的曲线。作业三要读的 record time 就是从 cyber_monitor 里拿的。

:::

### 二、填空题

:::q{type="填空" points="4" answer="Module Controller|Profile|Layer Menu|Tasks"}
Dreamview 左侧有四个常用的 Tab：{{blank}}、{{blank}}、{{blank}}、{{blank}}。做「路径规划实践」时要用到 Module Controller 和 Profile 两个。

---

**答案：Module Controller、Profile、Layer Menu、Tasks 四个。**

做路径规划实践要用 **Module Controller**（启停 Planning、Prediction、Routing）和 **Profile**（选 Scenario Profiles 里的 course 场景集）。Layer Menu 用来切视角，Point ofView 选 Map 是俯视图。Tasks 用来展开 PNC Monitor，得先关掉 Lock Task Panel。

:::

:::q{type="填空" points="4" answer="绕行障碍物|借道绕行"}
`course` 场景集里的四个典型场景是：自车道内行驶、{{blank}}、{{blank}} 和换道。

---

**答案：绕行障碍物、借道绕行。**

绕行障碍物是车道内横向偏移；借道绕行是跨到相邻车道去绕。两者都发生在车道内到车道间，但借道要占用相邻车道的通行权。

:::

:::q{type="填空" points="4" answer="planning.INFO 日志文件|时刻（时:分:秒）|DATA RECORDER"}
取某一时刻的速度规划数据，要跑一个脚本：

```bash
run modules/planning/tools/plot_st_nlp.py -f /opt/apollo/neo/data/log/planning.INFO -t 17:19:29
```

这个脚本的参数 `-f` 指向 {{blank}}，`-t` 是 {{blank}}，这个时刻从 {{blank}} 里读。（见 [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx)）

---

**答案：`-f` 指向 planning.INFO 日志文件，`-t` 是时刻（时:分:秒）。**

时刻从 DATA RECORDER 里读，作业三的原文是「时间可通过 DATA RECORDER 获取」。跑起来是个 jupyter notebook，会把 ST 图、DP 网格、决策结果和对应的 SL 曲线都画出来。

:::

### 三、程序题

:::q{type="程序" points="14"}
写出完成下面三件事的命令：启动 Dreamview、回放一个循环的离线数据包、取出某一时刻的速度规划数据。

---

```bash
# 一键启动 Dreamview
bash scripts/bootstrap_neo.sh

# 回放离线数据包，-l 循环
cyber_recorder play -f demo_3.5.record -l

# 取某一时刻的速度规划数据
run modules/planning/tools/plot_st_nlp.py \
    -f /opt/apollo/neo/data/log/planning.INFO \
    -t 17:19:29
```

回放和取数不是同一个进程：回放时 planning 模块会往 `data/log/planning.INFO` 里写日志，`plot_st_nlp.py` 是事后从这个日志里按时刻把 ST 图和 NLP 结果读出来画图。

:::

:::q{type="程序" points="14"}
用伪代码写出「根据 ST 图决定超车还是让行」的判断逻辑，并说明 DP 的代价函数由哪几项组成。

---

```
# 输入：ST 图上的上边界、障碍物投影线集合、DP 网格
# 输出：决策 OVERTAKE 或 YIELD，以及一条 SL 曲线

1. 在 ST 图上构造可行域：
   上界 = 速度限制给出的时间下界
   下界 = 每条障碍物投影线的上包络
   可行域 = 上下界之间且不在任何障碍物线下方
2. 在可行域上跑 DP，从起点格搜到终点格，代价取最小
3. 读出这条 DP 路径：
   若路径存在一段位于障碍物线的上方 -> 决策 OVERTAKE
   若路径只能全程位于障碍物线的下方 -> 决策 YIELD
4. 把 ST 图上的路径反投影回 SL 图，得到最终的 SL 曲线
```

DP 的代价函数大致是三项之和：**障碍物代价**（离障碍物投影线多近）、**横向代价**（离参考线多远）、**纵向代价**（加速度和加速度变化率有多大）。前两项保证不撞、不错位，第三项保证开得平稳。

:::

### 四、解答题

:::q{type="解答" points="25"}
完整说明 S-T 图的构成：横轴纵轴各是什么，一条障碍物在图上画成什么样，上下边界从哪来，可行域是什么，决策又从哪来。

---

**坐标系。** 横轴是纵向距离 $s$，纵轴是时间 $t$。速度规划的问题本来是「在什么位置、什么时刻、什么速度」，换到 ST 空间之后，「什么时候到某个位置」变成了一个点，速度变成了那条线的斜率。

**障碍物的投影。** 前车以速度 $v$ 匀速行驶时，它在 ST 图上是一条直线，斜率 $\frac{dt}{ds} = \frac{1}{v}$。前车匀减速时是向下凹的曲线，加速时向上凸。车越快，线越平；车越慢，线越陡——所以图上一条陡的线代表一辆慢车，它会横穿本车的时间轴。

**上边界。** 由速度限制给出。同一段路上限速 $v_{max}$，那么通过这一段最快也要花 $\frac{\Delta s}{v_{max}}$ 的时间，这画出了时间下界。限速分段变化时，上边界是阶梯状的。

**下边界。** 由障碍物投影线给出。本车不能撞上障碍物，所以它在 ST 图上必须待在障碍物线的上方（更晚到达同一个位置）。

**可行域。** 上下边界之间、且在所有障碍物线之上的那块区域。落在可行域里的点，都是「存在某个时刻、某个位置、本车可以合法通过」。

**决策。** 决策来自可行域的连通性。如果存在一条从起点到终点的路径能绕到障碍物线上方（先加速拉开纵向距离，再变道超过），那就是 OVERTAKE；如果可行域被障碍物线切断，只能待在它下方跟车减速，就是 YIELD。

**DP 的作用。** 在 ST 网格上做动态规划，从起点格搜到终点格，路径代价取最小，同时记下回溯路径。这条路径反投影回 SL 空间就是最终执行的 SL 曲线。

:::
:::

:::q{type="解答" points="25"}
说明 Planning 模块的三个子模块各决定什么，它们之间是什么关系，为什么不能完全并行。（见 [第2讲 无人车基本框架.pptx · p.45](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx)）

---

**三个子模块。**

**路径规划**决定走哪条线，输出的是几何形状：路径的曲线、路径边界（车辆宽度的余量）。它关心的是空间上通不通得过。

**速度规划**决定沿这条线走多快，输出的是时间维度的信息：每个位置上的目标速度、由此生成的轨迹点时间。它关心的是这条线能不能安全地按时走完。

**开发空间规划**是路径规划在没有车道线时的特例，输出自车轨迹与库位、障碍物的相对关系。

**关系。**

有先后：路径先定几何，速度再沿这条几何定时间。速度规划必须知道路径的形状才能算 Frenet 坐标下的 $s$、$d$ 和曲率。

也有耦合：路径规划需要避障，速度规划也要避障。速度规划给出的「这条路上哪里快、哪里慢」会影响路径规划要不要绕行；路径规划改了几何，速度规划要重算。Apollo 的做法是交替优化：DPQ 先出路径，QP 在路径给定的情况下出速度解，再把速度解反馈给路径规划迭代若干轮。

不能完全并行是因为速度规划依赖路径的结果。真正能并行的是同一层内的多个候选方案——比如同时算几条候选路径各自的 ST 图再选。

**场景上的分工。** 绕行障碍物、借道绕行、换道主要是路径规划的活；加速超车、减速让行、掉头、减速带主要是速度规划的活；泊车是开发空间规划的活。看一个场景属于哪一类，决定了调哪个模块的参数。

:::
::::

## 第三卷 · 控制与参数

### 答题卡

| 题号 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 | 得分 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 类型 | 选 | 选 | 选 | 选 | 选 | 填 | 填 | 填 | 程 | 程 | 解 | 解 | |
| 满分 | 2 | 2 | 2 | 2 | 2 | 4 | 4 | 4 | 14 | 14 | 25 | 25 | 100 |

### 一、单项选择题

::::quiz{title="第三卷"}
:::q{type="选择" points="2" answer="1"}
横向 LQR 的代价函数是 $J = q_0 e_y^2 + q_1 \dot e_y^2 + q_2 e_\psi^2 + q_3 \dot e_\psi^2$。四个权重分别对应：（见 [作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx)）

1. 横向位置误差、横向位置误差变化率、航向误差、航向误差变化率
2. 航向误差、航向误差变化率、横向位置误差、横向位置误差变化率
3. 横向位置误差、航向误差、横向位置误差变化率、航向误差变化率
4. 速度误差、加速度误差、航向误差、航向误差变化率

---

**答案：1**

$e_y$ 是横向位置误差，$\dot e_y$ 是它的变化率，$e_\psi$ 是航向误差，$\dot e_\psi$ 是航向误差变化率。

:::

:::q{type="选择" points="2" answer="2"}
作业里的横向控制参数是 `matrix q` 取 0.05、0.0、1.0、0.0。这说明：

1. 横向位置误差的影响最大
2. 航向误差的影响最大，横向位置误差的影响很小
3. 四个权重一样大
4. 这是一组无效参数

---

**答案：2**

$q_2 = 1.0$ 是 $q_0 = 0.05$ 的二十倍。横向控制主要靠航向误差驱动，横向位置误差的权重很小。

:::

:::q{type="选择" points="2" answer="2"}
纵向控制里设置前视（lookahead）的主要目的是：

1. 让车看得更远
2. 减小跟踪滞后
3. 提高限速
4. 减少传感器数量

---

**答案：2（减小跟踪滞后）**

纯跟踪用当前位置算误差，车已经走过头了才纠回来，必然有滞后。用前方一小段路做参考点，误差提前产生，滞后就小了。

:::

:::q{type="选择" points="2" answer="2"}
把概率霍夫变换的 `HOUGH_MAX_GAP` 调得太大，最大的风险是：

1. 计算变慢
2. 斜率不同的两组点被连成一条线
3. 图像变模糊
4. 线段长度变短

---

**答案：2**

断裂 allowance 太大时，护栏边缘和相邻车道线这类本来不共线的点会被连起来，斜率均值被拉歪。

:::

:::q{type="选择" points="2" answer="1"}
判断横向控制效果好不好，最先看的曲线是：

1. `cross_track_error`
2. `planning speed`
3. `curvature`
4. `station_error`

---

**答案：1（`cross_track_error`）**

横向偏差是横向控制的第一指标。它在零线附近小幅波动说明控制正常；持续偏移说明反馈权重太小或者存在稳态误差；高频抖动说明 $q_1$、$q_3$ 太小或 $k_d$ 太大。

:::

### 二、填空题

:::q{type="填空" points="4" answer="apollo_workspace/modules/control/conf/control_conf.pb.txt|lat_controller_conf"}
作业四给出的两个配置文件路径是什么？放纵向控制参数的是 {{blank}}，放横向控制参数的在 {{blank}} 里。（见 [作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx)）

---

**答案：纵向在 `apollo_workspace/modules/control/conf/control_conf.pb.txt`，横向在 `lat_controller_conf` 里。**

`lat_controller_conf` 是同一个目录下的横向控制器配置，里面的 `matrix q` 是 LQR 的四个权重，`reverse_matrix q` 是倒车时用的另一组。

:::

:::q{type="填空" points="4" answer="control_info|cd modules/tools/control_info/|jupyter notebook"}
控制误差曲线用 {{blank}} 工具画，启动它要 {{blank}}，然后 {{blank}}。

---

**答案：工具是 `control_info`。命令是 `cd modules/tools/control_info/` 然后 `jupyter notebook`。**

它画出的曲线包括 `cross_track_error`、`heading_error`、`heading_rate_error`、`curvature_error`、`curvature_rate_error`、`left_line_error`、`right_line_error`、`line_heading_error`、`line_slope_error`。

:::

:::q{type="填空" points="4" answer="kp|kd"}
纵向 PID 里，{{blank}} 调大会让速度曲线出现超调和振荡；{{blank}} 调大能抑制超调，但会放大噪声。

---

**答案：`kp` 调大响应更快，但速度曲线出现超调和振荡。`kd` 调大能抑制超调、曲线更平，但微分项会放大噪声，输出（油门和刹车）更毛。**

`ki` 调大则消除稳态误差更快，但积分饱和后容易过冲。

:::

### 三、程序题

:::q{type="程序" points="14"}
写出横向 LQR 的反馈律，并说明 $Q_{bar}$ 和 $R_{bar}$ 是什么。

---

$$
k = (Q_{bar} + R_{bar} A^{T})^{-1} A^{T} Q_{bar}, \qquad u = -k \cdot e
$$

$e$ 是状态误差向量 $(\Delta e_y, \Delta \dot e_y, \Delta e_\psi, \Delta \dot e_\psi)^T$，$A$ 是离散化的状态转移矩阵。

$Q_{bar}$ 是**状态权重矩阵**，由连续时间的 $Q$ 离散化并加上状态增广（把常数项并进状态）得到。$Q$ 的对角线就是 $q_0$ 到 $q_3$，所以调这四个权重就是在调 $Q_{bar}$。

$R_{bar}$ 是**控制量权重矩阵**，主要给方向盘角速度留权重，作用是抑制控制动作的幅度，避免为了压误差而给出过于剧烈的转向。

$u = -k \cdot e$ 里的负号是反馈。稳态前馈不由这一项给出，那部分由参考线的曲率直接算。

:::

:::q{type="程序" points="14"}
写出离散 PID 的三个分量，并说明每一项在自动驾驶纵向控制里对应什么物理作用。

---

$$
u(k) = u_{ff} + k_p e(k) + k_i \sum_{j=0}^{k} e(j) + k_d \big(e(k) - e(k-1)\big)
$$

$u_{ff}$ 是**前馈**，由规划给出的期望加速度直接算出来，不由误差驱动。前馈把「本来该给多少」先给掉，反馈只负责补差。

$k_p e(k)$ 是**比例项**，对应当前误差。误差越大给得越多，响应快，但过大就震荡。

$k_i \sum e(j)$ 是**积分项**，把历史误差累积起来，用来消除稳态误差。自动驾驶里起步和长时间匀速行驶时误差会累积，积分项负责把它清掉。副作用是积分饱和后过冲。

$k_d (e(k) - e(k-1))$ 是**微分项**，对误差的变化率响应，起阻尼作用，抑制超调。副作用是放大噪声。

> [!TIP]
> Apollo 的纵向控制不是直接对速度误差做 PID，而是先把速度误差积分成「位置误差」再 PID。这样起步和停车更稳，因为速度误差本身在启停阶段不连续。

:::

### 四、解答题

:::q{type="解答" points="25"}
横向 LQR 的四个权重 $q_0$、$q_1$、$q_2$、$q_3$ 各自调大会发生什么？作业里的取值是 0.05、0.0、1.0、0.0，为什么 $q_2$ 比 $q_0$ 大二十倍？

---

**四个权重调大的后果。**

$q_0$（横向位置误差）调大：循迹更紧，横向偏差被压得更小。代价是控制器更敏感，车会贴着参考线走，急弯处容易切内道，压线甚至出线。

$q_1$（横向位置误差变化率）调大：抑制横向抖动，轨迹更平顺。代价是响应变钝，横向偏差的收敛变慢，弯道里跟不上。

$q_2$（航向误差）调大：车头更早对准路径方向，车身和路径的夹角更小。副作用是车头摆动会变多，急打方向的时候更明显。

$q_3$（航向误差变化率）调大：转向动作更平滑，方向盘不抖。代价是转向滞后变大，转向本身也变慢。

**为什么 $q_2$ 比 $q_0$ 大二十倍。**

横向控制的状态是 $(\Delta e_y, \Delta \dot e_y, \Delta e_\psi, \Delta \dot e_\psi)$。$q_0$ 和 $q_2$ 分别是位置误差和航向误差的权重。

关键在于**两个误差不是独立的**。横向位置误差是横向偏差本身，航向误差是车身朝向与路径切线的夹角。航向误差对横向偏差的导数关系是：横向偏差的变化主要由航向误差引起。也就是说，航向误差是横向偏差的「变化率」量级，它对系统的作用更强，权重就可以小；反过来，横向偏差在稳态时是一个常量，靠 $q_0$ 惩罚它代价很大——一味加大 $q_0$ 会让控制器为了压住一个常量偏差而给出很大的、持续的方向修正，表现为车在路上画龙。

所以 Apollo 取 $q_0 = 0.05$ 配 $q_2 = 1.0$：航向误差主导转向动作，位置误差只做轻微修正，$q_1$ 和 $q_3$ 取 0 表示不对变化率做额外惩罚，避免过度阻尼带来的滞后。

**倒车的另一组。** `reverse_matrix q` 是 0.5、0.0、0.4、0.0。倒车的横向误差定义和前进不同（参考线和几何关系反了），位置误差的权重要大得多，航向误差的权重也小一些。

:::

:::q{type="解答" points="25"}
给出一套「横向控制效果不好」的诊断流程：看哪些曲线、按什么顺序看、每种现象对应什么原因。

---

**第一步：先分清是哪一层的问题。**

看 `planning speed` 和轨迹图。规划出来的轨迹本身就不合理（比如规划穿过了障碍物、路径边界压到障碍物），那是**规划层**的问题，改控制参数没有用。把轨迹图和 `planning kappa`（规划曲率）对照：轨迹的形状应该和规划曲率一致，方向盘动作应该和 `curvature` 曲线一致。**规划曲率对不上轨迹形状，是规划的问题；轨迹形状对但实际轨迹偏离，是控制的问题。**

**第二步：看 `cross_track_error`。**

零线附近小幅波动：控制正常。持续单向偏移：稳态误差，可能是增益不够，也可能是存在一个恒定的外部扰动（车身歪、路面不平）。高频抖动：$q_1$、$q_3$ 太小或者 $k_d$ 太大。

**第三步：看 `heading_error`。**

它持续存在且不收敛，说明航向环的问题，$q_2$ 太小或者前馈的曲率算错了（曲率前馈错了会让车头始终偏一个角度）。它和 `cross_track_error` 一起振荡，说明两个环的增益不匹配。

**第四步：看 `curvature_error`。**

它是实际曲率与规划曲率之差。`curvature_error` 大而 `cross_track_error` 小，说明车在按规划的曲率走，只是走偏了——那是位置环的稳态误差。`curvature_error` 小而 `cross_track_error` 大，说明车走对了形状但整体平移了。

**第五步：看左右车道线误差。**

`left_line_error` 和 `right_line_error` 用来判断压线。一侧持续接近零说明车骑在车道线上。

**第六步：看 `line_slope_error` 和 `line_heading_error`。**

这两个是路径本身的横向斜率和朝向误差，用来判断**参考线**算得对不对。如果参考线在抖，问题在规划或者在车道线感知，不在控制器。

**顺序的意义：** 先用 `curvature_error` 把「规划的问题」和「控制的问题」分开，再在控制内部用「位置误差」和「航向误差」把「位置环」和「航向环」分开。不按这个顺序调参数，很容易把规划的问题用控制参数去掩盖，最后两边都改不对。

:::
::::
