---
title: 第 7 章 控制理论
timestamp: 2026-10-24T11:00:00+08:00
series: Auto-Drive-Intro
tags: [课内, Auto-Drive-Intro]
description: 控制模块的输入有四路：轨迹参考线、定位信息、地图信息、车辆反馈信号；输出只有三路：油门、刹车、方向盘 第七章 无人驾驶控制理论.pdf ·…
source_url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/课件/
toc: true
draft: false
---

```cardlink
url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/课件/
title: "智能驾驶导论 · 课件"
description: "中南大学《智能驾驶导论》课件，十一份 PDF，覆盖概述、感知、地图、定位、预测、控制与强化学习"
host: 10.188.31.55:5244
```

[← 返回《Auto-Drive-Intro》目录](../)

## 第 7 章 控制理论

### 控制模块在软件栈里的位置

控制模块的输入有四路：轨迹参考线、定位信息、地图信息、车辆反馈信号；输出只有三路：油门、刹车、方向盘 [第七章 无人驾驶控制理论.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=5)。这个接口决定了控制层的边界：感知、定位、地图、预测、规划都在上游，控制层拿到的是一条参考轨迹加上一堆状态量，要产出的是三个执行器的量。

控制任务有四条：不同环境下（雨雪、陡坡、石头路）的轨迹跟踪；保证车辆运动的可行性；保证平稳性；保证实时性 [第七章 无人驾驶控制理论.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=6)。四条里，实时性和平稳性是相互拉扯的：算得越久解越准，但留给执行的时间越少，车上人感觉到的顿挫越明显。

控制之所以需要一整套数学分支，是因为它要消除数学模型与物理模型的差异。三条硬指标是稳定性好、稳态误差小、动态误差小 [第七章 无人驾驶控制理论.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=10)。

### 控制模块的三个环节

控制模块被切成预处理、Controller、后处理三段 [第七章 无人驾驶控制理论.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=8)。

预处理做四件事：数据清洗（去掉不正常信号和 Planning 里的异常值）、合理性检查（定位信息、车盘底座信息等）、紧急处理（例如前方突然出现障碍物时的 Emergency Stop）、信号去噪（去噪声成分并做滞后补偿）[第七章 无人驾驶控制理论.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=8)。

后处理做两件事：Saturation/Limitation 处理，补偿执行器的固有缺陷；信号去噪，做简单的平滑和异常检查 [第七章 无人驾驶控制理论.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=9)。执行器饱和是个容易被忽略的坑：控制器算出再大的方向盘转角，车也只能转到物理极限，不做限幅就是把无效指令一路送到 CAN 总线上。

Controller 内部的设计分四步：建模、系统参数辨识、控制观测器设计、参数 Fine Tuning [第七章 无人驾驶控制理论.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=9)。

### 控制系统的评价指标

时域指标四个：超调量、超调时间、调整时间、上升时间。频域指标三个：通过频率、带宽、截止频率 [第七章 无人驾驶控制理论.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=11)。

横纵向控制分开看：纵向（加减速）看超调和调整时间，因为误差是标量且方向单一；横向（转向）看跟踪精度和相位滞后，因为误差是二维的，方向盘指令还要和车速耦合。

### 三种变换工具

傅里叶变换把信号拆成正弦余弦的叠加，拉普拉斯变换把微分变成乘法。拉普拉斯变换的定义是把时域函数乘上指数衰减再对时间积分：

$$
F(s) = \int_{0}^{\infty} f(t)\, e^{-st}\, \mathrm{d}t
$$

其中 $s = \sigma + j\omega$ 是复频率 [第七章 无人驾驶控制理论.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=12)。连续系统分析走 s 平面，离散系统走 Z 变换，课件把常见时间函数的拉氏变换和 Z 变换列在同一张表里对照 [第七章 无人驾驶控制理论.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=13)。

### PID 控制器：结构与三路作用

PID 是结构最简单、应用范围最广、最有效的控制器。三个优点：通用，适用于所有模型；属于反馈控制；输入可以是速度、位置等信号 [第七章 无人驾驶控制理论.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=15)。

三路输出相加后送进被控对象，构成闭环：

$$
u(t) = K_p\, e(t) + \frac{1}{T_I}\int_{0}^{t} e(\tau)\,\mathrm{d}\tau + T_D \frac{\mathrm{d}e(t)}{\mathrm{d}t}
$$

其中 $e(t) = R(t) - y(t)$ 是参考值与实际值的偏差 [第七章 无人驾驶控制理论.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=15)。

](images/fig-07-p15.png)

比例项 $K_p$ 越大，响应速度越快、稳态误差越小，但稳态误差消不掉；$K_p$ 继续加大还会增加系统不稳定性，甚至让系统震荡乃至发散 [第七章 无人驾驶控制理论.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=16)。

积分项的强弱取决于积分时间常数 $T_I$，$T_I$ 越小积分作用越强。它能消除稳态误差、提高系统无差度，代价是稳定性下降、动态响应变慢 [第七章 无人驾驶控制理论.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=17)。积分是消除稳态误差的唯一一路，也是引入 windup 的那一路。

微分项反映偏差信号的变化率，具有预见性，能产生超前的控制作用。它的副作用是对噪声干扰有放大作用，过强的微分调节对系统抗干扰不利，而且微分环节不单独使用，要与另外两种调节规律结合成 PD 或 PID [第七章 无人驾驶控制理论.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=18)。控制量来自定位和感知，本身带噪声，微分就是噪声放大器。

### 位置式与增量式 PID

按控制器输出和执行机构的关系，PID 分三种：位置式输出控制量大小的实际值，增量式输出控制量的增量，另有递推式算法 [第七章 无人驾驶控制理论.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=19)。

增量式写法只依赖当前和前两拍的误差，因此天然带积分累积：

$$
\Delta u(k) = K_p\bigl(e(k) - e(k-1)\bigr) + K_I\, e(k) + K_D\bigl(e(k) - 2e(k-1) + e(k-2)\bigr)
$$

对比位置式的 $u(k) = K_p e(k) + K_I\sum_{i=0}^{k} e(i) + K_D\bigl(e(k) - e(k-1)\bigr)$ [第七章 无人驾驶控制理论.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=19)。

在无人车领域，速度控制用位置式 PID，方向盘控制也用位置式 PID [第七章 无人驾驶控制理论.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=19)。位置上 PID 对单一变量控制很有效，无人车上可控的量是速度、加速度、位置 [第七章 无人驾驶控制理论.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=20)。

多个控制量需要串联：外环控制器的输出作为内环控制器的设定值，内环控制器的输出去操纵控制阀。串联改善控制效果，应对多变量控制和多干扰项 [第七章 无人驾驶控制理论.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=20)。

### 复杂控制器：建模与系统辨识

复杂控制器要同时满足三个要求：按照运动规划要求到达目标，适应不同路况，适配不同车型。做法是横向建模和纵向建模分开，控制器也分成横向控制和纵向控制 [第七章 无人驾驶控制理论.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=22)。

车辆模型表征车辆的物理特性，通过车辆状态和控制输入预测车辆下一时刻的轨迹。横向动力学模型有两个观测误差：heading 误差和横向误差 [第七章 无人驾驶控制理论.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=23)。

](images/fig-07-p23.png)

模型的某些未知参数（如前轮摩擦系数）要靠系统辨识获取，即通过实验采集参数。分三类：白箱（汽车大部分属于白箱辨识）、黑箱、灰箱，还有一类是 Learning-Based 方法（机器学习）[第七章 无人驾驶控制理论.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=24)。

### 最优控制理论的谱系

最优控制的定义是：给定一个系统和环境扰动，根据需求条件，实现最优的控制量策略。课件把相关控制器排成一条阶梯，复杂度和精准度都从左往右递增 [第七章 无人驾驶控制理论.pdf · p.25](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=25)。

](images/fig-07-p25.png)

### LQR：线性二次调节器

LQR 是基于模型的控制器，通过车辆状态和模型推理使误差最小，可用于设计横向控制器 [第七章 无人驾驶控制理论.pdf · p.26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=26)。

状态的取法决定控制器的形状：状态 X 取横向误差及其变化率、heading 误差及其变化率；控制量 U 取方向盘、油门、刹车 [第七章 无人驾驶控制理论.pdf · p.26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=26)。

线性系统写成状态方程，控制律是状态的线性反馈：

$$
\dot{x} = Ax + Bu, \qquad u = -Kx
$$

代价函数把状态误差和控制量消耗同时计入，$Q$ 和 $R$ 为正定矩阵：

$$
\text{cost} = \int_{0}^{T} \left(x^{\top} Q x + u^{\top} R u\right)\,\mathrm{d}t
$$

课件给了一句结论：尽可能减少误差，同时使用最小的输入量，这样能做到省油、平滑 [第七章 无人驾驶控制理论.pdf · p.27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=27)。

$K$ 的求法是解 Riccati 方程，连续时间下 $P$ 满足

$$
-\dot{P} = A^{\top} P + PA - PBR^{-1}B^{\top}P + Q, \qquad K = R^{-1}B^{\top}P
$$

离散版本是 $P_{k+1} = A^{\top}P_k A - A^{\top}P_k B\left(R + B^{\top}P_k B\right)^{-1}B^{\top}P_k A + Q$ [第七章 无人驾驶控制理论.pdf · p.28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=28)。$Q$ 大表示更在意误差，$R$ 大表示更省输入；调这两个矩阵是 LQR 落地时最常做的事。课件特别指出，无人车中使用 LQR 时 $N$ 一般为 0 [第七章 无人驾驶控制理论.pdf · p.28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=28)。

### MPC：模型预测控制

MPC 的基本步骤是三步：建立模型；计算有限时间范围内的最优控制输入；执行第一组控制输入 [第七章 无人驾驶控制理论.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=29)。

迭代优化的思想是：先算出一串控制输入并优化这一串，只执行第一组，然后循环 [第七章 无人驾驶控制理论.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=29)。LQR 与 MPC 的差别就在优化区间：LQR 针对固定的时间区间（一般为无限长时间）做优化，MPC 针对不同的时间区间分别优化 [第七章 无人驾驶控制理论.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=29)。

MPC 的代价是时效性与准确性的取舍：花费时间越长准确性越好，但那是搜索问题；解空间靠车辆动力学模型来缩小，模型、约束、优化三者结合 [第七章 无人驾驶控制理论.pdf · p.30](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=30)。

](images/fig-07-p31.png)

误差跟踪的状态方程把扰动显式列出来：

$$
\dot{x} = Ax + Bu + C
$$

状态 X 取横向误差、heading 误差、位置误差、速度误差；控制量 U 取转向角、加速度；扰动 C 取横向角变化率和加速度参考值。离散形式是 $x(k+1) = A(k)x(k) + B(k)u(k) + C$ [第七章 无人驾驶控制理论.pdf · p.32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=32)。

完整的求解链路是：对未来 N 个周期做状态方程预测，写出目标函数和约束，问题落成二次规划形式，求出一组控制序列 [第七章 无人驾驶控制理论.pdf · p.33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=33)。二次规划能实时求解，是 MPC 能上车的前提；写成非凸问题就只能离线算了。

### Matlab 工具箱与 Apollo

Apollo 的控制算法包含纵向 PID 控制器、横向 PID 控制器和 MPC 控制器 [第七章 无人驾驶控制理论.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=35)。这正好说明 PID 和 MPC 不是二选一：纵向用 PID，横向用 MPC 或者 PID，各自解决自己擅长的问题。

Matlab 侧的工具箱清单是：Simulink、Carsim，Control System Toolbox、Robust Control Toolbox、Automated Driving Toolbox、Model Predictive Control Toolbox [第七章 无人驾驶控制理论.pdf · p.36](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=36)。
