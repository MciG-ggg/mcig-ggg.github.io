---
title: 第 10 章 多目标优化中多个最优解的定位
timestamp: 2026-10-21T20:30:00+08:00
series: 计算智能
tags: [课内, 计算智能]
description: 非线性方程系统（nonlinear equation system, NES）是一组需要同时成立的方程。课件把它的形式写成：
source_url: http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/
toc: true
draft: false
---
```cardlink
url: http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/课件/
title: "计算智能 · 课程课件"
description: "中南大学自动化学院《计算智能》课程的全部课件"
host: 10.188.31.55:5244
```

[← 返回《计算智能：进化算法与优化方法》目录](../)

## 第 10 章 多目标优化中多个最优解的定位

### 非线性方程系统：形式、求解目标与三类已有方法

非线性方程系统（nonlinear equation system, NES）是一组需要同时成立的方程。课件把它的形式写成：

$$
e_1(x)=0,\ e_2(x)=0,\ \dots,\ e_m(x)=0,\qquad x=(x_1,\dots,x_D)\in S=\prod_{i=1}^{D}[L_i,U_i]
$$

$m$ 个方程、$D$ 个决策变量，第 $i$ 个变量的取值区间是 $[L_i,U_i]$，$S$ 是这些区间的直积 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=4)。这类问题出现在化学过程、机器人、电子电路、工程材料与物理中。

课件给的例子很简单：

$$
e_1(x_1,x_2)=x_1^2+x_2^2-1=0,\qquad e_2(x_1,x_2)=x_1-x_2=0,\qquad -1\le x_1,x_2\le 1
$$

第一条方程是单位圆，第二条是对角直线，两条曲线在决策空间里交于两点，课件把这两个交点标成最优解 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=5)。一个 NES 因此可能同时包含多个最优解。

这类问题的分量有数学上的来源。1998 年菲尔兹奖得主 Steve Smale 列出了面向 21 世纪的 18 个挑战问题，其中包含 "Does P = NP?"；与 NES 相关的有三个：Problem 4（多项式的整数零点）、Problem 8（把动力学引入经济理论）、Problem 17（解多项式方程）。求 NES 的最优解既有工程意义，也是数学的核心问题之一 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=6)。

用进化算法解 NES，目标是"一次运行定位所有最优解"（locate all the optimal solutions in a single run）。课件给出的流程是：NES 先经过一次变换变成某种优化问题，再交给进化算法求解 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=7)。围绕这次变换，已有的做法分三类。

单目标变换把方程组压成一个标量目标：

$$
\min_{x\in S}\ \sum_{i=1}^{m}\left|e_i(x)\right|
\qquad\text{或}\qquad
\min_{x\in S}\ \sum_{i=1}^{m}e_i^2(x)
$$

主要缺陷：一次运行通常只能找到一个最优解 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=8)。

约束变换把方程组放进约束条件：

$$
\min_{x\in S}\ \sum_{i=1}^{m}\left|e_i(x)\right|
\quad\text{s.t.}\quad e_i(x)\ge 0,\ i=1,\dots,m
$$

约束也可以写成 $e_i(x)=0$。这一类方法同样受"一次运行只能定位一个最优解"限制，另外还要额外引入约束处理技术 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=9)。

多目标变换（CA 方法，Grosan 与 Abraham 2008）把每个方程当成一个目标：

$$
\min_{x\in S}\ \left(\left|e_1(x)\right|,\left|e_2(x)\right|,\dots,\left|e_m(x)\right|\right)
$$

目标个数等于方程个数。缺陷有两个：可能受"维数灾难"（也就是超多目标问题）困扰；一次运行也许只找到一个解 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=10)。

### MONES：把方程系统转成双目标问题

用进化算法解 NES 时，人们期望一次运行定位多个最优解。这个过程与用进化算法解多目标优化问题（multiobjective optimization problem, MOP）很像。于是自然的问题来了：NES 能否被变换成 MOP，让多目标进化算法去解变换后的问题。Song、Wang、Li 与 Cai 在 2015 年给出了肯定答案，方法称为 MONES（multiobjective optimization for NESs）[Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=11)。

变换后的双目标问题写成：

$$
\min_{x\in S}\begin{cases}
f_1(x)=x_1+\displaystyle\sum_{i=1}^{m}\left|e_i(x)\right|\\[2mm]
f_2(x)=1-x_1+m\cdot\max\left(\left|e_1(x)\right|,\dots,\left|e_m(x)\right|\right)
\end{cases}
$$

课件在这一页把原方程组和变换后的双目标问题并排放在一起 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=12)，然后把这个式子拆成两项分别讲。第一项只依赖第一个决策变量，课件用 $\alpha_1$、$\alpha_2$ 记它：

$$
\min_{x\in S}\begin{cases}
\alpha_1(x)=x_1\\[1mm]
\alpha_2(x)=1-x_1
\end{cases}
$$

$\alpha_1$ 随 $x_1$ 严格单调递增，$\alpha_2$ 随 $x_1$ 严格单调递减，两条直线在 $x_1=0.5$ 处相交。课件由此给出结论：NES 决策空间里的每一个决策向量都是第一项的 Pareto 最优解，第一项最优解在目标空间里的像都落在线段 $y=1-x$ 上 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=13)。

第二项度量方程组被满足的程度，课件用 $\beta_1$、$\beta_2$ 记它：

$$
\min_{x\in S}\begin{cases}
\beta_1(x)=\displaystyle\sum_{i=1}^{m}\left|e_i(x)\right|\\[2mm]
\beta_2(x)=m\cdot\max\left(\left|e_1(x)\right|,\dots,\left|e_m(x)\right|\right)
\end{cases}
$$

两项都是非负的。$\beta_1$ 是各方程绝对值之和，$\beta_2$ 是各方程绝对值的最大值再乘方程个数 $m$。课件给出两条 Remark：Remark I 是最优解 $x^*$ 满足 $\beta_1(x^*)=\beta_2(x^*)=0$；Remark II 是参数 $m$ 用来让 $\beta_1(x^*)$ 与 $\beta_2(x^*)$ 的量级相当 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=14)。

把两项加起来就是上面的 $f_1$ 与 $f_2$。于是 NES 最优解在目标空间里的像同样落在 $y=1-x$ 的线段上 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=15)。课件对 MONES 的评价是三点：NES 被转成了一个双目标优化问题；这个变换带来若干非常好的性质；NSGA-II 之类的多目标进化算法可以直接拿来解这个变换后的问题 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=16)。

### MONES 与 CA 的差别，以及 MONES 的实验

课件用前面那个具体系统对比 CA 与 MONES，在决策空间和目标空间各做一遍。决策空间里放进六个个体 A 到 F，目标空间里画出这些个体在两种变换下的像。差别落在筛选结果上——按 CA 的目标空间做非支配排序，留下的个体是 B 与 D；按 MONES 的目标空间做，留下的个体是 E 与 B [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=17) [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=18)。同样一批个体、同样的最优解，两种变换给出的 Pareto front 上的分配并不一样。

实验用 F1 到 F7 共七个测试实例。课件给出的表格记录了每个实例的决策变量个数、决策空间、线性方程个数、非线性方程个数与最优解个数。F1 是上面那个两变量例子，两个最优解；F2 有 20 个决策变量、2 个非线性方程、2 个最优解；F3 是 2 个决策变量、11 个最优解；F4 是 2 个决策变量、15 个最优解；F5、F6、F7 的最优解个数都是无穷多个 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=19)。

评价指标有两个。第一个是 IGD（Inverted Generational Distance，反向世代距离），它度量求得的 Pareto front 与真实前沿的距离，越小越好 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=20)。课件的表格给出 CA 与 MONES 在七个实例上的 best、mean、worst 与标准差。F1 上 IGD 的平均值 CA 是 9.99E-01、MONES 是 2.01E-04；F3 上是 1.16E+00 对 2.12E-03；F6 上是 1.66E+00 对 2.31E-02；F7 上是 8.41E-01 对 1.15E-01。MONES 在每个实例上都更低。

第二个指标是 NOF（Number of the Optimal Solutions Found，最优解的数目），它直接数找到了多少个最优解 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=21)。F3 上 CA 找到 1.00E+00 个、MONES 找到 1.10E+01 个；F4 上是 1.00E+00 对 1.41E+01；F5 上是 1.00E+00 对 3.73E+01；F6 上是 0.00E+00 对 7.31E+01；F7 上平均值是 1.80E+00 对 2.02E+01。CA 在这几个实例上几乎只找到一个解。

课件还画出 F1、F4、F5 的收敛行为，在决策空间里对比 CA 与 MONES 的一次典型运行 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=22)。

### WeB：给每个决策变量配一个权重

MONES 有一处弱点：只用第一个决策变量构造双目标问题。若多个最优解的第一个决策变量取值相同，MONES 会把其中一些丢掉。WeB（weighted biobjective transformation）针对这一点改动。

WeB 保留 MONES 的三个好性质：双目标结构（目标函数个数最少，只有两个）、两个目标互相冲突（NES 的最优解因此成为变换后问题的 Pareto 最优解）、Pareto front 是线性的（$y=1-x$，是多目标进化算法最容易逼近的类型）[Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=23)。

改动在第一项：给每个决策变量配一个权重，权重向量 $w=(w_1,\dots,w_n)$ 的第 $i$ 个分量 $w_i$ 在 0 和 1 之间随机取值，并与 $x_i$ 绑定 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=24)。变换后的双目标问题写成：

$$
\min_{x\in S}\begin{cases}
f_1(x)=\dfrac{\sum_{i=1}^{n}w_i x_i}{\sum_{i=1}^{n}w_i}+\displaystyle\sum_{i=1}^{m}\left|e_i(x)\right|\\[3mm]
f_2(x)=1-\dfrac{\sum_{i=1}^{n}w_i x_i}{\sum_{i=1}^{n}w_i}+\displaystyle\sum_{i=1}^{m}\left|e_i(x)\right|
\end{cases}
$$

分子是决策变量的加权和，分母是权重之和，所以这一项是 $x$ 的一个加权平均，取值落在 $x$ 各分量的最小值与最大值之间；再加上第二项，$f_1$ 与 $f_2$ 仍然互相冲突。

WeB 与 MONES 的关系很清楚：若 $w_1\ne 0$ 且 $(w_2,\dots,w_n)=(0,\dots,0)$，WeB 就退化成 MONES，所以 MONES 只是 WeB 的一个特例。WeB 继承了 MONES 的三个好性质，并多出一条 one-to-one mapping：NES 的所有最优解会映到 Pareto front 上互不相同的点，从而不会互相挤掉 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.25](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=25)。

### 多模态优化问题：形式与已有方法

现实中的很多优化问题具有 multimodal 性质，多个最优解可能同时存在。这类问题称为多模态优化问题（multimodal optimization problem, MMOP），形式是：

$$
\max_{x\in S} f(x)\quad\text{或}\quad\min_{x\in S} f(x),
\qquad x=(x_1,\dots,x_D)\in S=\prod_{i=1}^{D}[L_i,U_i]
$$

与 NES 的差别在于目标只有一个，多个最优解来自函数本身的多峰结构 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.27](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=27)。课件这一页给出若干多模态函数的图形例子 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.28](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=28)。

解 MMOP 的已有方法有两类。

Niching methods（小生境方法）在决策空间里显式划分区域，让每个峰由若干个体占据。最早的 niching method 是 Cavicchio 在 1970 年提出的 preselection method；现在流行的做法有 clearing（Pétrowski，ICEC 1996）、fitness sharing（Goldberg 与 Richardson，ICGA 1987）、crowding（De Jong，1975 年博士论文）、restricted tournament selection（Harik，ICGA 1995）、speciation（Li 等，ECJ 2002）。缺陷是：需要一些依赖于具体问题的 niching 参数 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=29)。

多目标方法通常构造两个目标：第一个是原多模态函数，第二个是距离信息（Das 等，IEEE TEVC 2013）或梯度信息（Deb 与 Saha，ECJ 2012）。缺陷有两个：无法保证变换后的两个目标完全冲突；原问题的最优解与变换后问题的 Pareto 最优解之间的关系难以从理论上验证 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.30](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=30)。

### MOMMOP：双目标变换、隐含小生境与两个问题

MOMMOP（multiobjective optimization for MMOPs，Wang、Li、Gary 与 Song，2015）把 MONES 的思路搬到 MMOP 上 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=31)。

变换后的双目标问题写成：

$$
\min_{x\in S}\begin{cases}
f_1(x)=x_1+\dfrac{\left|f(x)-\mathrm{best\_refer}\right|}{\left|\mathrm{worst\_refer}-\mathrm{best\_refer}\right|}\cdot (U_1-L_1)\cdot \eta\\[3mm]
f_2(x)=1-x_1+\dfrac{\left|f(x)-\mathrm{best\_refer}\right|}{\left|\mathrm{worst\_refer}-\mathrm{best\_refer}\right|}\cdot (U_1-L_1)\cdot \eta
\end{cases}
$$

第一项是 $x_1$ 与 $1-x_1$，与 MONES 的第一项相同；第二项由课件单独展开讲 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.32](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=32)。第二项里的各个量课件逐个标注：$\mathrm{best\_refer}$ 是进化过程中最好的个体的目标函数值，$\mathrm{worst\_refer}$ 是最差个体的目标函数值，$f(x)$ 是当前个体的目标函数值，$U_1-L_1$ 是第一个决策变量的取值范围，最后那个系数是 the scaling factor（缩放因子），这里的 $\eta$ 记它，课件第 33 页的公式把它写作 $\alpha$，第 34 页的公式写成斜体的 $a$。课件的 Remark 点明设计意图：目的是让第一项与第二项处在同一量级。对原 MMOP 的最优解，$f(x)=\mathrm{best\_refer}$，第二项取到零 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.33](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=33)。把两项合起来，MMOP 最优解在目标空间里的像同样落在 $y=1-x$ 的线段上 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.34](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=34)。

课件接着回答"MOMMOP 为什么有效"。答案是 MOMMOP 是一种隐含的小生境方法（implicit niching method）。课件用三个个体说明。决策空间里的点是 $(0.1,1)$、$(0.15,0.8)$、$(0.6,0.8)$，横坐标是 $x$、纵坐标是 $f(x)$。变换到目标空间，三个个体的像是 $(0.0,0.9)$、$(0.35,1.05)$、$(0.8,0.6)$。课件在图上算了两笔账：函数值 0.8 的那个个体加上第二项得到 $0.15+0.2=0.35$ 与 $1-0.15+0.2=1.05$，函数值 0.8 的另一个个体得到 $0.6+0.2=0.8$ 与 $1-0.6+0.2=0.6$。关键在中间两个点：它们的目标函数值相同，第一个决策变量不同，变换后落到目标空间里两个不同的位置，彼此不支配；函数值最高的那个点第二项为零，落在 $(0.0,0.9)$。个体在目标空间里靠位置分开，效果上等价于把不同区域分开占据，这就是"隐含的小生境" [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=35)。

课件随后指出 MOMMOP 的两个问题。

第一个问题：一些最优解可能在一个或多个决策变量上取相同的值 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.36](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=36)。课件给出的解法是换用别的决策变量来构造双目标问题：分别用 $x_1$、$x_2$、$x_D$ 构造三个双目标问题 BOP1、BOP2、BOPD，形如

$$
\min_{x\in S}\begin{cases}
f_1^{(k)}(x)=x_k+\dfrac{\left|f(x)-\mathrm{best\_refer}\right|}{\left|\mathrm{worst\_refer}-\mathrm{best\_refer}\right|}\cdot (U_k-L_k)\cdot \eta\\[3mm]
f_2^{(k)}(x)=1-x_k+\dfrac{\left|f(x)-\mathrm{best\_refer}\right|}{\left|\mathrm{worst\_refer}-\mathrm{best\_refer}\right|}\cdot (U_k-L_k)\cdot \eta
\end{cases}
$$

支配判据也随之改变：只有当 $x_u$ 在全部三个双目标问题上都支配 $x_v$ 时，才判定 $x_u$ 支配 $x_v$：

$$
x_u\prec x_v\ \text{on BOP1}\ \wedge\ x_u\prec x_v\ \text{on BOP2}\ \wedge\ \dots\ \wedge\ x_u\prec x_v\ \text{on BOPD}
$$

第二个问题：某些吸引域（basins of attraction）里个体很少，同时同一吸引域内有些个体彼此十分相似 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.37](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=37)。后者让相似的个体互相不支配，谁也淘汰不掉谁，计算资源被白白浪费。课件引入第二条比较准则来补足 Pareto 支配：$f(x_a)$ 优于 $f(x_b)$，并且 $x_a$ 与 $x_b$ 归一化之后的差小于 0.01：

$$
f(x_a)\ \text{优于}\ f(x_b)\ \wedge\ \mathrm{normalization}(x_a)-\mathrm{normalization}(x_b)<0.01
$$

比较两个个体时，两条准则并列使用 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.38](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=38)：满足"在所有 BOP 上支配"，或者满足"$f$ 更优且归一化后足够接近"，任一条成立即判定前者支配后者。

### 实验结果与整个方法框架

实验用 IEEE CEC2013 niching methods 专题与竞赛提供的 20 个 benchmark 函数。课件列出它们的名单：F1 Five-Uneven-Peak Trap、F2 Equal Maxima、F3 Uneven Decreasing Maxima、F4 Himmelblau、F5 Six-Hump Camel Back、F6 Shubert with 2D、F7 Vincent with 2D、F8 Shubert with 3D、F9 Vincent with 3D、F10 Modified Rastrigin，F11 到 F20 是 Composition Function 1 到 4 在 2D、3D、5D、10D、20D 上的组合版本 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.39](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=39)。

第一组对比针对 CEC2013 里四个较新的方法：dADE/nrand/1、NVMO、PNSGA-II、MNSGA-II。多问题 Wilcoxon 检验的结果是 MOMMOP 对这四个方法全部显著（在 0.05 与 0.1 两个显著性水平上都是 Yes），$p$ 值依次为 3.61E-02、2.71E-03、3.81E-05、4.77E-06。Friedman 检验的排名是 MOMMOP 1.75、dADE/nrand/1 2.275、NVMO 2.75、PNSGA-II 3.475、MNSGA-II 4.75，MOMMOP 排在最前 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.40](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=40)。

第二组对比针对四个单目标方法：NCDE、NSDE、LIPS、r2pso。Friedman 排名 MOMMOP 1.875、LIPS 2.525、NSDE 2.725、NCDE 3.25、r2pso 4.625。Wilcoxon 检验里 MOMMOP 对 NCDE 显著（$p=1.87E-04$）、对 r2pso 显著（$p=4.77E-06$），对 NSDE 在 0.1 水平上显著而 0.05 水平上不显著（$p=7.64E-02$），对 LIPS 同样在 0.1 水平上显著、0.05 水平上不显著（$p=5.46E-02$）[Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.41](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=41)。

第三组对比针对两个多目标方法：MOBIDE 与 BMPGA。Friedman 排名 MOMMOP 1.375、MOBIDE 1.75、BMPGA 2.875。Wilcoxon 检验对 MOBIDE 不显著（$p=1.56E-01$），对 BMPGA 显著（$p=1.56E-02$）[Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.42](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=42)。

课件最后给出一个数字：216 个最优解，配图是多模态函数 $f(x_1,x_2)$ 的曲面与被定位到的解 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.43](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=43)。

整套方法的框架由四点组成：为 NES 与 MMOP 提出一个基于多目标优化的通用框架；NES 或 MMOP 被变换成一个双目标问题，且两个目标互相冲突；NES 或 MMOP 的最优解就是变换后双目标问题的 Pareto 最优解；多目标进化算法在一次运行里逼近这个问题的 Pareto 最优解，NES 或 MMOP 的多个最优解随之到手 [Chapter 10_Multiobjective Optimization for Locating Multiple Optimal Solutions of NESs and MMOPs.pdf · p.45](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2010_Multiobjective%20Optimization%20for%20Locating%20Multiple%20Optimal%20Solutions%20of%20NESs%20and%20MMOPs.pdf#page=45)。
