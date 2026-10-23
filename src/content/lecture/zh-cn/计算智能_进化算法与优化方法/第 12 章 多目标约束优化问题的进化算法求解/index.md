---
title: 第 12 章 多目标约束优化问题的进化算法求解
timestamp: 2026-10-21T20:30:00+08:00
series: 计算智能
tags: [课内, 计算智能]
description: 多目标约束优化问题（multiobjective constrained optimization problem, MCOP）在多目标优化…
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

## 第 12 章 多目标约束优化问题的进化算法求解

### 多目标约束优化问题的形式

多目标约束优化问题（multiobjective constrained optimization problem, MCOP）在多目标优化的基础上加了约束。课件把问题写成目标函数向量加两类约束 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.2](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=2)：

$$
\min\; f_1(\mathbf{x}), f_2(\mathbf{x}), \dots, f_m(\mathbf{x})
$$

$$
\text{s.t.}\quad g_j(\mathbf{x}) \le 0,\; j = 1, \dots, l
$$

$$
h_j(\mathbf{x}) = 0,\; j = l + 1, \dots, m
$$

$$
\mathbf{x} = (x_1, x_2, \dots, x_D) \in S
$$

$D$ 是决策变量的维数，$S$ 是决策空间。两类约束连续编号：不等式约束 $g_j(\mathbf{x}) \le 0$ 占前 $l$ 个下标，等式约束 $h_j(\mathbf{x}) = 0$ 的下标接着从 $l+1$ 数起；目标函数的末位下标与等式约束的末位下标都写作 $m$。

约束把一部分解挡在可行域之外，帕累托前沿因此分成两条：不受约束影响的那条叫无约束帕累托前沿，落进可行域的那条叫约束帕累托前沿 (constrained PF)。课件用四张图区分两者的关系 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.2](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=2)。有一种情形里整条无约束前沿都落在可行域内，约束帕累托前沿与无约束前沿完全重合，约束不起作用。另一种情形里前沿有一段跑到可行域外面，这一段被切掉，约束帕累托前沿变短并向可行域内侧弯。可行域的形状直接决定前沿的形状，前沿不再只由目标函数之间的冲突决定。

课件给出这个领域的两大挑战 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.2](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=2)。第一，现有的测试问题设计得不好，可行域比例过大就是典型表现，可行域一大，随机初始化就能撞上大量可行解，约束形同虚设。第二，现有方法无法很好地收敛到帕累托前沿，在这样的问题上拿到的解集往往既不收敛也不均匀。

已有人工 MCOP 测试函数按构造方式分成若干族 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.3](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=3)：Deb 等人提出的 CTP 函数，Zhang 等人提出的 CF 函数，Jain 等人提出的 C-DTLZ 函数，Li 等人提出的 NCTPs 函数，以及 Fan 等人提出的 DAC-MOP1-15 函数。后面两节讲的 MW 与 DOC 是针对这套测试函数的缺陷重新设计出来的两套测试套件。

### 测试套件一：MW

MW 套件分两步造测试函数 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=4)：第一步全局控制 (global control process)，第二步局部调整 (local adjustment process)。两步各管一件事：全局控制管可行域的大小，局部调整管约束的复杂度。

全局控制分三步 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=4)。第一步选一组数学形式相近的函数，课件的例子是两个圆：

$$
L_1:\; x^2 + y^2 - 1^2 = 0
$$

$$
L_2:\; x^2 + y^2 - 1.2^2 = 0
$$

第二步由其中一条曲线产生无约束帕累托前沿。取 $L_1$ 时前沿满足 $f_1^2 + f_2^2 - 1^2 = 0$，即

$$
f_2 = \sqrt{1 - f_1^2}
$$

第三步由同族的另一条曲线写出不等式约束。取 $L_2$ 得 $f_1^2 + f_2^2 - 1.2^2 = 0$，写成

$$
c(\bar{\mathbf{x}}) = f_1^2 + f_2^2 - 1.2^2 \le 0
$$

两个圆合起来给出一个测试问题 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=4)：

$$
\min \begin{cases} f_1 = x_1 \\ f_2 = g\sqrt{1 - f_1^2} \end{cases}
\quad \text{s.t.}\quad c(\bar{\mathbf{x}}) = f_1^2 + f_2^2 - 1.2^2 \le 0
$$

$g$ 是决策变量部分，负责把决策空间撑开。配套的图标出可行域的边界是 $L_2$，约束前沿与无约束前沿重合在 $L_1$ 上 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=4)。换一个半径，可行域的大小随之改变，这就是全局控制能调节的地方。

局部调整往约束里塞进一个周期函数 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=5)，函数形式是

$$
A \sin\left(B \cdot l(\bar{F})^{\,C}\right)^{D}
$$

$l(\bar{F})$ 取目标函数值向量的一个角度，$A$ 调幅值，$B$ 决定局部形状的个数，$C$ 决定它们的分布，$D$ 决定凹凸性。落到具体的约束上，课件写的是 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=5)：

$$
f_1^2 + f_2^2 - \left(1.2 - 0.15 \sin(20l)\right)^2 \le 0
$$

角度项的取法是 $l = \arctan(f_2/f_1)$，写在配套的图注里 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=5)。周期函数让可行域的边界长出一排波浪，边界复杂度因此上升。

MW 套件有四条特点 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=6)。可行域比例小且可控，来自全局控制里换半径的操作。约束在目标函数上有足够强的非线性，来自局部调整加进去的周期函数。约束帕累托前沿的几何结构多样。全局控制如果选多个相近的函数一起用，还能造出带多个约束的测试问题。

套件共 14 个问题，编号 MW1 到 MW14，分两批出现在课件里 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=7) 与 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=8)。其中 MW4、MW8、MW14 的图上标了 $m=3$，是三目标问题。

### 测试套件二：DOC

DOC 套件针对的是另一个缺口 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=9)。真实的 MCOP 里，决策空间的约束 (decision constraints) 与目标空间的约束 (objective constraints) 同时存在，现有人工测试问题从不同时考虑这两类约束，模拟真实场景的能力因此有限。课件用购车问题说明两类约束的区别 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=9)。价格与舒适度是两个目标，约束有三条：Cost $\le \$50K$ 直接写成目标函数的不等式，属于目标空间约束；Seats $\ge 5$ 与 Country $=$ Germany 描述的是决策变量本身，在目标空间里写不出来，属于决策空间约束。目标空间约束切掉可行域的一角，决策空间约束则直接改变决策变量能取的范围。

DOC 由三部分组成 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=10)：一个无约束的多目标优化问题、目标空间里的约束、决策空间里的约束。

课件给出的构造例子按这三部分铺开 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=11)。无约束问题取

$$
\min \begin{cases} f_1 = x_1 \\ f_2 = g(\mathbf{x})\left(1 - \frac{f_1}{g(\mathbf{x})}\right) \end{cases}
$$

目标空间约束取两条不等式：

$$
g_1 = f_1^2 + f_2^2 \le 1
$$

$$
g_2 = (f_1 + 1)^2 + (f_2 + 1)^2 \ge 5
$$

决策空间约束取一条不等式加一条等式：

$$
g_3 = x_2 + 2x_3 \le 6
$$

$$
h_1 = 2x_2 + x_3 = 6
$$

两类约束的效果不同 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=11)。目标空间约束在目标平面上切出一个很小的可行域，帕累托前沿从一条直线变成一段弧。决策空间约束作用在变量上，$x_2$–$x_3$ 平面上的可行域被压到极窄。

DOC 共 9 个实例，编号 DOC-1 到 DOC-9 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=12)。实例表给出每个问题的目标数 $m$、维数 $D$、目标空间约束个数 (NOC)、决策空间约束个数 (NDC)、不等式约束个数 (NIC)、等式约束个数 (NEC)、可行域比例，以及帕累托前沿和可行域各自的性质。可行域比例一列说明这套问题的难度：DOC-1 的可行域比例是 26.97%，DOC-4 是 0.53%，其余七个实例全是 0.00%。前沿性质一列覆盖凹、凸、线性这些形状和连续、离散、混合、退化这几种形态，可行域性质一列覆盖非线性、极小、多模这几种形态。配套的图给出一个完整的问题定义：目标函数带 $g(\mathbf{x})$ 的具体表达式，目标空间约束、决策空间约束、搜索空间全部列出。

整套 DOC 的特点写在课件的收尾页 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=13)。决策空间约束的类型很多，不等式约束、等式约束、线性约束、非线性约束都有，这些约束取自真实应用，所以决策空间的可行域有非线性、极小、多模等复杂性质。目标空间约束设计得简单可控，作用是把可行域缩小，同时让帕累托前沿有连续、离散、混合这些不同形态。整体上，DOC 给约束多目标进化算法出了难题：算法要同时给出分布均匀、收敛良好、而且可行的解集。

### ToP：两阶段框架

ToP 是一个简单而高效的两阶段框架 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=14)。框架的图给出三步：初始种群先撒在可行域外，第一阶段结束后种群被拉进可行域并聚到帕累托前沿附近，第二阶段再把种群铺到整条前沿上。

第一阶段做的事是 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=15)：把 MCOP 改写成带约束的单目标问题 (SCOP)。改写方式是忽略目标函数之间的冲突，把 $m$ 个目标函数直接求和：

$$
\min\; \sum_{i=1}^{m} f_i(\mathbf{x})
$$

两类约束原样保留，不等式约束仍是 $g_j(\mathbf{x}) \le 0$（$j = 1, \dots, l$），等式约束仍是 $h_j(\mathbf{x}) = 0$（$j = l+1, \dots, m$）。求和让目标函数之间不再互相拉扯，单目标求解器能一路把种群拉进可行域。课件给出这样改写的两点好处 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=15)：收敛速度更快，可行域内部的早熟收敛也被缓解。

第一阶段结束后的种群位置由图的三个子图给出 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=15)。三个子图分别对应种群聚在帕累托前沿上、中、下三处的情况，每一种情形下种群都落进了可行域并紧贴前沿，蓝色箭头标出种群将要扩展的方向。

第二阶段做回多目标 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=16)：用一个具体的多目标进化算法求出最终解。原则上任何多目标进化算法都能放进第二阶段。课件在 DOC-1、DOC-4、DOC-6、DOC-7 上对比了加 ToP 前后 NSGA-II-CDP 与 IDEA 的表现 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=16)：加进 ToP 之后，解的分布明显贴近真实的帕累托前沿，前沿两端的空白也被填上。

### ToR：两个排名加权的适应度

ToR 面向约束处理技术 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=17)。在约束多目标优化里，约束支配原理 (constrained dominance principle, CDP) 是其中一类。CDP 的优点是简单、灵活、没有参数、容易嵌进多目标进化算法。缺点也在这里：CDP 总是让约束优先于目标，容易导致早熟收敛。课件给出 CDP 的定义 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=17)：解 $\vec{a}$ 约束支配解 $\vec{b}$（记作 $\vec{a} \prec_c \vec{b}$）当三条之一成立：$\vec{a}$ 与 $\vec{b}$ 都可行且 $\vec{a} \prec \vec{b}$；$\vec{a}$ 可行而 $\vec{b}$ 不可行；$\vec{a}$ 与 $\vec{b}$ 都不可行且 $G(\vec{a}) < G(\vec{b})$，$G$ 是约束违反量。

ToR 的思路是绕开 CDP 单一排序的局限 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=18)：每个解的适应度由两个排名加权求和，一个排名按 CDP 给出 $R_i^c$，另一个排名忽略约束、按帕累托支配给出 $R_i^p$。这样目标函数的信息也进了适应度。

课件把两个排序的流程写成两个算法 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=19)。算法一按 CDP 排序，算法二按帕累托支配排序，两者步骤相同：先把种群按各自的支配关系分进非支配层 $L = \{L_1, L_2, \dots\}$，再在每一层内部按拥挤度距离降序排，最后记录每个解的名次，分别得到 $R_i^c$ 与 $R_i^p$。

适应度函数与权重系数写在课件的公式里 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=19)：

$$
\min\; fit_i = \alpha R_i^c + (1 - \alpha) R_i^p
$$

$$
\alpha = 0.5 + 0.5 \cdot P_{fea}
$$

$P_{fea}$ 是可行解在种群中所占的比例，它自适应地反映进化过程中种群的状态，系数 $\alpha$ 因此在约束与目标之间做平衡。可行解多时 $\alpha$ 接近 1，适应度主要由 CDP 排名决定；可行解少时 $\alpha$ 接近 0.5，两个排名的权重相当，目标函数的信息能多进来一些。

课件用三个例子演示 $\alpha$ 随 $P_{fea}$ 变化的效果 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=20) [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=21) [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=22)。例 (a) 的 $P_{fea} = 0$，系数 $\alpha = 0.5$；例 (b) 的 $P_{fea} = 0.4$，系数 $\alpha = 0.7$；例 (c) 的 $P_{fea} = 0.8$，系数 $\alpha = 0.9$。每个例子给出 10 个解的目标值 $(f_1, f_2)$、两种排名给出的名次（表里同时标出非支配排序的层号 NL 与拥挤度距离 CD）、以及算出的适应度。$R_i^c$ 行与 $fit_i$ 行各有 5 个格子加了底色，标出各自行里数值最小的 5 个解。三个例子连起来看，这两套解集随 $P_{fea}$ 上升逐步靠拢：例 (a) 里按 CDP 选出的 5 个解与按适应度选出的 5 个解重合 3 个，例 (b) 重合 4 个，例 (c) 完全重合。

ToR 的特点写在最后一页 [Chapter 12_Evolutionary Algorithms for Solving Multiobjective Constrained Optimization Problems.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%2012_Evolutionary%20Algorithms%20for%20Solving%20Multiobjective%20Constrained%20Optimization%20Problems.pdf#page=23)。因为排序时用了帕累托支配，ToR 用上了目标函数的信息，保住了若干有潜力的不可行解。这些不可行解能帮算法去探可行域里还没走到的地方，从不可行的一侧逼近那些压在可行域边界上的帕累托最优解。$P_{fea}$ 趋近 1 时，ToR 的表现与 CDP 接近，可行解的数量有保证。这一条重要，因为最后交出去的解必须可行，不可行解再多也不能算数。
