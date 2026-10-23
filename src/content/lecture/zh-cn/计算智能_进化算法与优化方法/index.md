---
title: 计算智能：进化算法与优化方法
timestamp: 2026-10-21T20:30:00+08:00
series: 计算智能
tags: [课内, 计算智能]
description: 中南大学自动化学院《计算智能》课程课件的中文精读版。按课件的十二个章节讲：进化算法框架、GA、ES/EP、DE、PSO、术语体系、算法比较方法、多目标优化，以及约束优化问题的求解。每处结论都链回课件 PDF 的对应页。
source_url: http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/
toc: false
draft: false
---

```cardlink
url: http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/课件/
title: "计算智能 · 课程课件"
description: "中南大学自动化学院《计算智能》课程的全部课件"
host: 10.188.31.55:5244
```

- 第 0 章 开篇 — 课程信息与提纲、计算智能的定义与应用范围、课程主线、九个研究方向、重要性与现状、讲授方式与考试方式
- 第 1 章 计算智能与进化算法导论 — 优化问题的三要素、适应度地形与局部/全局最优、多点并行与随机转变准则、EA 通用框架与贯穿术语
- 第 2 章 遗传算法 (GA) — 编码与解码、轮盘赌选择、交叉与变异、模式定理、实数编码
- 第 3 章 进化策略 (ES) 与进化规划 (EP) — (1+1)-ES、加号与逗号选择、步长自适应、CMA-ES、1/5 成功法则
- 第 4 章 差分进化 (DE) — 差分变异、二项式与指数交叉、贪婪选择、CoDE 与 QOX/OXDE
- 第 5 章 粒子群优化 (PSO) — 速度与位置更新、惯性权重、gbest/lbest 拓扑、OL-PSO
- 第 6 章 计算智能术语体系 — 探索与开发、全局与局部、多样性与收敛性等六个对子
- 第 7 章 算法比较方法 — 基准函数、FES 预算、成功率与收敛速度、Wilcoxon 秩和检验
- 第 8 章 多目标进化算法 — 帕累托支配、非支配排序、拥挤度距离、NSGA-II、MOEA/D、IGD 与 HV
- 第 9 章 单目标约束优化问题的进化算法求解 — 约束条件的表达与进化算法中的约束处理策略
- 第 10 章 多目标优化中多个最优解的定位 — 多目标情形下多个最优解的定位方法
- 第 12 章 多目标约束优化问题的进化算法求解 — 多目标与约束并存时的处理方式与求解流程

### 四大机制对照表

四种算法的关键差异一次性摆在下表，每一行都能直接用来做选型判断 [Chapter 2_ Introduction to GAs.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%202_%20Introduction%20to%20GAs.pdf#page=7) [Chapter 3_ Introduction to ES and EP.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%203_%20Introduction%20to%20ES%20and%20EP.pdf#page=4) [Chapter 4_ Introduction to Differential Evolution.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%204_%20Introduction%20to%20Differential%20Evolution.pdf#page=5) [Chapter 5_ Introduction to Particle Swarm Optimization.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Compute-Intelligence/converted/Chapter%205_%20Introduction%20to%20Particle%20Swarm%20Optimization.pdf#page=7)。

本文用 $N$ 表示通用种群规模；DE 写作 $NP$，ES 的父代数写作 $\mu$，比较父代种群时 $N = NP = \mu$，且 $\lambda \ge \mu$。

| 维度 | GA | ES / EP | DE | PSO |
| --- | --- | --- | --- | --- |
| 个体表示 | 二进制串（实数问题需先编码） | 实数向量 $x$；ES 还含步长 $\delta$ | 实数向量 $x$ | 实数向量 $x$（称「粒子位置」） |
| 主要算子 | 选择 + 单点交叉 + 位翻转变异 | 高斯变异 $\delta\cdot N(0,1)$；ES 还有交叉 | 差分变异 + 二项式交叉 | 速度更新 + 位置更新 |
| 新解来源 | 父代串某段互换 + 每一位小概率翻转 | 父代 + 一个正态扰动 | 基向量 + $F$ × 差分向量 + 交叉 | 上一步位置 + 刚算出的速度 |
| 选择机制 | 轮盘赌 / 锦标赛（整代排序） | $(\mu{+}\lambda)$ 或 $(\mu,\lambda)$（整代排序） | 一对一贪婪（只与自己的旧解比） | **无显式选择**；粒子永不淘汰 |
| 关键参数 | $p_c$，$p_m$，$L$（串长） | $\mu$，$\lambda$，初始 $\delta$（ES 还含 $\tau$） | $F$，$CR$，$NP$ | $\omega$，$c_1$，$c_2$，$NP$ |
| 是否需要编码 | 是（连续问题需编码成串） | 否（直接实数） | 否 | 否 |
| 适合的问题 | 离散 / 组合 / 中低维连续 | 实数连续，可配合 CMA-ES 解强耦合 | 实数连续，参数少、上手快 | 实数连续，有记忆，适合动态 / 带约束 |
| 不适合的问题 | 高维连续（串长爆炸） | 离散（实数算子无意义） | 离散；强多峰易早熟 | 纯离散（粒子位置本就是实数） |
| 典型失效模式 | 串长不够精度上不去；$p_m$ 太小早熟 | $\delta$ 不会自适应 → 卡死；$(\mu{+}\lambda)$ 易早熟 | 贪婪选择 + 种群聚拢 → 早熟 | gbest 拓扑 → 社会吸引过强，整个种群跟随一个粒子 |

**怎么读这张表**：先看「主要算子」一行。这是四种算法最本质的分歧，其余各行都是它推出来的。GA 用「位翻转 + 单点交叉」，ES 用「高斯扰动」，DE 用「差分向量 + 交叉」，PSO 用「速度更新」。算子不同决定了参数不同（对应「关键参数」一行）、适用域不同、典型失效模式不同。

## 章节

- [第 0 章 开篇](<第 0 章 开篇>)
- [第 1 章 计算智能与进化算法导论](<第 1 章 计算智能与进化算法导论>)
- [第 2 章 遗传算法 (GA)](<第 2 章 遗传算法 (GA)>)
- [第 3 章 进化策略 (ES) 与进化规划 (EP)](<第 3 章 进化策略 (ES) 与进化规划 (EP)>)
- [第 4 章 差分进化 (DE)](<第 4 章 差分进化 (DE)>)
- [第 5 章 粒子群优化 (PSO)](<第 5 章 粒子群优化 (PSO)>)
- [第 6 章 计算智能术语体系](<第 6 章 计算智能术语体系>)
- [第 7 章 算法比较方法](<第 7 章 算法比较方法>)
- [第 8 章 多目标进化算法](<第 8 章 多目标进化算法>)
- [第 9 章 单目标约束优化问题的进化算法求解](<第 9 章 单目标约束优化问题的进化算法求解>)
- [第 10 章 多目标优化中多个最优解的定位](<第 10 章 多目标优化中多个最优解的定位>)
- [第 12 章 多目标约束优化问题的进化算法求解](<第 12 章 多目标约束优化问题的进化算法求解>)
