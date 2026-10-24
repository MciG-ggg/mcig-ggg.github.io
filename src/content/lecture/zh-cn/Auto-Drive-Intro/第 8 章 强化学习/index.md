---
title: 第 8 章 强化学习
timestamp: 2026-10-24T11:00:00+08:00
series: Auto-Drive-Intro
tags: [课内, Auto-Drive-Intro]
description: 强化学习是通过和环境交互学习，学习到如何在相应观测中采取最优策略的行为。三个特点：不需要标注；具有鲁棒性；对行为的学习更友好 第八章 无人驾…
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

## 第 8 章 强化学习

### 强化学习基础

#### 强化学习的基本框架

强化学习是通过和环境交互学习，学习到如何在相应观测中采取最优策略的行为。三个特点：不需要标注；具有鲁棒性；对行为的学习更友好 [第八章 无人驾驶强化学习.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=4)。

七个术语构成完整回路：Environment 是整体任务的工作环境；Reward 是激励，对行为好坏的评价；Policy 是智能体的一组行为；Agent 是智能体，一般是 RL 的作用对象；Action 是智能体可以采取的所有可能行动；Sensors 返回环境当前情况；MDP 模型是马尔可夫决策过程 [第八章 无人驾驶强化学习.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=4)。

奖励设计对强化学习尤其重要，不同环境可以有不同的奖励 [第八章 无人驾驶强化学习.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=4)。这是 RL 落地最容易翻车的地方：奖励写歪一点，学出来的策略就朝着人类不想让它去的方向走，而且训练指标看着还在涨。

#### 逆向强化学习与模仿学习

逆向强化学习要找一种高效可靠的 Reward 方法。思路是专家在完成任务时其决策往往最优或接近最优，当所有策略的累积回报期望都小于专家策略时，对应的回报函数就是从示例学到的回报函数 [第八章 无人驾驶强化学习.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=5)。学徒学习的两条算法是最大边际规划算法 Max Margin Plan 和基于最大熵的逆向强化学习 [第八章 无人驾驶强化学习.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=5)。

模仿学习从专家提供的范例中学习，一般是人类专家的决策数据，每个决策包含状态和动作序列，把状态作为特征、动作作为标记做分类或回归，从而得到最优策略模型。目标是让模型生成的状态-动作轨迹分布和输入的轨迹分布相匹配。它属于监督学习方法（行为克隆），特点是泛化性差、依赖大量数据，需要数据增广 [第八章 无人驾驶强化学习.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=6)。行为克隆的病根是分布偏移：训练时看到的是专家轨迹，测试时看到的是自己的轨迹，一旦偏了一点就回不来。

#### 强化学习的分类

深度学习提供感知能力但缺乏决策能力，强化学习提供决策能力，非常适合用作无人驾驶的决策规划 [第八章 无人驾驶强化学习.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=7)。

三组分类轴：Model-based 与 Model-free；On-Policy 与 Off-Policy；SARSA 与 Q-Learning [第八章 无人驾驶强化学习.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=7)。SARSA 和 Q-Learning 的区别落在「用哪个动作去估下一状态的最大值」：SARSA 是 on-policy，用实际选出的动作；Q-Learning 是 off-policy，直接取最大值。

常见网络里有 DQN（端到端的学习方式，深度卷积神经网络加 Q 学习，带经验回放技术）和 DDPG（actor-critic 算法，用深度神经网络作为逼近器）[第八章 无人驾驶强化学习.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=7)。

#### 无人车与强化学习

感知端用 Radar、Lidar、GPS、Camera，抽出高级特征，由机器学习把特征转化为知识，知识处理后进行推理，做出合理的动作决策。课件追问「有多少是可以通过学习得到的」，并点到 Mapping 这个难点 [第八章 无人驾驶强化学习.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=8)。

](images/fig-08-p08.png)

#### Q-table 与 Q-Learning

Q-table 为每一个 state 上的每一个 action 计算出最大的未来 reward 期望，表中每一格直接给出了最佳策略下在该状态采取该动作能获得的最大未来奖励期望。课件给的例子每个 state 允许四种操作：左移、右移、上移、下移 [第八章 无人驾驶强化学习.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=9)。

Q-Learning 的更新步骤是：选取一个动作（基于当前 Q 值估计在状态 state 下选下一个动作 action）；采用该动作，观察输出的状态 $s'$ 和奖励 reward；然后更新。更新公式把「当前 Q 值」「奖励」「下一状态的最大期望未来奖励」三项组合起来，学习率 $\alpha$ 控制新信息的吸收速度，折扣因子 $\gamma$ 控制未来的权重：

$$
Q(s,a) \leftarrow Q(s,a) + \alpha\left[R + \gamma \max_{a'} Q(s',a') - Q(s,a)\right]
$$

[第八章 无人驾驶强化学习.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=10)

#### 从 Q-table 到 DQN

价值的定义是未来折扣累积回报，$R_t$ 是从第 $t$ 步起的奖励按折扣加权求和：

$$
R_t = r_t + \gamma r_{t+1} + \gamma^2 r_{t+2} + \cdots
$$

DQN 的设定是 4 个 action（上下左右），每走一步有 Reward，计算未来 Reward 的最大总和 [第八章 无人驾驶强化学习.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=12)。

State-action value function $Q^\Pi(s,a)$ 在 Q-Learning 里用最佳策略来最大化未来的 Reward 值，是 off-policy 的，持续迭代更新每一步的 $(s,a)$ [第八章 无人驾驶强化学习.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=13)。

](images/fig-08-p13.png)

探索与使用的两难是这一节的核心：贪心是局部最优，初期对信息一无所知 [第八章 无人驾驶强化学习.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=13)。

持续更新 $(s,a)$ 是 Q-Learning 的最大问题：效率低、泛化能力差，对复杂问题会出现维度爆炸。解法是用深度学习代替 Q-Table [第八章 无人驾驶强化学习.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=14)。

#### DQN 的网络结构与训练

DQN 的骨架来自 DeepMind 的 Nature 论文，网络是端到端的：输入 84×84×4 的四帧灰度堆叠，经三层卷积（8×8/stride 4/32 个滤波器、4×4/stride 2/64 个、3×3/stride 1/64 个，均用 ReLU）降到 7×7×64，接一个 512 单位的全连接层，最后一层线性输出 18 个动作值 [第八章 无人驾驶强化学习.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=15)。

损失函数是目标 Q 值与当前 Q 值之差的平方，括号里的 $r + \gamma\max_{a'} Q(s',a')$ 是回归目标：

$$
L = \mathbb{E}\left[\left(r + \gamma \max_{a'} Q(s',a') - Q(s,a)\right)^2\right]
$$

[第八章 无人驾驶强化学习.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=15)

训练流程五步：给定状态转移方式 $\langle s,a,r,s'\rangle$；在状态 $s$ 用前向推理计算所有 action 的预测 Q 值；前向推理下一步 $s'$ 及其最大 $Q'$ 值 $\max_{a'}Q(s',a')$；把 Q 值设为 $r + \gamma\max_{a'}Q(s',a')$；用 loss function 做梯度更新 [第八章 无人驾驶强化学习.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=16)。

经验回放解决数据利用率和数据相关性两个问题：提高数据利用率，遗忘数据流顺序（加入随机性）[第八章 无人驾驶强化学习.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=16)。另外三条工程技巧：Learning 优化每 1000 步才优化一次 Loss function；Reward Clipping 把奖励截断，防止个别大奖励主导梯度；Skip Frames 跳帧采样 [第八章 无人驾驶强化学习.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=16)。

课件对伪代码补了一句很关键的提醒：那里的循环迭代不是训练网络，而是更新 Q 值和观测结果，需要把这些观测和 Action 存下来，再去更新网络。Epsilon 是探索周围空间的概率，对应探索和使用的两难 [第八章 无人驾驶强化学习.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=17)。

#### Deep Traffic 项目

Deep Traffic 对应 MIT 6.S094，配套论文是 Deep Traffic: Crowdsourced Hyperparameter Tuning of Deep Reinforcement Learning Systems for Multi-Agent Dense Traffic Navigation [第八章 无人驾驶强化学习.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=19)。多智能体密集交通导航加上众包调参，是这一章里离自动驾驶最近的一个可动手的工程。

### 强化深度学习案例

#### 案例的三个问题

强化深度学习案例这份报告做三件事，每件事对应一个具体痛点 [第八章 强化深度学习案例.pdf · p.3](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=3)：

- 多机器人协同清扫任务要求覆盖率高、重复率低、任务量一致，对应面向多目标的清扫机器人协同覆盖路径规划方法。
- 环境参与者增多带来大量环境信息，机器人信息处理困难进而决策失效，对应基于混合注意力导向经验回放的围捕机器人协同决策规划方法。
- 基于参数化动作空间的行为决策中，离散动作与连续参数存在耦合，对应基于双通道奖励评估的足球机器人协同进攻决策方法。

#### 多机器人行为决策的方法现状

基于规则的方法包括博弈论、模糊推理、有限状态机、行为决策树，优点是可解释性，缺点是规则完备性要求高、需要专家经验。基于学习的方法是深度强化学习方法，分 DQN 方法簇和 PG 方法簇，优点是泛化性强、能应对复杂环境，缺点是可解释性弱 [第八章 强化深度学习案例.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=4)。

多机器人深度强化学习分独立式方法和集中式方法。独立式方法用独立 Q 学习、并行深度确定性策略梯度，打破了环境静态性的假设 [第八章 强化深度学习案例.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=4)。

两个研究缺口各自有对应的技术路线。大量环境信息输入问题：建立通信通道、建立分层结构、用注意力机制。参数化动作空间决策：P-DDPG、P-DQN、MP-DQN、Hybrid Actor-Critic、Deep MAPQN、Deep MAHHQN [第八章 强化深度学习案例.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=5)。

路径规划方法这边，基于搜索的方法有图搜索（Dijkstra、A*、D*）、随机采样方法（概率图法、快速搜索随机树法 RRT）、智能优化算法（粒子群、萤火虫、遗传算法、引力搜索），学习方法则是深度强化学习方法 [第八章 强化深度学习案例.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=6)。

#### 清扫机器人：栅格覆盖路径规划

已知栅格地图下的多目标覆盖路径规划，坐标系有三套：地图坐标系 $\{R\}$、栅格坐标系 $\{G\}$、世界坐标系 $\{W\}$，靠 Costmap2D::worldToMap() 做转换 [第八章 强化深度学习案例.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=8)。

栅格按类型分三类：已覆盖栅格或不可行栅格、其他类型栅格、第三类。评价某个栅格的隶属度用两个指标：路径方向变化角度；路径发散程度。后者根据机器人当前所在位置 8 个方向的栅格与机器人起点的距离制定两两比分评分表，越靠近起点的栅格被选为下一个路径点的可能性越大 [第八章 强化深度学习案例.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=8)。

规划流程是一个带回环的流程图：上位机获取全局地图和定位信息 → 地图预处理 → 计算周围 8 个栅格的隶属度函数 → 计算栅格模糊评价矩阵 → 若机器人陷入死区，把栅格大小调整为 $L_2$，以死区点为起点、最近的未覆盖栅格为终点，用 A* 规划一条逃离路径 → 若所有自由栅格都已划分为已覆盖栅格则结束，否则寻找最近的未覆盖栅格，把隶属度最高的栅格坐标转换至全局坐标加入路径点集合 → 最后调整当前位置、生成最终覆盖路径并优化 [第八章 强化深度学习案例.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=8)。

#### 清扫机器人：遗传算法任务分配

任务分配的目标是任务一致性和同步性高，把覆盖路径划分成若干段（段数与集群机器人数量相同），分别作为各机器人的覆盖任务。做法是获取当前可工作机器人数量，把覆盖路径自适应划分成同样多的段，用遗传算法搜索路径的最优划分点，每一段的路径长度方差作为个体的适应度函数，让每台机器人分配到的任务量相当 [第八章 强化深度学习案例.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=9)。

遗传算法流程：基于覆盖路径生成基划分点序列 $P_0$ → 在 $P_0$ 基础上替换某些路径点产生 $q$ 个序号序列作为初始种群 → 计算每个个体的适应度 → 按轮盘赌选择个体 → 两两配对的个体按交叉概率 $P_{cross}$ 交叉 → 个体按变异概率 $P_{mutate}$ 变异 → 达到迭代次数则生成最优路径划分点序列，把路径分成 $m$ 份作为各机器人的覆盖任务路径 [第八章 强化深度学习案例.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=9)。

#### 清扫机器人：系统实现与实验

硬件是每台机器人搭载 X86 工控板、STM32 控制器、IMU 惯性导航、通信路由器、激光雷达 TIM561、超声波传感器等，可原地旋转，用在室内平整地面，俯视直径约 0.5 m。软件基于 Ubuntu 16.04 下的 ROS kinetic 和 Rviz 可视化平台联合开发 [第八章 强化深度学习案例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=10)。

](images/fig-08-p11.png)

栅格大小对覆盖效果的影响，实验数据是：

| 栅格大小 L/m | 覆盖率 | 重复率 | 路径长度/m |
| --- | --- | --- | --- |
| 0.5 | 70.70% | 0.32% | 378.6 |
| 0.6 | 66.09% | 0.18% | 291.35 |
| 0.7 | 63.55% | 0.35% | 251.7 |
| 0.8 | 61.5% | 0.32% | 213.25 |

栅格越小，规划路径越密集，越能靠近障碍物；栅格越大，路径间距越大，离障碍物越远 [第八章 强化深度学习案例.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=13)。重复率始终保持在 0.3%–0.4% 以内，靠的是机器人陷入死区时用变尺度 A* 规划逃离路径，适时调整规划栅格大小，让路径更短更平滑 [第八章 强化深度学习案例.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=13)。这是覆盖率与路径长度之间的直接取舍：覆盖率最高的那档路径长度多了 80%。

任务分配的一致性随机器人数量的变化很小，4 到 7 台机器人时任务量方差都在 4–5 m² 以内（4.11749、4.85304、4.42349、4.44970），收敛曲线显示遗传算法收敛较快，能找到最优路径划分点 [第八章 强化深度学习案例.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=14) [第八章 强化深度学习案例.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=15)。

#### 围捕机器人：多机器人 MDP 与模型

多机器人协同决策可以看作部分可观测马尔可夫决策过程。$n$ 个机器人的 MDP 由元组 $(S,O,A,\gamma,T,P,R)$ 描述，粗体表示多机器人联合变量：环境的全局状态 $s \in S$ 由各机器人的部分观测状态 $o_i \in O$（$i = 1,\dots,n$）组成；各机器人按决策策略 $\pi: O \times A \to [0,1]$ 选择动作 $a_i \in A$，形成联合动作集合 $\boldsymbol{a} = [a_1,a_2,\dots,a_n]$；联合动作对环境产生影响，环境返回联合奖励 $\boldsymbol{r} = [r_1,r_2,\dots,r_n]$；$\gamma$ 是折扣因子，$T$ 表示步数，$P$ 是马尔可夫状态转移概率。联合累计回报是各机器人各时刻折扣回报之和：

$$
R = \sum_{t=1}^{T}\gamma^{t}\sum_{i=1}^{n} r_i^{t}
$$

目标是在该策略下联合累计回报最大，并完成多机器人协同围捕任务 [第八章 强化深度学习案例.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=17)。

模型设计分三块：状态空间、动作空间、奖励函数。单个机器人的局部观测状态包括自身位置和速度、其他机器人的相对位置、对手的相对位置和相对速度、地标的相对位置。动作空间定义为 x 和 y 方向上的加速度，机器人和对手的初始速度都设为 0；对手的最大速度和最大加速度都略高于机器人，这样有利于对手挣脱、机器人实现二次追捕 [第八章 强化深度学习案例.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=18)。奖励由三部分相加：同态奖励、异态奖励和一个带系数 $\epsilon$ 的协同奖励：

$$
R = \sum_{i=1}^{n}\left(r_i^{ss} + r_i^{dd} + \epsilon\, r_i^{cc}\right)
$$

[第八章 强化深度学习案例.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=18)

#### 混合注意力模块与 MADDPG

混合注意力模块先对机器人的状态和动作分别编码，得到低维细密度的表示；再由共享矩阵 $Q$ 和 $K$ 把嵌入编码线性变换为查询 $q$ 和键值 $k$；然后构造一个额外的 MLP 感知器 $M$，以查询、键值及两者按键值维数比例伸缩点积为输入计算相似度并归一化 [第八章 强化深度学习案例.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=19)。

其他机器人的贡献度由价值加权总和组成，查询与键值的相似度经过比例伸缩和 Softmax 得到权重：

$$
q_i = Qg(s_i), \qquad k_j = Kh(s_j), \qquad w_{ij} = \mathrm{softmax}_j\!\left(q_i \cdot k_j\right), \qquad c_i = \sum_{j \ne i} w_{ij}\, Vh(s_j)
$$

[第八章 强化深度学习案例.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=19)

基础框架是多智能体深度确定性策略梯度 MADDPG，假设所有机器人都处于相同的动态环境中。它采用集中式训练和分布式执行：每个机器人都有一个集中的 Critic 网络接收全局环境状态，Actor 网络只接收局部观测状态；Actor 和 Critic 都由全连接网络组成，激活函数为 ReLU [第八章 强化深度学习案例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=20)。改进有两处：一是在 Critic 网络里插入混合注意力模块，分别对环境状态和动作信息投入不同程度的注意力，加强 Critic 的信息处理能力；二是采用多维度混合注意力，各维度专注于不同方面的信息，维度之间参数（$Q,K,V,M$）不共享，最后生成基于不同维度的其他机器人的贡献加权和，再聚合各维度得到 $h$ 维的贡献 [第八章 强化深度学习案例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=20)。

#### 混合注意力导向的优先经验回放

做法是把优先经验回放扩展到多智能体领域，用各机器人的时间差分偏差加权和衡量经验重要性，再把注意力权重线性变换为经验样本重要性的权重。样本 $e$ 的采样概率按优先级取：

$$
P(e) = \frac{p_e^{\beta}}{\sum_{l} p_l^{\beta}}
$$

其中 $p_e = 1/r(e)$ 表示样本重要性排序，$l$ 是样本总数，$\beta$ 表示使用优先级的程度，$\beta = 0$ 时表示均匀采样 [第八章 强化深度学习案例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=21)。重要性的度量来自时间差分偏差，机器人 $i$ 的偏差是即时奖励加上折扣后的目标 Q 值减去当前 Q 值：

$$
\delta_i = r_i + \gamma Q(s_{i,t+1}, a_{i,t+1}) - Q(s_{i,t}, a_{i,t})
$$

注意力权重矩阵由多头结果求和再归一化，样本重要性权重综合状态矩阵和动作矩阵 [第八章 强化深度学习案例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=21)。

#### 围捕机器人的实验结果

围捕机器人的实验结果，三种算法在 3 vs 1 场景、注意力维度 $h=4$ 下的表现为：MADDPG 平均总奖励 342.30、收敛周期 20,000；HA-MADDPG 577.44、24,000；HAER-MADDPG 632.17、15,000 [第八章 强化深度学习案例.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=22)

](images/fig-08-p22.png)

注意力维度 $h$ 的影响：$h=1$ 时平均总奖励 487.19、收敛周期 30,000；$h=2$ 时 499.48、30,000；$h=4$ 时 577.44、24,000 [第八章 强化深度学习案例.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=23)。同一张表里，PERHA-MADDPG 拿到 626.53，HAER-MADDPG 632.17，说明优先经验回放这一项贡献有限 [第八章 强化深度学习案例.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=23)。扩展性实验在 3 vs 1、3 vs 2、4 vs 1、4 vs 2 四种配置下与 COMMNET、BICNET 对比，HAER-MADDPG 全部最高且收敛周期最短：3 vs 2 时 HAER-MADDPG 381.15 / 30,000，MADDPG 只有 280.65 / 48,000；4 vs 2 时 491.47 / 48,000 对 199.00 / 60,000 [第八章 强化深度学习案例.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=24)。机器人和对手越多，差距越大，这正是「注意其他机器人」这套机制的收益来源。可视化实验看了机器人 1 号在 5000、10000、20000 次训练周期时对机器人 2 号和 3 号的对比关注度，分别对应策略探索初期、学习有效策略时期和最终收敛的时期：第 5000 次时关注度均衡，随训练周期增长差异逐渐拉开，说明它开始学会有侧重地关注环境信息；而且对状态的关注度差异比对动作的明显，这表明对状态和动作分别关注是有必要的 [第八章 强化深度学习案例.pdf · p.25](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=25)。

实车实验里机器人以 0.2 m/s 匀速运动，对手静止，所以不存在二次追捕；局部观测状态简化为自身位置、其他机器人相对位置、对手相对位置和地标相对位置。动作空间是向周围八个方向移动一个栅格以及原地停止，用 Gumbel-SoftMax 重参数技巧让 HAER-MADDPG 能用在离散动作空间上 [第八章 强化深度学习案例.pdf · p.26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=26)。连续策略梯度方法要落到离散动作上，靠的是 Gumbel-SoftMax 这个可微重参数技巧。

#### 足球机器人：环境与参数化动作空间

基于半场进攻足球环境 HFO（Half Field Offense），它基于 RoboCup 2D 服务器实现二维半场足球进攻任务，比赛队伍支持由非玩家控制的内置 AI 队友（NPCs）和玩家控制的机器人任意组合。HFO 囊括连续状态空间和离散、连续及参数化的动作空间，具备部分观测和完全观测状态选择和进攻防守角色转换的能力，提供高级和低级两种状态空间，低级状态空间包含 58 个详细状态特征以及对手及队友的状态特征 [第八章 强化深度学习案例.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=29)。

参数化动作空间把离散动作和连续参数绑在一起：踢球带 targetx、targety、speed 三个连续参数，以一定速度把球踢到指定坐标；移动带 targetx、targety，以最大冲刺速度移动到指定位置；运球同样带 targetx、targety，未接触球则先尝试获取球权；射门没有参数。

| 动作 | 参数 | 取值范围 |
| --- | --- | --- |
| 踢球 | targetx, targety, speed | 连续 |
| 移动 | targetx, targety | 连续 |
| 运球 | targetx, targety | 连续 |
| 射门 | 无 | 无 |

[第八章 强化深度学习案例.pdf · p.30](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=30)。

#### 足球机器人的奖励函数

比赛在四个条件之一时结束：进球；足球出界；球被拦截；已达到比赛最大时间限制。奖励设计除了从距离层面给奖给罚，还深入到角度层面 [第八章 强化深度学习案例.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=31)。

五个分项各管一段：球距奖励按进攻机器人与足球的距离给奖励，距离用 $1/(d(a,b)-1)$ 这种形式压缩；踢球奖励在第一次获取球权时给；传球奖励在球权转移到队友时给；进球奖励是终止状态奖励；射角奖励按进攻机器人与防守机器人的射角差给，与获取球权与否无关 [第八章 强化深度学习案例.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=31)。

最终奖励是这些项的加权和，其中一项带乘性系数：

$$
r = r_{end} + r_{mtb} + r_{kick} + r_{ktg} + r_{pass} + r_{kickangle}
$$

[第八章 强化深度学习案例.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=31)

#### MAPDDPG 与双通道 Critic

MAPDDPG 把 HAER-MADDPG 扩展到多机器人协同任务领域。Actor 网络以机器人 $i$ 的局部观测状态信息为输入，经过一个全连接网络模块后输出各离散动作值和相应的连续动作参数值，机器人执行最大离散动作值对应的离散动作及其相应的连续动作参数。为了实现集中式训练，Critic 网络以全局信息 $s$、其他机器人所有离散动作值集合 $u$ 和相应的所有连续动作参数集合 $p$ 为输入，最终输出单个标量状态-混合动作值 [第八章 强化深度学习案例.pdf · p.32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=32)。BI-MAPDDPG 的动机来自人的决策习惯：先决策离散动作，再决策连续动作参数，且离散动作对连续动作参数有决定作用。改进有两处：一方面 Actor 网络的连续动作参数决策不仅仅以局部状态为输入，还考虑了离散动作值输入；另一方面设计双通道奖励评估的 Critic 网络，分别评估状态-离散动作值和状态-混合动作值，通过自适应调整在整个决策奖励或惩罚中离散动作作用所占的权重，对状态-离散动作价值进行评估 [第八章 强化深度学习案例.pdf · p.34](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=34)。Critic 网络的实现是：输入全局观测状态 $s$、其他机器人所有离散动作值集合 $u$ 和连续动作参数集合 $p$，经过全连接网络模块后引申出两条分支，一支经过一层线性网络输出状态-离散动作值，另一支融合状态-离散动作值后经过一层线性网络最终输出状态-混合动作值 [第八章 强化深度学习案例.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=35)。

离散动作权重 $\lambda$ 与其在整个决策过程中发挥的作用成正比，取值范围 $[0,1]$，并设计成随训练周期自适应调整。$\lambda_{max}$ 和 $\lambda_{min}$ 分别是离散动作奖励权重的最大值和最小值，episode 表示训练周期，$c$ 为常数，取连续动作发挥主要作用的期望周期。训练早期 episode 小时的 $\lambda$ 较大，离散动作主导；后期逐渐降到 $\lambda_{min}$，连续参数主导。Critic 网络损失函数写成两部分之和：

$$
L = L_1(\phi_1) + L_2(\phi_1, \phi_2, \phi_3)
$$

[第八章 强化深度学习案例.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=35)

#### 足球机器人的实验结果

射角奖励的作用可以量化。2v1 场景下，不含射角奖励的模型进球占比 0.880、被拦截 0.088、超界 0.031、超时 0.0012；含射角奖励的模型进球占比 0.910、被拦截 0.067、超界 0.022、超时 0.0000 [第八章 强化深度学习案例.pdf · p.36](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=36)。两种模型都能到 0.88 的进球占比，射角奖励做的是降低被拦截的概率、提高进球命中率，从而加快学习协同进攻策略的收敛 [第八章 强化深度学习案例.pdf · p.36](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=36)。

离散动作权重最小值 $\lambda_{min}$ 的扫描结果：0.5、0.6、0.7、0.8、0.9 对应的进球占比依次是 0.751、0.816、0.859、0.910、0.876，平均奖励依次是 5.815、6.356、6.506、6.727、6.659 [第八章 强化深度学习案例.pdf · p.37](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=37)。0.8 效果最佳，0.9 次之。原因是前期寻球运球过程对最终进球起关键作用，所以前期能学到正确的离散动作策略至关重要，取值不能太小；而后期训练中连续动作参数在最终踢球进洞过程中发挥更大作用，为了区分状态-离散动作值和状态-混合动作值的更新，取值不宜过大 [第八章 强化深度学习案例.pdf · p.37](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=37)。这一条把「奖励权重」和「训练阶段」绑在一起，是双通道设计的动机。

横向对比几个参数化动作空间的方法，2v1 进攻场景：P-DDPG 收敛进球占比 0.495、开始收敛周期 5000；MAPDDPG 0.542、4000；P-DQN 0.1、无法收敛；Deep MAPQN 0.5、7500；Deep MAHHQN 0.55、10000；BI-MAPDDPG 0.654、7500 [第八章 强化深度学习案例.pdf · p.38](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=38)。

四种场景的最终对比，BI-MAPDDPG 在每个场景都最好，场景越难差距越大（3v2 时 P-DDPG 进球占比只剩 0.252，BI-MAPDDPG 还有 0.762）[第八章 强化深度学习案例.pdf · p.42](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=42)：

| 场景 | P-DDPG 进球/奖励 | MAPDDPG 进球/奖励 | BI-MAPDDPG 进球/奖励 |
| --- | --- | --- | --- |
| 2v1 | 0.703 / 5.630 | 0.831 / 5.829 | 0.910 / 6.727 |
| 3v1 | 0.525 / 3.654 | 0.707 / 4.757 | 0.883 / 6.419 |
| 2v2 | 0.436 / 3.703 | 0.787 / 5.757 | 0.843 / 6.426 |
| 3v2 | 0.252 / 2.189 | 0.538 / 3.713 | 0.762 / 5.838 |

#### 展望

三个方向：覆盖路径规划通过曲线拟合等方式平滑路径提高应用泛化能力，采用平行障碍物边缘线等方法让覆盖路径尽量贴近静态障碍物；围捕决策面向基于规则或基于学习策略的对手展开研究，对手由动态障碍物或机器人实现，协同围捕任务由具有可变速度的机器人完成；离散动作权重参数的表现形式多样化，从线性函数到非线性函数，甚至以网络学习的方式自适应调整，并扩展到协同防守决策领域 [第八章 强化深度学习案例.pdf · p.43](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=43)。
