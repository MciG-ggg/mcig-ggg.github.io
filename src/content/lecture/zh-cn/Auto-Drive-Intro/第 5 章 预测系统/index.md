---
title: 第 5 章 预测系统
timestamp: 2026-10-24T11:00:00+08:00
series: Auto-Drive-Intro
tags: [课内, Auto-Drive-Intro]
description: PNC 是 Planning and Control，课件把自动驾驶软件栈按数据流画了出来：感知、定位、HD-Map 产出环境与自车状态，预…
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

## 第 5 章 预测系统

### 预测系统

#### PNC 在软件栈里的位置

PNC 是 Planning and Control，课件把自动驾驶软件栈按数据流画了出来：感知、定位、HD-Map 产出环境与自车状态，预测模块输出他车与行人的未来轨迹，Routing 给路径，Planning 分决策、路径、速度三层，Control 出控制量，Canbus 执行 [第五章 无人驾驶预测系统.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=5)。

](images/fig-05-p05.png)

预测的位置是感知和规划之间的那一格：感知给出「现在有什么」，预测给出「下一刻会怎样」，规划据此决定自车怎么走。行业侧的度量是 2022 年 DMV 自动驾驶报告里 Cruise、AutoX、Zoox、WeRide、DIDI 的平均接管里程（MPI）对比 [第五章 无人驾驶预测系统.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=6)。MPI 反映的是预测错了多少、规划因此要接管的频率。

#### 预测任务本身

为什么需要预测：不同类型的交通参与者需要不同类型的预测；要求是实时的、准确的；还要能通过更多 corner case 学会处理罕见场景。人是怎么判断的，这是模型要模仿的对象 [第五章 无人驾驶预测系统.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=8)。

做预测有两条路线 [第五章 无人驾驶预测系统.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=9)：

- Model-based：结合人类知识，能解 corner case。流程是行为模型给出 Turn Right、Go Straight 这样的模型，各自产生一条轨迹，更新概率，输出预测轨迹集合。
- Data-driven：机器学习，靠更多数据。流程是观测行为加训练数据进机器学习模型，直接输出预测轨迹。

](images/fig-05-p09.png)

#### 车辆预测：从车道模型到网络结构

车辆预测的第一层抽象是 Lane Model。Lane sequence 有三种来源：HD Map、Junction、Off lane。分类是在路口处把可能的车道连接关系离散成几个类别，比如 Lane0 → Lane1 → Lane3 → Lane7、Lane0 → Lane2 → Lane6、Lane0 → Lane4 → Lane5，再给每类配一个搜索概率 [第五章 无人驾驶预测系统.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=11)。

网络吃三类输入 [第五章 无人驾驶预测系统.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=12)：

- 车道特征：参考车道、参考车道的曲率、Traffic law。
- 车辆状态：速度、加速度、朝向、朝向角速度、类型、尺寸。
- 环境：周围交通参与者的分布。

网络结构是两级：上层 Network 出 Behavior，下层 Sequence data 出轨迹序列，用 RNN 实现 [第五章 无人驾驶预测系统.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=13)。序列数据的网络可以从这些里挑：MLP、CNN、RNN（LSTM、GRU）、Attention、TCN [第五章 无人驾驶预测系统.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=14)。

数据管线是三段：特征工程、模型优化、样本工程 [第五章 无人驾驶预测系统.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=15)。样本工程里的自动标注流程是：带传感器数据的日志进入开发流程，标注器从历史数据产出标注数据，正负样本都要，标注后做标签清洗和标签平衡，再经过数据挖掘与主动学习选出待标注样本，形成带标注数据的数据集，模型训练后上线并收集新数据，闭环 [第五章 无人驾驶预测系统.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=16)。

轨迹构建有三种基线：卡尔曼滤波、多项式拟合、速度外推 [第五章 无人驾驶预测系统.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=17)。

两个 state of the art 值得记住结构。Uber 的方案是 CNN 模型，特征拼接、展平、光栅化，state 序列进、输出序列出，图生成用全信息，轨迹生成是回归问题 [第五章 无人驾驶预测系统.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=18)。Waymo 的 ChauffeurNet 是 mid-to-mid 学习：多任务、模仿学习，把预测和规划合在一起 [第五章 无人驾驶预测系统.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=19)。它的闭环结构值得记：数据渲染器把网络输入渲染成图，神经网络输出预测的 waypoints，控制优化把 waypoints 转成控制量，车辆在真实或仿真环境中执行，控制量更新环境，环境更新当前位姿，动态路由器据此给出新路线，路线再回灌渲染器 [第五章 无人驾驶预测系统.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=19)。

#### 行人预测

行人比车难预测，原因是三条：高随机性、交通约束低、没有运动学模型可用。常用 benchmark 是 ETH 和 UCY [第五章 无人驾驶预测系统.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=22)。

三个递进的工作。Social LSTM 用 LSTM 加 social-pooling 做人与人的交互建模 [第五章 无人驾驶预测系统.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=23)。Social GAN 用生成对抗网络做社会可接受的轨迹，同样考虑人与人的交互 [第五章 无人驾驶预测系统.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=24)。Peeking into the Feature 用多任务学习，配合栅格映射，把感知和预测结合起来 [第五章 无人驾驶预测系统.pdf · p.25](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=25) [第五章 无人驾驶预测系统.pdf · p.26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=26)。

### 预测系统实例

#### 实例一：面向未知环境的高效遍历路径规划

另一份主参考是中南大学的一份学位论文式报告，三个工作分别是面向未知环境的移动机器人高效遍历路径规划、基于群组层次的时空图神经网络行人轨迹预测、基于高斯混合模型的移动机器人社会规范化路径规划 [第五章 无人驾驶预测系统实例.pdf · p.2](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=2)。

动机是环境的未知和行人的运动不确定性给路径规划带来挑战，阻碍移动机器人落地 [第五章 无人驾驶预测系统实例.pdf · p.4](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=4)。社会规范化路径规划的定义是：在人与机器人共存的环境中，机器人在规划自身路径时充分考虑人类的行为模式和社会规范，以融入社会环境 [第五章 无人驾驶预测系统实例.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=5)。

报告给每一类方法都列了优缺点，复习时对号入座 [第五章 无人驾驶预测系统实例.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=6)：

| 方法 | 优 | 缺 |
| --- | --- | --- |
| 遍历路径规划类 | 不需要对环境有深入先验知识 | 部分环境要调参数 |
| 最短路径类 | 能在较短时间内找到最优路径 | 对环境模型要求高，需要准确建模 |
| 探索策略类 | 自适应调整探索策略，适用各种环境 | 需要大量时间和计算资源，需要调优 |
| 图神经网络行人预测 | 可解释性强，能清晰描述行人运动规律 | 难以捕捉行人运动行为的非线性和复杂性 |
| 深度学习行人预测 | 更好地处理复杂数据和场景 | 大规模图数据上计算复杂度高 |
| 社会规范路径规划 | 能较好考虑个体间社会交互和动态 | 参数设置和调优复杂，需要实验和经验确定最优参数 |

**ETPP 的问题设定。** 已有工作用路径成本和信息增益对边界点综合评估、选当下最优的探索目标，容易陷入局部最优。既然每个边界点都有探索价值，就从全局最优角度把自主探索视为边界点旅行商问题，通过遍历访问边界点完成环境遍历 [第五章 无人驾驶预测系统实例.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=8)。

系统流程是四段：外部输入经 SLAM 后，边界点生成和道路图构建（混合策略边界点生成、基于势场道路图构建），再做遍历路径规划（基于信息熵的路径规划、ACXCOOT 智能优化算法），最后是路径跟踪和更新（APG-SAC 运动控制、探索目标更新、路径更新）[第五章 无人驾驶预测系统实例.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=8)。

边界点生成与道路图构建的细节：最远探测边界由机器人位姿和探测距离算出，边界点 $p_{f1}$ 与 $p_{f2}$ 分别位于当前朝向两侧张角的一半处：

$$
\theta_{f1} = \theta - \frac{\varphi}{2}, \qquad \theta_{f2} = \theta + \frac{\varphi}{2}, \qquad x_f = x_{bot} + d_m \cos\theta_f, \qquad y_f = y_{bot} + d_m \sin\theta_f
$$

道路图用德劳内三角剖分算法生成道路边，边界点做道路节点并优化（剔除与位置优化）[第五章 无人驾驶预测系统实例.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=9)。

遍历顺序用自适应迭代循环交叉白冠鸡算法（ACXCOOT）求：普通个体重新生成边界点遍历顺序，组长用循环交叉算子移动，组长降级机制设定组长迭代次数上限，迭代次数自适应计算 [第五章 无人驾驶预测系统实例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10)。路径上顺次访问与逆序访问对应不同的代价，报告用向量余弦相似度衡量两次访问顺序的差异。

结果。相对 COOT 算法，加入 ACXCOOT 后 ETPP 的计算耗时平均降低 0.77 s（46.10%），在所有对比算法中耗时最短；把所有智能优化算法都部署到遍历路径规划模块里记录路径成本，ACXCOOT 在所有环境中路径成本都最低，且标准差几乎最小，说明求解更稳定、精度更高 [第五章 无人驾驶预测系统实例.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=15)。增大激光雷达探测半径时所有方法的时间和路径成本都显著下降，但 ETPP 仍然更好 [第五章 无人驾驶预测系统实例.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=16)。对比方法里最好的一个是 Multi-RRT [第五章 无人驾驶预测系统实例.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=13)。实验规模是十个环境、每个环境五次实验 [第五章 无人驾驶预测系统实例.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=14)。

#### 实例二：基于群组层次的时空图神经网络行人轨迹预测

这个方法（HSGNN）针对两个问题 [第五章 无人驾驶预测系统实例.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=19)：

- 问题一：已有工作在行人图建模时纳入环境中的所有行人，无关行人分散了模型对主体行为有影响的重点行人的关注度，预测结果不理想。对策是用自注意力识别重点行人。
- 问题二：人群中 70% 的行人倾向于组成群组一起行动，群组行为更复杂，深层行为特征难以学习。对策是对行人做群组划分，分别学群组内和群组间的行为。

模型分四块：行人分组模块、稀疏时空图学习（轨迹预测模块）、轨迹整合模块 [第五章 无人驾驶预测系统实例.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=19)。

](images/fig-05-p20.png)

两个设计决定要记住 [第五章 无人驾驶预测系统实例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=20)：

- 空间交互图用自注意力识别人群中对行人主体行为有影响的重点行人；时间交互图用自注意力识别对行人主体未来行为有显著影响的历史轨迹。
- 时间交互图不做时空融合，原因是每次预测过程中行人数量不定。

稀疏性靠 sparse-softmax 保证，它把普通 softmax 的指数减一，使归一化后的邻接矩阵保持稀疏：

$$
\mathrm{sparse\text{-}softmax}(z_i) = \frac{e^{z_i} - 1}{\sum_{j=1}^{k}\left(e^{z_j} - 1\right)}
$$

特征聚合的两个决定 [第五章 无人驾驶预测系统实例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=21)：

- 时间聚合特征不做卷积，因为时序层面更关注整条观测轨迹的运动趋势，卷积会把这类信息被局部聚合掩盖。
- 空间聚合特征做卷积，目的是学习相邻节点之间的交互关系。

输出层用 TCN 一次性产出整段预测轨迹，输出是双高斯分布参数（$\hat{\mu}$、$\hat{\sigma}$、$\hat{\rho}$），损失函数是负对数似然。负对数似然就是把双高斯密度取对数再取负、求和，总损失再除以人数：

$$
\mathrm{Loss}_n = -\sum_{t=1}^{T_{pred}} \log f_{\mathcal{N}}\left(x_n^t, y_n^t, \hat{\mu}_n^t, \hat{\sigma}_n^t, \hat{\rho}_n^t\right), \qquad \mathrm{Loss} = \frac{1}{N_p}\sum_{n=1}^{N_p} \mathrm{Loss}_n
$$

$$
f_{\mathcal{N}} = \frac{1}{2\pi \hat{\sigma}_{n,1}\hat{\sigma}_{n,2}\sqrt{1-\hat{\rho}_n^{2}}}
\exp\left[-\frac{1}{2\left(1-\hat{\rho}_n^{2}\right)}
\left(\frac{(x-\hat{\mu}_{n,1})^{2}}{\hat{\sigma}_{n,1}^{2}} + \frac{(y-\hat{\mu}_{n,2})^{2}}{\hat{\sigma}_{n,2}^{2}} - \frac{2\hat{\rho}_n(x-\hat{\mu}_{n,1})(y-\hat{\mu}_{n,2})}{\hat{\sigma}_{n,1}\hat{\sigma}_{n,2}}\right)\right]
$$

轨迹整合模块的做法是：编码阶段学出行人观测轨迹的特征向量并算特征相似性，用它做群组划分；解码阶段对群组内、群组间、行人个体三类预测轨迹做整合，用群组池化层和反平均池化层输出最终结果，还用直通估计器辅助训练 [第五章 无人驾驶预测系统实例.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=22)。

评价用两个指标：ADE 是预测轨迹与真值在所有时刻的平均距离误差，FDE 是终点距离误差：

$$
\mathrm{ADE} = \frac{1}{N}\sum_{i=1}^{N}\sum_{t=1}^{T}\min_k \left\| \hat{p}_i^{t} - p_i^{t,k} \right\|_2, \qquad \mathrm{FDE} = \frac{1}{N}\sum_{i=1}^{N}\min_k \left\| \hat{p}_i^{T} - p_i^{T,k} \right\|_2
$$

数据集是 ETH（ETH、HOTEL）和 UCY（UNIV、ZARA1、ZARA2），本文方法 HSGNN 与其余九种方法对比，整体表现最好 [第五章 无人驾驶预测系统实例.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=23)。原因有二：预测用的是时间卷积网络，整条轨迹一次性完成，避免了序列预测网络逐帧预测时的误差传递；对轨迹时序特征的学习让模型能预测出运动趋势。

消融实验把三个模块的价值量化了 [第五章 无人驾驶预测系统实例.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=24)：

- 去掉行人分组模块（只留个体预测）：ADE 下降 65.81%，FDE 下降 84.14%。
- 去掉时间交互图学习模块（只留空间交互图）：ADE 下降 5.98%，FDE 下降 11.25%。
- 用 softmax 换掉 sparse-softmax（邻接矩阵不再稀疏）：ADE 下降 44.25%，FDE 下降 39.94%。

可见分组模块和稀疏性是两块最大的贡献，时间交互图次之。定性结果上，HSGNN 能预测出行人群组停止移动、行人加入群组交谈这类行为 [第五章 无人驾驶预测系统实例.pdf · p.26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=26)。失败案例也交代了：六位行人组成一个群组移动的场景里群组划分结果不理想，原因可能是数据集样本不足（缺乏大规模群组样本），或者网络学不到群组的深层特征 [第五章 无人驾驶预测系统实例.pdf · p.27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=27)。

#### 实例三：基于高斯混合模型的社会规范化路径规划

这个工作要解决的是：反应式规划方法让机器人避碰行为过于突兀，而且容易闯入行人的社交空间。方案是用高斯混合模型把行人舒适空间具象化，用轨迹优化规划长时段轨迹，提高行驶平滑性 [第五章 无人驾驶预测系统实例.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=29)。

社交空间的构造依据有两条：Hall 的行人距离学把行人交互空间划成四类；人群中 70% 的行人倾向于组成群组共同行动。目标是构建一种对个体和群组、静态和动态都适用的通用社交空间，降低后端轨迹优化的实现难度 [第五章 无人驾驶预测系统实例.pdf · p.30](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=30)。高斯混合模型把行人位置的概率密度写成若干高斯分量之和：

$$
p(y) = \sum_{i=1}^{M} h_i\,\mathcal{N}(y;\, \mu_i, \Sigma_i)
$$

混合权重 $h_i$ 由各分量在当前观测点的密度归一化得到，$\mathcal{N}(y; \mu_i, \Sigma_i)$ 是均值 $\mu_i$、协方差 $\Sigma_i$ 的高斯密度。

四种模型对应四种情况 [第五章 无人驾驶预测系统实例.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=31)：

| 情形 | 判据 | 高斯构造 |
| --- | --- | --- |
| 静态个体 | $h_v = 0$ | 位置固定，两个方向的 $\sigma$ 相同（$\sigma_1 = \sigma_2 = d/2$，$d$ 是到机器人的距离），模型边界连续、有界、覆盖面积最小 |
| 动态个体 | $h_v > 0$ | 沿运动方向的 $\sigma$ 随速度增大，垂直方向保持不变，边界仍连续 |
| 静态群体 | 群内 $h_v = 0$ | 群组内所有个体位置求和求均值，合成一个覆盖群组的高斯，模型覆盖区域最小 |
| 动态群体 | 群内 $h_v > 0$ | 动态个体加静态群体两部分的合成 |

局部轨迹表示分两步。航路点从上层模块的全局路径中按累计距离条件选出，条件是到上一点的累计距离不小于给定阈值 [第五章 无人驾驶预测系统实例.pdf · p.32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=32)。每段轨迹用分段高阶多项式表示，$\beta$ 阶多项式系数由边界条件、航路点约束、连续性约束解出：

$$
p_i(t) = \sum_{k=1}^{\beta} c_k^{\,i} t^{k}, \qquad p(t) = p^{M}(t) + \sum_{i=1}^{M-1} s_i(t)\, p^{i}(t)
$$

其中 $s_i(t)$ 是第 $i$ 段的多项式权重，各段权重交替取 $s_0 - s_1 + s_2 - \cdots$，让整条轨迹在航路点处连续 [第五章 无人驾驶预测系统实例.pdf · p.32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=32)。

轨迹优化问题把要求写成目标函数加约束：机器人在行进途中不与障碍物碰撞、不与行人群组碰撞、行驶轨迹平滑、耗时尽可能短、满足机器人运动学约束 [第五章 无人驾驶预测系统实例.pdf · p.33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=33)。目标函数把各条要求加权求和：

$$
J = J_{ct} + J_{co} + J_{ch} + J_{v} + J_{a} + \lambda_{\omega} J_{\omega}
$$

其中 $J_{ct}$ 是轨迹跟踪与执行时间项，$J_{co}$ 是避碰项，$J_{ch}$ 是避撞项，$J_{v}$ 和 $J_{a}$ 是速度与加速度项，$J_{\omega}$ 是角速度项，$\lambda_{\omega}$ 是它的权重。硬约束（边界条件、航路点、连续性、运动学）被转成带惩罚的软约束项并入同一个最小化问题 [第五章 无人驾驶预测系统实例.pdf · p.33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=33)。

评价用两个因子，物理安全性和心理舒适性各管一头 [第五章 无人驾驶预测系统实例.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=35)。社交群组因子 SGI 衡量物理安全性，取所有社交空间里最危险的那一个，密度项乘上两个朝向项：

$$
\mathrm{SGI} = \max_{i=1:N_{sp}} \frac{\mathcal{N}_{\mu,\Sigma}(p_{bot})}{A} \cdot \left(-0.9 \times \frac{|\theta_{hr,i}|}{\pi} + 1\right) \cdot \left(-\frac{|\theta_{rh,i}|}{\pi} + 1\right)
$$

相对运动因子 RMI 衡量心理舒适性，看机器人与行人的相对运动和间距：

$$
\mathrm{RMI} = \max_{i=1:N_{g}} \frac{2 + v_{bot}\cos(\theta_{rh}) + v_{h,i}\cos(\theta_{hr})}{\left\| p_{O^{i}_{s,i}} - p_{bot} \right\|_2}
$$

](images/fig-05-p35.png)

仿真设置有走廊、商场、展馆三类环境，环境名后的 (a)(b)(c) 表示同一环境下不同配置（不同行人数、不同起终点）。走廊每个配置做十次仿真，商场和展馆分别做了十次行人随机速度行驶和五次行人匀速行驶的仿真 [第五章 无人驾驶预测系统实例.pdf · p.37](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=37)。场景里紫色是优化后的机器人局部轨迹，蓝色圆周是构建的行人社交空间，同一圆周内的行人属于同一群组 [第五章 无人驾驶预测系统实例.pdf · p.34](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=34)。

报告最后给的研究展望是三条：研究动态场景自主探索（动态障碍物识别与跟踪、动态障碍物轨迹预测、动态 SLAM、端到端运动控制器）；深入研究行人轨迹预测（把场景信息纳入图建模，构建行人与场景的交互关系；设计可处理任意历史轨迹时长的模型）；深入社会规范化路径规划（把人群密度纳入社交空间构建，以机器人第一视角避开人群稠密区域）[第五章 无人驾驶预测系统实例.pdf · p.42](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=42)。三条展望的落点很清楚：预测精度、预测鲁棒性、预测在规划里的落地方式。
