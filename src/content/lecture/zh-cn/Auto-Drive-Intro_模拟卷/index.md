---
title: Auto-Drive-Intro 模拟卷
timestamp: 2026-10-27T15:30:00+08:00
series: Auto-Drive-Intro
tags: [课内, Auto-Drive-Intro]
description: 《智能驾驶导论》期末模拟卷三套，每套 100 分 25 题。试卷与答案各一份 PDF，页面里直接读，也能下载打印。每道题的出处链回课件和作业的对应页。
source_url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/课件/
toc: true
draft: false
---

```cardlink
url: http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/课件/
title: "Auto-Drive-Intro · 课程课件与作业"
description: "《智能驾驶导论》的十一份课件与四份 APOLLO 平台作业，模拟卷的取材来源"
host: 10.188.31.55:5244
```

> [!IMPORTANT] 三套卷，每套 100 分 25 题
> 单选 10 题 20 分，填空 10 题 20 分，问答 3 题 30 分，程序 2 题 30 分。建议 120 分钟。

> [!TIP] 先闭卷做，再展开答案
> 答案在每卷末尾的折叠块里。程序题用 Python 3 写，题面给了要用的 NumPy 接口，不用启动 Apollo。

## 三套卷的分工

| 卷 | 取材侧重 | 单选与填空 | 问答 | 程序题 |
|---|---|---|---|---|
| A 卷 | 专题一：ETPP 未知环境遍历、HSGNN 行人轨迹预测、社交空间约束路径规划 | ETPP公式、HSGNN注意力与群组参数计算 | 遍历代价、群组划分、社交空间路径评价 | 稀疏注意力softmax、社交空间路径选择 |
| B 卷 | 专题二：多机器人协同覆盖、HAER-MADDPG 围捕决策、足球机器人协同进攻 | 覆盖路径数据、HAER-MADDPG与BI-MAPDDPG计算 | 遗传算法负载均衡、经验优先级、足球策略实验 | 覆盖路径分段评价、双通道策略分析 |
| C 卷 | 课程核心：控制、强化学习、定位、地图、感知与预测 | PID、Q-learning、定位与感知计算 | 直方图定位、多模态预测决策、控制策略比较 | 车道掩码拟合、纵向控制误差分析 |

配套复习资料在[《Auto-Drive-Intro 代码题复习》](/lecture/Auto-Drive-Intro_代码题复习)。讲义在[《Auto-Drive-Intro》](/lecture/Auto-Drive-Intro)。

## A 卷

**侧重：** 专题一：ETPP 未知环境遍历、HSGNN 行人轨迹预测、社交空间约束路径规划

:::pdf{src="/pdf/auto-drive-intro/a-paper.pdf" title="A 卷试卷" download="智能驾驶导论_A卷_试卷.pdf" height="760"}
:::

<details>
<summary>A 卷答案与评分细则</summary>

:::pdf{src="/pdf/auto-drive-intro/a-answer.pdf" title="A 卷答案" download="智能驾驶导论_A卷_答案.pdf" height="760"}
:::

</details>

<details>
<summary>A 卷题源对照</summary>

| 题号 | 主题 | 题型 | 分值 | 难度 | 来源 |
|---:|---|---|---:|---|---|
| 1 | ETPP扩展代价计算 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |
| 2 | 归一化信息熵停止条件 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=11) |
| 3 | ACXCOOT迭代次数限幅 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |
| 4 | 稀疏注意力softmax | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=20) |
| 5 | HSGNN时空注意力分工 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=20) |
| 6 | 群组特征相似权重 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=22) |
| 7 | 静态群组尺度参数 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=31) |
| 8 | 静态与动态社交空间 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.30-31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=30) |
| 9 | ACXCOOT耗时对比 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=15) |
| 10 | HSGNN群组层消融结果 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.24,27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=24) |
| 11 | ETPP射束长度计算 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=11) |
| 12 | ACXCOOT迭代次数限幅 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |
| 13 | 群组特征相似权重计算 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=22) |
| 14 | 稀疏注意力权重计算 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=20) |
| 15 | HSGNN多模态ADE计算 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.21,23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=21) |
| 16 | 静态群组局部原点 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=31) |
| 17 | 社交空间速度罚项 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=33) |
| 18 | 轨迹平滑与时间目标 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=33) |
| 19 | 社交空间SGI计算 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=35) |
| 20 | HSGNN消融影响排序 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=24) |
| 21 | ETPP邻居扩展代价与选择 | 问答 | 10 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.10-11,15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |
| 22 | HSGNN群组划分与注意力权重 | 问答 | 10 | 综合 | [第五章 无人驾驶预测系统实例.pdf · p.19-27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=19) |
| 23 | 静态社交空间约束下的候选路径评价 | 问答 | 10 | 综合 | [第五章 无人驾驶预测系统实例.pdf · p.29-33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=29) |
| 24 | 稀疏注意力行softmax程序 | 程序 | 15 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=20) |
| 25 | 社交空间约束下的路径选择程序 | 程序 | 15 | 综合 | [第五章 无人驾驶预测系统实例.pdf · p.30-33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=30) |

</details>

## B 卷

**侧重：** 专题二：多机器人协同覆盖、HAER-MADDPG 围捕决策、足球机器人协同进攻

:::pdf{src="/pdf/auto-drive-intro/b-paper.pdf" title="B 卷试卷" download="智能驾驶导论_B卷_试卷.pdf" height="760"}
:::

<details>
<summary>B 卷答案与评分细则</summary>

:::pdf{src="/pdf/auto-drive-intro/b-answer.pdf" title="B 卷答案" download="智能驾驶导论_B卷_答案.pdf" height="760"}
:::

</details>

<details>
<summary>B 卷题源对照</summary>

| 题号 | 主题 | 题型 | 分值 | 难度 | 来源 |
|---:|---|---|---:|---|---|
| 1 | 栅格尺寸与覆盖实验结果 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=13) |
| 2 | 按可工作机器人数量分配路径 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=9) |
| 3 | 覆盖任务死区恢复 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=8) |
| 4 | 状态与动作注意力贡献 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=19) |
| 5 | 优先经验回放均匀采样 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=21) |
| 6 | 集中训练与分布执行 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=20) |
| 7 | 多机器人样本重要性 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=21) |
| 8 | 离散动作权重退火 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=35) |
| 9 | 离散动作与连续参数依赖 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.34](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=34) |
| 10 | BI-MAPDDPG实验奖励比较 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.42](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=42) |
| 11 | 候选路径分段方差 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=9) |
| 12 | 不同栅格尺寸的路径长度差 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=13) |
| 13 | 多机器人任务量方差差值 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=14) |
| 14 | 状态与动作注意力融合权重 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=21) |
| 15 | 优先经验回放采样概率 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=21) |
| 16 | 多机器人样本重要性 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=21) |
| 17 | HAER-MADDPG收敛周期降幅 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=22) |
| 18 | 离散动作权重退火计算 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=35) |
| 19 | 足球机器人进球占比变化 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.37](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=37) |
| 20 | 双通道方法平均奖励差 | 填空 | 2 | 中等 | [第八章 强化深度学习案例.pdf · p.42](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=42) |
| 21 | 遗传算法覆盖路径分段与负载均衡 | 问答 | 10 | 中等 | [第八章 强化深度学习案例.pdf · p.9,14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=9) |
| 22 | HAER-MADDPG经验重要性与采样概率 | 问答 | 10 | 综合 | [第八章 强化深度学习案例.pdf · p.19,21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=19) |
| 23 | BI-MAPDDPG动作权重与足球实验指标 | 问答 | 10 | 综合 | [第八章 强化深度学习案例.pdf · p.34-37](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=34) |
| 24 | 覆盖路径分段方差评价程序 | 程序 | 15 | 中等 | [第八章 强化深度学习案例.pdf · p.9,14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=9) |
| 25 | 双通道策略权重与实验分析程序 | 程序 | 15 | 综合 | [第八章 强化深度学习案例.pdf · p.34-37](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=34) |

</details>

## C 卷

**侧重：** 课程核心：控制、强化学习、定位、地图、感知与预测

:::pdf{src="/pdf/auto-drive-intro/c-paper.pdf" title="C 卷试卷" download="智能驾驶导论_C卷_试卷.pdf" height="760"}
:::

<details>
<summary>C 卷答案与评分细则</summary>

:::pdf{src="/pdf/auto-drive-intro/c-answer.pdf" title="C 卷答案" download="智能驾驶导论_C卷_答案.pdf" height="760"}
:::

</details>

<details>
<summary>C 卷题源对照</summary>

| 题号 | 主题 | 题型 | 分值 | 难度 | 来源 |
|---:|---|---|---:|---|---|
| 1 | 比例增益与响应及振荡 | 单选 | 2 | 基础 | [第七章 无人驾驶控制理论.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=16) |
| 2 | Q表最大值动作选择 | 单选 | 2 | 基础 | [第八章 无人驾驶强化学习.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=14) |
| 3 | GNSS更新间隔位移 | 单选 | 2 | 基础 | [第四章 无人驾驶定位系统2.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=10) |
| 4 | 地图精度与定位误差要求 | 单选 | 2 | 基础 | [第三章 高精度地图.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=6) |
| 5 | 车道掩码实例区分 | 单选 | 2 | 基础 | [第二章 无人驾驶感知基础.pdf · p.16-17,25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=16) |
| 6 | 地图约束下的多模态预测 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统.pdf · p.9,11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=9) |
| 7 | 微分作用与测量噪声 | 单选 | 2 | 基础 | [第七章 无人驾驶控制理论.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=18) |
| 8 | 半径离群点滤波 | 单选 | 2 | 基础 | [第二章-行车环境感知-激光雷达和点云.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=29) |
| 9 | OpenDRIVE参考线纵向距离 | 单选 | 2 | 基础 | [第三章 高精度地图.pdf · p.17-19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=17) |
| 10 | 随机经验回放 | 单选 | 2 | 基础 | [第八章 无人驾驶强化学习.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=16) |
| 11 | 位置式PID计算 | 填空 | 2 | 中等 | [第七章 无人驾驶控制理论.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=19) |
| 12 | 增量式PID计算 | 填空 | 2 | 中等 | [第七章 无人驾驶控制理论.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=19) |
| 13 | Q-learning更新值计算 | 填空 | 2 | 中等 | [第八章 无人驾驶强化学习.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=14) |
| 14 | 直方图滤波观测更新 | 填空 | 2 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=23) |
| 15 | 环形位置概率运动更新 | 填空 | 2 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=24) |
| 16 | 折扣回报计算 | 填空 | 2 | 中等 | [第八章 无人驾驶强化学习.pdf · p.12,14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=12) |
| 17 | 导航地图精度尺度比较 | 填空 | 2 | 中等 | [第三章 高精度地图.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=6) |
| 18 | 预测平均位移误差 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=23) |
| 19 | 点云三维最近邻距离 | 填空 | 2 | 中等 | [第二章-行车环境感知-激光雷达和点云.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=35) |
| 20 | RTK基站距离条件 | 填空 | 2 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=9) |
| 21 | 直方图定位运动与观测更新 | 问答 | 10 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.23-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=23) |
| 22 | 多模态预测与规划风险决策 | 问答 | 10 | 综合 | [第三章 高精度地图.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=9)；[第五章 无人驾驶预测系统实例.pdf · p.19-23,27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=19) |
| 23 | PID控制与强化学习动作比较 | 问答 | 10 | 综合 | [第七章 无人驾驶控制理论.pdf · p.15-18,29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=15)；[第八章 无人驾驶强化学习.pdf · p.4,8,13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=4) |
| 24 | 车道掩码左右边界拟合程序 | 程序 | 15 | 中等 | [第二章 无人驾驶感知基础.pdf · p.25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=25) |
| 25 | 纵向控制记录误差指标程序 | 程序 | 15 | 综合 | [第七章 无人驾驶控制理论.pdf · p.15-19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=15) |

</details>

## 命题边界

- 题目依据十一份课件和四份作业。试卷采用原创样例，不是教师发布的正式考卷。
- 程序题给出必要公式、输入输出和边界条件。程序题不要求安装或启动 Apollo。
- 数值样例、数组、简化代价和验收定义都是原创命题数据。源材料只提供概念和实验背景。
- 题目难度未经真实考生校准。参考程序用合成图像和人工数组自检，不能替代 Apollo 回放、实车实验或算法效果验证。

试卷与答案的 LaTeX 源文件在仓库的 `docs/course-drafts/exam-papers-20261027/`，命题蓝图和核验记录在同目录的 `build/` 下。
