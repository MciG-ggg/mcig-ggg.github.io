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

| 卷 | 章节覆盖 | 主要程序题 |
|---|---|---|
| A 卷 | 第 1 至第 8 章全覆盖，各章考点与 B、C 卷不同 | 三维点云最近邻距离、社会空间轨迹平滑与时间联合代价 |
| B 卷 | 第 1 至第 8 章全覆盖，各章考点与 A、C 卷不同 | 点云半径离群点过滤、ETPP 道路图扩展代价选择 |
| C 卷 | 第 1 至第 8 章全覆盖，各章考点与 A、B 卷不同 | 二维覆盖路径长度、MPC 预测时域状态跟踪代价 |

配套复习资料在[《Auto-Drive-Intro 代码题复习》](/lecture/Auto-Drive-Intro_代码题复习)。讲义在[《Auto-Drive-Intro》](/lecture/Auto-Drive-Intro)。

## A 卷

**侧重：** 第 1 至第 8 章全覆盖，各章考点与 B、C 卷不同

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
| 1 | 驾驶自动化等级与后援责任〔ch1P1〕 | 单选 | 2 | 基础 | [第一章 无人驾驶概述_2026.pdf · p.20-21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=20) |
| 2 | 静态物体分割的车道线检测〔ch2P1〕 | 单选 | 2 | 基础 | [第二章 无人驾驶感知基础.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=7) |
| 3 | OpenDRIVE 的 s、t 坐标〔ch3P2〕 | 单选 | 2 | 基础 | [第三章 高精度地图.pdf · p.17-19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=17) |
| 4 | RTK 已知位置基站〔ch4P2〕 | 单选 | 2 | 基础 | [第四章 无人驾驶定位系统2.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=9) |
| 5 | ETPP 扩展代价计算〔ch5P2〕 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |
| 6 | 路径规划输入信息〔ch6P1〕 | 单选 | 2 | 基础 | [作业二：基于百度apollo平台的多场景下路径规划.docx · p.7](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx#page=7) |
| 7 | LQR 状态方程〔ch7P1〕 | 单选 | 2 | 基础 | [第七章 无人驾驶控制理论.pdf · p.27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=27) |
| 8 | Q-learning 最大动作价值〔ch8P1〕 | 单选 | 2 | 基础 | [第八章 无人驾驶强化学习.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=14) |
| 9 | 图像和点云的外部算法处理〔ch2P2〕 | 单选 | 2 | 基础 | [第二章 无人驾驶感知基础.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=5) |
| 10 | 高精度地图的深度学习元素识别〔ch3P4〕 | 单选 | 2 | 基础 | [第三章 高精度地图.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=10) |
| 11 | IMU 与 GNSS 更新频率〔ch1P2〕 | 填空 | 2 | 中等 | [第一章 无人驾驶概述_2026.pdf · p.34](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=34) |
| 12 | 感知基础任务与分割〔ch2P3〕 | 填空 | 2 | 中等 | [第二章 无人驾驶感知基础.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=5) |
| 13 | 高精地图道路连接关系〔ch3P5〕 | 填空 | 2 | 中等 | [第三章 高精度地图.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=9) |
| 14 | RTK 基站位置条件〔ch4P13〕 | 填空 | 2 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=9) |
| 15 | 预测系统的实时与准确要求〔ch5P1〕 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统.pdf · p.8](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=8) |
| 16 | 道路限速对规划速度的约束计算〔ch6P9〕 | 填空 | 2 | 中等 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.3](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=3) |
| 17 | 增量式 PID 计算〔ch7P2〕 | 填空 | 2 | 中等 | [第七章 无人驾驶控制理论.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=19) |
| 18 | Q-learning 更新值〔ch8P2〕 | 填空 | 2 | 中等 | [第八章 无人驾驶强化学习.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=14) |
| 19 | AI 安全与验证支柱〔ch1P12〕 | 填空 | 2 | 中等 | [第一章 无人驾驶概述_2026.pdf · p.55](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=55) |
| 20 | IMU 与磁力计融合校正角度〔ch4P14〕 | 填空 | 2 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=22) |
| 21 | ETPP 代价、熵与迭代次数〔ch5P4〕 | 问答 | 10 | 中等 | [第五章 无人驾驶预测系统实例.pdf · p.10-11,15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |
| 22 | Apollo 多场景路径规划〔ch6P3〕 | 问答 | 10 | 综合 | [作业二：基于百度apollo平台的多场景下路径规划.docx · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx#page=17) |
| 23 | PID 环节作用与噪声〔ch7P3〕 | 问答 | 10 | 综合 | [第七章 无人驾驶控制理论.pdf · p.16-18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=16) |
| 24 | 三维点云最近邻距离程序〔ch2P6〕 | 程序 | 15 | 中等 | [第二章-行车环境感知-激光雷达和点云.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=35) |
| 25 | 社会空间轨迹平滑与时间联合代价程序〔ch5P5〕 | 程序 | 15 | 综合 | [第五章 无人驾驶预测系统实例.pdf · p.30-33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=30) |

</details>

## B 卷

**侧重：** 第 1 至第 8 章全覆盖，各章考点与 A、C 卷不同

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
| 1 | 自动驾驶软件架构演进〔ch1P4〕 | 单选 | 2 | 基础 | [第一章 无人驾驶概述_2026.pdf · p.28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=28) |
| 2 | 目标检测的 two-step 与 one-step 架构〔ch2P4〕 | 单选 | 2 | 基础 | [第二章 无人驾驶感知基础.pdf · p.28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=28) |
| 3 | 高精度地图交通信息〔ch3P3〕 | 单选 | 2 | 基础 | [第三章 高精度地图.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=9) |
| 4 | SLAM 回环检测〔ch4P12〕 | 单选 | 2 | 基础 | [第四章 无人驾驶定位系统2.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=20) |
| 5 | 预测方法中人类知识与数据学习的差异〔ch5P6〕 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=9) |
| 6 | Apollo 泊车场景状态〔ch6P4〕 | 单选 | 2 | 基础 | [作业二：基于百度apollo平台的多场景下路径规划.docx · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx#page=22) |
| 7 | LQR 状态误差和控制量代价组成〔ch7P4〕 | 单选 | 2 | 基础 | [第七章 无人驾驶控制理论.pdf · p.27-28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=27) |
| 8 | 集中训练与分布执行〔ch8P4〕 | 单选 | 2 | 基础 | [第八章 强化深度学习案例.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=20) |
| 9 | ACXCOOT 普通个体的边界点顺序更新〔ch5P7〕 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统实例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |
| 10 | 自动驾驶软件栈层次〔ch1P5〕 | 单选 | 2 | 基础 | [第一章 无人驾驶概述_2026.pdf · p.38](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=38) |
| 11 | 现代预测技术方向〔ch1P6〕 | 填空 | 2 | 中等 | [第一章 无人驾驶概述_2026.pdf · p.49](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=49) |
| 12 | Mask R-CNN 的 mask 输出分支〔ch2P5〕 | 填空 | 2 | 中等 | [第二章 无人驾驶感知基础.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=24) |
| 13 | 高精地图与导航地图精度〔ch3P10〕 | 填空 | 2 | 中等 | [第三章 高精度地图.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=6) |
| 14 | GNSS 更新间隔位移〔ch4P15〕 | 填空 | 2 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=10) |
| 15 | 预测数据标签平衡〔ch5P15〕 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=16) |
| 16 | 安全距离对车辆接近行为的影响〔ch6P28〕 | 填空 | 2 | 中等 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=12) |
| 17 | PID 积分时间常数〔ch7P5〕 | 填空 | 2 | 中等 | [第七章 无人驾驶控制理论.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=17) |
| 18 | 经验回放随机采样〔ch8P5〕 | 填空 | 2 | 中等 | [第八章 无人驾驶强化学习.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=16) |
| 19 | 机器人数量与覆盖分段〔ch8P6〕 | 填空 | 2 | 中等 | [第八章 无人驾驶强化学习.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=10) |
| 20 | 功能安全标准〔ch1P7〕 | 填空 | 2 | 中等 | [第一章 无人驾驶概述_2026.pdf · p.55](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=55) |
| 21 | LaneNet 车道实例分割〔ch2P7〕 | 问答 | 10 | 中等 | [第二章 无人驾驶感知基础.pdf · p.25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=25) |
| 22 | HSGNN 群组划分、消融模块与案例分析〔ch5P8〕 | 问答 | 10 | 综合 | [第五章 无人驾驶预测系统实例.pdf · p.20-24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=20) |
| 23 | Apollo S-T 曲线与速度决策〔ch6P6〕 | 问答 | 10 | 综合 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.25-27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=25) |
| 24 | 点云半径离群点过滤程序〔ch2P12〕 | 程序 | 15 | 中等 | [第二章-行车环境感知-激光雷达和点云.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=29) |
| 25 | ETPP 道路图扩展代价选择程序〔ch5P12〕 | 程序 | 15 | 综合 | [第五章 无人驾驶预测系统实例.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=10) |

</details>

## C 卷

**侧重：** 第 1 至第 8 章全覆盖，各章考点与 A、B 卷不同

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
| 1 | 车路云协同的 RSU〔ch1P8〕 | 单选 | 2 | 基础 | [第一章 无人驾驶概述_2026.pdf · p.26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=26) |
| 2 | 语义分割的像素级类别〔ch2P8〕 | 单选 | 2 | 基础 | [第二章 无人驾驶感知基础.pdf · p.32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=32) |
| 3 | 高精度地图与导航地图的精度差异〔ch3P9〕 | 单选 | 2 | 基础 | [第三章 高精度地图.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=9) |
| 4 | 直方图滤波信息更新〔ch4P4〕 | 单选 | 2 | 基础 | [第四章 无人驾驶定位系统2.pdf · p.25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=25) |
| 5 | 基于地图的车道序列预测〔ch5P9〕 | 单选 | 2 | 基础 | [第五章 无人驾驶预测系统.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=11) |
| 6 | PNC Monitor 车辆参数〔ch6P7〕 | 单选 | 2 | 基础 | [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx#page=13) |
| 7 | MPC 反馈与滚动重优化〔ch7P6〕 | 单选 | 2 | 基础 | [第七章 无人驾驶控制理论.pdf · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=31) |
| 8 | 折扣回报〔ch8P7〕 | 单选 | 2 | 基础 | [第八章 无人驾驶强化学习.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=12) |
| 9 | 感知中的目标跟踪〔ch1P9〕 | 单选 | 2 | 基础 | [第一章 无人驾驶概述_2026.pdf · p.47](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=47) |
| 10 | FCN 全卷积结构〔ch2P9〕 | 单选 | 2 | 基础 | [第二章 无人驾驶感知基础.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=23) |
| 11 | ISO 21448 与能力不足风险〔ch1P10〕 | 填空 | 2 | 中等 | [第一章 无人驾驶概述_2026.pdf · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=35) |
| 12 | KD-Tree 近邻搜索〔ch2P10〕 | 填空 | 2 | 中等 | [第二章-行车环境感知-激光雷达和点云.pdf · p.43](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=43) |
| 13 | OpenDRIVE 高度坐标〔ch3P8〕 | 填空 | 2 | 中等 | [第三章 高精度地图.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=19) |
| 14 | 直方图滤波不确定移动〔ch4P11〕 | 填空 | 2 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=26) |
| 15 | 轨迹生成器方法〔ch5P13〕 | 填空 | 2 | 中等 | [第五章 无人驾驶预测系统.pdf · p.13-14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=13) |
| 16 | 减速带场景下的悬挂系统〔ch6P27〕 | 填空 | 2 | 中等 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.3](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=3) |
| 17 | 位置式与增量式 PID〔ch7P7〕 | 填空 | 2 | 中等 | [第七章 无人驾驶控制理论.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=19) |
| 18 | DQN Epsilon 探索概率〔ch8P10〕 | 填空 | 2 | 中等 | [第八章 无人驾驶强化学习.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=17) |
| 19 | 经典底层控制算法〔ch1P11〕 | 填空 | 2 | 中等 | [第一章 无人驾驶概述_2026.pdf · p.51](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=51) |
| 20 | DeepLab 空洞卷积感受野〔ch2P11〕 | 填空 | 2 | 中等 | [第二章 无人驾驶感知基础.pdf · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=22) |
| 21 | IMU 与 GNSS 融合定位〔ch4P16〕 | 问答 | 10 | 中等 | [第四章 无人驾驶定位系统2.pdf · p.6,22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=6) |
| 22 | HSGNN 时空注意力、TCN 与多模态评估〔ch5P10〕 | 问答 | 10 | 综合 | [第五章 无人驾驶预测系统实例.pdf · p.20-22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=20) |
| 23 | 串联 PID 外环与内环的级联关系〔ch7P9〕 | 问答 | 10 | 综合 | [第七章 无人驾驶控制理论.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=23) |
| 24 | 二维覆盖路径长度程序〔ch8P9〕 | 程序 | 15 | 中等 | [第八章 无人驾驶强化学习.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=9) |
| 25 | MPC 预测时域状态跟踪代价程序〔ch7P10〕 | 程序 | 15 | 综合 | [第七章 无人驾驶控制理论.pdf · p.31-33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=31) |

</details>

## 命题边界

- 题目依据十一份课件和四份作业。试卷采用原创样例，不是教师发布的正式考卷。
- 程序题给出必要公式、输入输出和边界条件。程序题不要求安装或启动 Apollo。
- 数值样例、数组、简化代价和验收定义都是原创命题数据。源材料只提供概念和实验背景。
- 题目难度未经真实考生校准。参考程序用合成图像和人工数组自检，不能替代 Apollo 回放、实车实验或算法效果验证。

试卷与答案的 LaTeX 源文件在仓库的 `docs/course-drafts/exam-papers-20261027/`，命题蓝图和核验记录在同目录的 `build/` 下。
