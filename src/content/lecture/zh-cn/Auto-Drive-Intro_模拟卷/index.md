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
description: "《智能驾驶导论》的十一份课件与四次 APOLLO 平台作业，模拟卷的取材来源"
host: 10.188.31.55:5244
```

三套卷，每套 100 分 25 题：单选 10 题 20 分，填空 10 题 20 分，问答 3 题 30 分，程序 2 题 30 分。建议 120 分钟。

**答案在每卷末尾的折叠块里。** 闭卷做完再展开对答案。程序题用 Python 3 写，题面给了要用的 NumPy 与 OpenCV 接口，不用启动 Apollo。

| 卷 | 侧重 | 第 24 题 | 第 25 题 |
|---|---|---|---|
| A 卷 | 控制、车道线检测、高精地图、定位、预测、强化学习、Cyber 通道 | 灰度、高斯平滑、Canny、ROI 与 Hough | 路径采样边界与候选代价 |
| B 卷 | 点云、Frenet、多源定位、预测、速度规划、LQR 与 PID、Cyber 回放 | Hough 线段筛选与左右线合并 | 速度差分、阈值与距离一致性 |
| C 卷 | 语义分割、地图验证、SLAM、预测、泊车、MPC、强化学习、控制参数 | 分割掩码拟合与车道中心偏移 | 控制误差、超调与窗口内稳定时刻 |

配套复习资料在[《Auto-Drive-Intro 代码题复习》](/lecture/Auto-Drive-Intro_代码题复习)。讲义在[《Auto-Drive-Intro》](/lecture/Auto-Drive-Intro)。

## A 卷

:::pdf{src="/pdf/auto-drive-intro/a-paper.pdf" title="A 卷试卷" download="智能驾驶导论_A卷_试卷.pdf" height="760"}
:::

<details>
<summary>A 卷答案与评分细则</summary>

:::pdf{src="/pdf/auto-drive-intro/a-answer.pdf" title="A 卷答案" download="智能驾驶导论_A卷_答案.pdf" height="760"}
:::

</details>

<details>
<summary>A 卷题源对照</summary>

| 题号 | 主题 | 题型 | 分值 | 来源 |
|---:|---|---|---:|---|
| 1 | 控制输出 | 单选 | 2 | [第七章 无人驾驶控制理论.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=5) |
| 2 | Canny降噪 | 单选 | 2 | [第二章 无人驾驶感知基础.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=10) |
| 3 | Hough极坐标 | 单选 | 2 | [第二章 无人驾驶感知基础.pdf · p.12-13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=12) |
| 4 | LaneNet实例 | 单选 | 2 | [第二章 无人驾驶感知基础.pdf · p.25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=25) |
| 5 | 地图拓扑 | 单选 | 2 | [第三章 高精度地图.pdf · p.6,9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=6) |
| 6 | IMU误差 | 单选 | 2 | [第四章 无人驾驶定位系统2.pdf · p.12-14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=12) |
| 7 | 车辆预测 | 单选 | 2 | [第五章 无人驾驶预测系统.pdf · p.11-14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=11) |
| 8 | MPC执行 | 单选 | 2 | [第七章 无人驾驶控制理论.pdf · p.29-33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=29) |
| 9 | Q值更新目标 | 单选 | 2 | [第八章 无人驾驶强化学习.pdf · p.13-16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=13) |
| 10 | Cyber通道监视 | 单选 | 2 | [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.20-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx#page=20) |
| 11 | 灰度转换 | 填空 | 2 | [第二章 无人驾驶感知基础.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=10) |
| 12 | Hough方程 | 填空 | 2 | [第二章 无人驾驶感知基础.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=13) |
| 13 | 道路纵向坐标 | 填空 | 2 | [第三章 高精度地图.pdf · p.17-19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=17) |
| 14 | PID积分 | 填空 | 2 | [第七章 无人驾驶控制理论.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=17) |
| 15 | 陀螺仪测量 | 填空 | 2 | [第四章 无人驾驶定位系统2.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=13) |
| 16 | 速度差分 | 填空 | 2 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.10-12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=10) |
| 17 | Q值计算 | 填空 | 2 | [第八章 无人驾驶强化学习.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=14) |
| 18 | 点云欧氏距离 | 填空 | 2 | [第二章-行车环境感知-激光雷达和点云.pdf · p.22,35-43](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=22) |
| 19 | 速度误差符号 | 填空 | 2 | [第七章 无人驾驶控制理论.pdf · p.15-20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=15)；[作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.4-5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx#page=4) |
| 20 | 匀速预测 | 填空 | 2 | [第五章 无人驾驶预测系统.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=17) |
| 21 | 传统与学习车道线检测 | 问答 | 10 | [第二章 无人驾驶感知基础.pdf · p.10-15,25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=10) |
| 22 | GNSS与IMU融合 | 问答 | 10 | [第四章 无人驾驶定位系统2.pdf · p.6,9-16,22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=6) |
| 23 | Apollo路径规划实验 | 问答 | 10 | [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.20-30](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx#page=20)；[作业二：基于百度apollo平台的多场景下路径规划.docx · p.15-24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx#page=15) |
| 24 | 图像到ROI与Hough程序 | 程序 | 15 | [第二章 无人驾驶感知基础.pdf · p.10-15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=10) |
| 25 | 路径候选约束与代价程序 | 程序 | 15 | [作业二：基于百度apollo平台的多场景下路径规划.docx · p.7,17,24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx#page=7) |

</details>

## B 卷

:::pdf{src="/pdf/auto-drive-intro/b-paper.pdf" title="B 卷试卷" download="智能驾驶导论_B卷_试卷.pdf" height="760"}
:::

<details>
<summary>B 卷答案与评分细则</summary>

:::pdf{src="/pdf/auto-drive-intro/b-answer.pdf" title="B 卷答案" download="智能驾驶导论_B卷_答案.pdf" height="760"}
:::

</details>

<details>
<summary>B 卷题源对照</summary>

| 题号 | 主题 | 题型 | 分值 | 来源 |
|---:|---|---|---:|---|
| 1 | 感知任务区别 | 单选 | 2 | [第一章 无人驾驶概述_2026.pdf · p.47](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=47) |
| 2 | 点云离群点 | 单选 | 2 | [第二章-行车环境感知-激光雷达和点云.pdf · p.28-31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=28) |
| 3 | Frenet横向坐标 | 单选 | 2 | [第三章 高精度地图.pdf · p.17-19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=17) |
| 4 | 多路径定位 | 单选 | 2 | [第四章 无人驾驶定位系统2.pdf · p.9-10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=9) |
| 5 | 行人交互 | 单选 | 2 | [第五章 无人驾驶预测系统.pdf · p.22-24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=22)；[第五章 无人驾驶预测系统实例.pdf · p.19-22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=19) |
| 6 | S-T含义 | 单选 | 2 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.25-27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=25) |
| 7 | LQR状态权重 | 单选 | 2 | [第七章 无人驾驶控制理论.pdf · p.26-28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=26)；[作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.27-29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx#page=27) |
| 8 | 经验回放 | 单选 | 2 | [第八章 无人驾驶强化学习.pdf · p.16-17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=16) |
| 9 | cyber数据回放 | 单选 | 2 | [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.20-23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx#page=20) |
| 10 | PID微分噪声 | 单选 | 2 | [第七章 无人驾驶控制理论.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=18) |
| 11 | 点云配准 | 填空 | 2 | [第四章 无人驾驶定位系统2.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=15) |
| 12 | KD-tree空间划分 | 填空 | 2 | [第二章-行车环境感知-激光雷达和点云.pdf · p.38-44](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=38) |
| 13 | S-T斜率 | 填空 | 2 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.25-27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=25) |
| 14 | 定位传感器融合 | 填空 | 2 | [第四章 无人驾驶定位系统2.pdf · p.6,22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=6) |
| 15 | LQR控制代价 | 填空 | 2 | [第七章 无人驾驶控制理论.pdf · p.27-28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=27) |
| 16 | 恒加速度预测 | 填空 | 2 | [第五章 无人驾驶预测系统.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=17) |
| 17 | PID计算 | 填空 | 2 | [第七章 无人驾驶控制理论.pdf · p.15-19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=15) |
| 18 | Q值终止目标 | 填空 | 2 | [第八章 无人驾驶强化学习.pdf · p.14-16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=14) |
| 19 | 预测ADE | 填空 | 2 | [第五章 无人驾驶预测系统实例.pdf · p.23-24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=23) |
| 20 | 车道线交点计算 | 填空 | 2 | [第二章 无人驾驶感知基础.pdf · p.14-15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=14) |
| 21 | 点云地图与轨迹预测 | 问答 | 10 | [第二章-行车环境感知-激光雷达和点云.pdf · p.28-31,38-44](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0-%E8%A1%8C%E8%BD%A6%E7%8E%AF%E5%A2%83%E6%84%9F%E7%9F%A5-%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E5%92%8C%E7%82%B9%E4%BA%91.pdf#page=28)；[第三章 高精度地图.pdf · p.7-9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=7)；[第五章 无人驾驶预测系统.pdf · p.11-17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=11) |
| 22 | 速度规划与数据证据 | 问答 | 10 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.2-12,19-28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=2) |
| 23 | PID LQR MPC选择 | 问答 | 10 | [第七章 无人驾驶控制理论.pdf · p.16-18,26-33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=16)；[作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.5,27-29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx#page=5) |
| 24 | Hough左右车道线聚合程序 | 程序 | 15 | [第二章 无人驾驶感知基础.pdf · p.13-15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=13) |
| 25 | 速度差分及数据一致性程序 | 程序 | 15 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.7-12,22-28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=7) |

</details>

## C 卷

:::pdf{src="/pdf/auto-drive-intro/c-paper.pdf" title="C 卷试卷" download="智能驾驶导论_C卷_试卷.pdf" height="760"}
:::

<details>
<summary>C 卷答案与评分细则</summary>

:::pdf{src="/pdf/auto-drive-intro/c-answer.pdf" title="C 卷答案" download="智能驾驶导论_C卷_答案.pdf" height="760"}
:::

</details>

<details>
<summary>C 卷题源对照</summary>

| 题号 | 主题 | 题型 | 分值 | 来源 |
|---:|---|---|---:|---|
| 1 | 模块因果链 | 单选 | 2 | [第一章 无人驾驶概述_2026.pdf · p.40,44-52](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%80%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%A6%82%E8%BF%B0_2026.pdf#page=40) |
| 2 | 语义实例分割 | 单选 | 2 | [第二章 无人驾驶感知基础.pdf · p.16,25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=16) |
| 3 | 地图人工验证 | 单选 | 2 | [第三章 高精度地图.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=10) |
| 4 | SLAM闭环 | 单选 | 2 | [第四章 无人驾驶定位系统2.pdf · p.19-20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=19) |
| 5 | 预测不确定性 | 单选 | 2 | [第五章 无人驾驶预测系统.pdf · p.8-14,22-24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=8) |
| 6 | 开放空间泊车 | 单选 | 2 | [作业二：基于百度apollo平台的多场景下路径规划.docx · p.19-22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx#page=19) |
| 7 | PID积分累积 | 单选 | 2 | [第七章 无人驾驶控制理论.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=17) |
| 8 | MPC约束 | 单选 | 2 | [第七章 无人驾驶控制理论.pdf · p.29-33](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=29) |
| 9 | 模仿学习 | 单选 | 2 | [第八章 无人驾驶强化学习.pdf · p.4-6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=4) |
| 10 | 控制参数公平比较 | 单选 | 2 | [作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.2-7,24-32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx#page=2) |
| 11 | IMU频率 | 填空 | 2 | [第四章 无人驾驶定位系统2.pdf · p.14](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=14) |
| 12 | 车道ROI | 填空 | 2 | [第二章 无人驾驶感知基础.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=11) |
| 13 | 位置速度积分 | 填空 | 2 | [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.11,25-27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx#page=11) |
| 14 | LQR航向误差权重 | 填空 | 2 | [作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.27-29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx#page=27) |
| 15 | 规划运动参数 | 填空 | 2 | [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.25-34](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx#page=25) |
| 16 | Q-learning探索 | 填空 | 2 | [第八章 无人驾驶强化学习.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=17) |
| 17 | 多机器人奖励与观测 | 填空 | 2 | [第八章 强化深度学习案例.pdf · p.17-18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=17) |
| 18 | 预测FDE | 填空 | 2 | [第五章 无人驾驶预测系统实例.pdf · p.23-24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=23) |
| 19 | 像素偏移符号 | 填空 | 2 | [第二章 无人驾驶感知基础.pdf · p.25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=25) |
| 20 | 比例控制计算 | 填空 | 2 | [第七章 无人驾驶控制理论.pdf · p.15-16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=15) |
| 21 | SLAM与多源定位部署 | 问答 | 10 | [第四章 无人驾驶定位系统2.pdf · p.9-22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E7%B3%BB%E7%BB%9F2.pdf#page=9)；[第四章-汽车智能驾驶定位解决方案.pdf · p.13-15,25](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%9B%9B%E7%AB%A0-%E6%B1%BD%E8%BD%A6%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AE%9A%E4%BD%8D%E8%A7%A3%E5%86%B3%E6%96%B9%E6%A1%88.pdf#page=13) |
| 22 | 预测群组与强化学习边界 | 问答 | 10 | [第五章 无人驾驶预测系统.pdf · p.22-24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F.pdf#page=22)；[第五章 无人驾驶预测系统实例.pdf · p.19-29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%94%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E9%A2%84%E6%B5%8B%E7%B3%BB%E7%BB%9F%E5%AE%9E%E4%BE%8B.pdf#page=19)；[第八章 无人驾驶强化学习.pdf · p.4-6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E5%BC%BA%E5%8C%96%E5%AD%A6%E4%B9%A0.pdf#page=4)；[第八章 强化深度学习案例.pdf · p.17-18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E5%85%AB%E7%AB%A0%20%E5%BC%BA%E5%8C%96%E6%B7%B1%E5%BA%A6%E5%AD%A6%E4%B9%A0%E6%A1%88%E4%BE%8B.pdf#page=17) |
| 23 | Apollo控制结果证据 | 问答 | 10 | [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.20-30](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx#page=20)；[作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.4-7,24-32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx#page=4) |
| 24 | 分割掩码车道拟合程序 | 程序 | 15 | [第二章 无人驾驶感知基础.pdf · p.25-26](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%BA%8C%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%84%9F%E7%9F%A5%E5%9F%BA%E7%A1%80.pdf#page=25) |
| 25 | 控制误差与窗口稳定时刻程序 | 程序 | 15 | [第七章 无人驾驶控制理论.pdf · p.10-11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%83%E7%AB%A0%20%E6%97%A0%E4%BA%BA%E9%A9%BE%E9%A9%B6%E6%8E%A7%E5%88%B6%E7%90%86%E8%AE%BA.pdf#page=10)；[作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.4-7,24-32](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx#page=4) |

</details>

## 命题边界

- 题目依据十一份课件和四份作业。试卷采用原创样例，不是教师发布的正式考卷。
- 程序题给出必要公式、输入输出和边界条件。程序题不要求安装或启动 Apollo。
- 数值样例、数组、简化代价和验收定义都是原创命题数据。源材料只提供概念和实验背景。
- 题目难度未经真实考生校准。参考程序用合成图像和人工数组自检，不能替代 Apollo 回放、实车实验或算法效果验证。

试卷与答案的 LaTeX 源文件在仓库的 `docs/course-drafts/exam-papers-20261027/`，命题蓝图和核验记录在同目录的 `build/` 下。
