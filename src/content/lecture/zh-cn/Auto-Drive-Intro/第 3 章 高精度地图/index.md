---
title: 第 3 章 高精度地图
timestamp: 2026-10-24T11:00:00+08:00
series: Auto-Drive-Intro
tags: [课内, Auto-Drive-Intro]
description: 高清地图这一讲的知识主线是四段：HD Map 概览、OpenDRIVE 格式、点云建图、V2X 第三章 高精度地图.pdf · p.2。
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

## 第 3 章 高精度地图

### HD Map 在软件框架里的位置

高清地图这一讲的知识主线是四段：HD Map 概览、OpenDRIVE 格式、点云建图、V2X [第三章 高精度地图.pdf · p.2](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=2)。

HD Map 这一模块里装着五类东西：Visualization、Static Objects、HD Map info、Lidar Cloud Point Map、Merging [第三章 高精度地图.pdf · p.5](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=5)。HD Map 与导航地图的差别，课件用三行对比给出来 [第三章 高精度地图.pdf · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=6)：

| 维度 | High Dimension Map | Navigation Map |
| --- | --- | --- |
| 精度 | cm | m |
| 信息 | 3D lane info，More detail signals | Basic info |
| 用途 | Autonomous Vehicle 或 robotics | Human |

HD Map 在无人驾驶软件里同时挂到三个下游模块上 [第三章 高精度地图.pdf · p.7](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=7)：

- **与定位的关系**：Self-localization，Camera & Point cloud 的 Pre-processing、Coordinate transformation、Data fusion，结果是 High precision。
- **与感知的关系**：High range visualization、Shrink ROI、Saving Computation。地图的作用是把感知的搜索范围缩小下来。
- **与规划的关系**：Lane center 提供 Reference line 与 Boundary；Road network 提供 Merging zone 汇入口与 Junction 交叉口；Traffic information 提供 Stop sign 与 Speed limit [第三章 高精度地图.pdf · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=9)。

辅助课件也讲了高精地图在无人驾驶中的用法，以及它与传统的地图的差别：传统地图给司机看，高精地图包含大量驾驶辅助信息 [第2讲 无人车基本框架.pptx · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx)。

### 地图的表示方法：栅格、特征、拓扑

定位与建图这一讲把地图表示分成四类：Occupancy Grid map、Topological Map、Feature Map、Appearance Based map [第6讲 地图与定位.pptx · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。

尺度地图这一页把三种表示的取舍写清楚了 [第6讲 地图与定位.pptx · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)：

| 表示 | 优点 | 缺点 |
| --- | --- | --- |
| 栅格地图 | 提供度量信息，易理解、处理和融合 | 依赖传感器，数据量大 |
| 几何特征地图 | 提供度量信息，存储量较小 | 数据关联困难 |
| 拓扑地图 | 存储空间小，路径规划效率高 | 无度量信息，不适于定位 |

同页还有直接表征法：省去特征或栅格表示这一中间环节，直接用传感器读取的数据来构造机器人的位姿空间。

**占用栅格地图（Occupancy Grid Map）** 把环境离散成栅格单元，每个栅格储存该单元被障碍物占用的概率值 [第6讲 地图与定位.pptx · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。这一讲引的原始文献是 Moravec 与 Elfes 1985 年的宽角声呐高分辨率地图论文。

**特征地图** 描述的是特征元素的集合与子地图的集合，创建过程是获取感知数据 → 特征提取 → 坐标转换 → 地图融合 [第6讲 地图与定位.pptx · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。

这三种表示对应着三个不同的用途：栅格给几何感知用，特征地图给定位用，拓扑地图给路径规划用。课件开篇的「终极三问」把它们串起来——我在哪里（定位）、这是什么地方（建图）、我到哪里去（路径规划）[第6讲 地图与定位.pptx · p.3](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。

### 定位与建图的统一模型

定位和建图共用一套动态系统模型。动态系统通过两个方程建模 [第6讲 地图与定位.pptx · p.6](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)：状态转移方程说「上一时刻的状态经过转移函数，再叠加状态噪声，变成当前时刻的状态」，测量方程说「当前状态经过测量函数，再叠加测量噪声，变成传感器读到的观测」。两个方程写在一起就是这套模型：

$$
x_k = f_k(x_{k-1}, u_{k-1}) + v_{k-1}, \qquad z_k = h_k(x_k, u_k) + n_k
$$

转移函数与测量函数都可能是非线性的，状态噪声与测量噪声也可能是非 Gaussian 的，这是后面必须用 EKF、UKF、粒子滤波乃至非线性最小二乘来求解的原因 [第6讲 地图与定位.pptx · p.7](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。

这两条方程是后面所有定位算法的骨架。定位技术按实现方式分成六类 [第6讲 地图与定位.pptx · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)：航迹推算（Dead-Reckoning，用里程计和罗盘获取位移与转向，靠状态转移方程推算位置，随时间累计会发散）、惯性导航（INS）、GPS 定位、路标定位、地图匹配定位、同时定位与建图（SLAM）。

IMU 的坑集中在三类误差上 [第6讲 地图与定位.pptx · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)：

- **偏移误差**：陀螺仪和加速度计即使在没有旋转或加速的情况下也会有非零输出。加速度计输出做两次积分得到位移，即使很小的偏移误差也会被放大，随时间推进位移误差不断积累，最终没法再跟踪无人车的位置。
- **比例误差**：所测量的输出与被检测输入的变化之间的比率，随时间推进造成的位移误差也会不断积累。
- **白噪声**：与前两者不同，它不随时间系统性积累。

GPS 与 IMU 的互补性决定了融合方案：GPS 定位相对精准但更新频率低，惯性传感器的定位误差随时间累积但更新频率高，能在短时间内提供稳定的实时位置更新，融合方法是基于卡尔曼滤波 [第6讲 地图与定位.pptx · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。辅助课件里 GPS 的城市遮挡与反射问题、混合定位、捷联惯导定位、环境特征定位、多传感器融合定位各占一页 [第2讲 无人车基本框架.pptx · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx)。

**SLAM** 的定义是：移动机器人从一个未知的位置出发，在不断运动过程中，根据自身位姿估计和传感器对环境的感知构建增量式地图，同时利用该地图更新自己的定位 [第6讲 地图与定位.pptx · p.21](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。课件把它的求解方法分成两大类 [第6讲 地图与定位.pptx · p.22](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)：

| 大类 | 方法 |
| --- | --- |
| Bayesian Filtering | Gaussian Filters（EKF / UKF）、Particle Filters |
| Optimization | Linear Methods、Nonlinear Least Squares、Relaxation |

基于激光的 SLAM 与基于视觉的 SLAM 各有一页，配真实环境图与测试结果 [第6讲 地图与定位.pptx · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC6%E8%AE%B2%20%E5%9C%B0%E5%9B%BE%E4%B8%8E%E5%AE%9A%E4%BD%8D.pptx)。

### OpenDRIVE：用文件格式描述道路网络

OpenDRIVE 是描述道路网络逻辑描述的开放文件格式，它包含真实道路网络、免费且与厂商无关、拥有广泛的国际用户群，自 2006 年以来积累了大量用例 [第三章 高精度地图.pdf · p.15](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=15)。配套工具是 ordViewer64.1.9.1，一个 Linux 工具，不开源 [第三章 高精度地图.pdf · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=16)。

OpenDRIVE 的坐标系有两套 [第三章 高精度地图.pdf · p.17](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=17)：一套是 Track coordinates（Frenet coordinate），沿参考线从 s=0 走到 s=123.45；一套是 Inertial coordinates。

惯性系统给出三个姿态角和三个轴向 [第三章 高精度地图.pdf · p.18](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=18)：

](images/fig-03-p18.png)

跟踪坐标系则是三个分量 [第三章 高精度地图.pdf · p.19](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=19)：S 是沿参考线的位置，T 是横向位置，H 是高度。这两套坐标系的换算，就是把惯性坐标下的一个点投影到参考线的切向与法向：

$$
s = s_0 + (\mathbf{p} - \mathbf{r}_0) \cdot \hat{\mathbf{T}}, \qquad t = (\mathbf{p} - \mathbf{r}_0) \cdot \hat{\mathbf{N}}
$$

其中 $\mathbf{p}$ 是待变换的点，$\mathbf{r}_0$ 是参考线起点，$\hat{\mathbf{T}}$ 与 $\hat{\mathbf{N}}$ 是参考线在该处的切向与法向单位向量。切向分量给纵向里程 $s$，法向分量给横向偏移 $t$，这就是 Frenet 坐标的定义。ROAD OF OPENDRIVE 这一页就是把道路画成 s-t 平面 [第三章 高精度地图.pdf · p.20](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=20)。

### HD Map 怎么造出来

Apollo 的 HD Map 生产被拆成四个阶段 [第三章 高精度地图.pdf · p.10](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=10)：

](images/fig-03-p10.png)

| 阶段 | 具体动作 |
| --- | --- |
| 数据采集 | 道路 |
| 数据处理 | 点云拼接、底图生产 |
| 元素识别 | 基于深度学习的元素识别、基于深度学习的点云分类 |
| 人工验证 | 车道线与路沿、信号灯与标志牌、虚拟道路与逻辑关系 |

采集用的车是 Apollo Collective Car，车上装了 Camera、Lidar、GPS Antenna，后备箱里是 Industrial PC、NVME SSD Hard Disk、Trunk Liner、GPS Receiver [第三章 高精度地图.pdf · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=11)。

底图生产靠 SLAM，课件给的流程是 Sensor Data → Visual Odometry → Optimization → Mapping，其中 Loop Closing 把 Sensor Data 和 Optimization 两条线闭合起来 [第三章 高精度地图.pdf · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=12)。点云建图那一节重画了同一张流程图 [第三章 高精度地图.pdf · p.23](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=23)。

制图过程中用到的技术按模块归成四块 [第三章 高精度地图.pdf · p.13](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=13)：

](images/fig-03-p13.png)

- **点云处理**：运动补偿、三维模型 → 特征提取、点云融合 → 点云识别。
- **视觉处理**：图像识别。
- **多传感器融合**：3D-2D 投影 → 元素跟踪、聚类 → 过滤补全。
- **多传感器定位**：Lidar、Camera、IMU、GPS 一起用。

### 点云建图与传感器融合

Lidar 和 Image 这一页把融合方案分成三条路 [第三章 高精度地图.pdf · p.24](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=24)：

- **Sensing fusion，Camera-Lidar-Radar**：深度信息是 XYZ，图像信息是 RGB，两者联合做目标分类与检测。
- **iDAR**：Camera 的 RGB 图像 + Lidar 的 3D 点云，代表方案有 AEye、Mobileye。
- **SLAM**：传感器位姿估计加粒子滤波。
- **Mapping**：重建全局地图。

课件另有一页点云建图的演示截图，含论文标题与点云地图画面 [第三章 高精度地图.pdf · p.25](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=25)。辅助课件把高精地图的采集、构建、定位、与规划的关系各占一页 [第2讲 无人车基本框架.pptx · p.31](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx)。

### V2X：车路协同的通信侧

V2X 那一页把两类「传感器」放在一起对比 [第三章 高精度地图.pdf · p.27](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=27)：

](images/fig-03-p27.png)

| 传感器 | 特性 |
| --- | --- |
| HD Map | Static Perception |
| V2X | Low latency、High robust、Dynamic network、Beyond visual range |

网络一侧有三种站点角色：ITS Vehicle station、ITS Center station、ITS Roadside station。

两套标准在竞争 [第三章 高精度地图.pdf · p.28](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=28)：

| 标准 | 地域 | 形态 | 频段 | 代表 |
| --- | --- | --- | --- | --- |
| C-V2X（LTE-V） | 中国 | Share station、Direct mode | — | 大唐、华为 |
| DSRC | 美国、日本 | WIFI | 5.8GHz | Sony、AT&T |

课件后面还列了 5G+V2X。辅助课件把 V2X 的构成、意义、优点、标准之争、行业标准、LTE-V 的后来居上与应用场景各占一页 [第2讲 无人车基本框架.pptx · p.35](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx)。

主参考课件结尾一页把这四块内容并排列出：HD MAP OVERVIEW、OpenDrive Format、SLAM 的 Joint Mapping、V2X [第三章 高精度地图.pdf · p.29](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E8%AF%BE%E4%BB%B6/%E7%AC%AC%E4%B8%89%E7%AB%A0%20%E9%AB%98%E7%B2%BE%E5%BA%A6%E5%9C%B0%E5%9B%BE.pdf#page=29)。
