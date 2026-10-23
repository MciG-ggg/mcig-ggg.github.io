## 课后实践：APOLLO 平台大作业

前面六讲讲的是原理，这一章讲的是动手。课程把四次作业挂在 Apollo 平台上，从可视化调试一直到控制参数分析，正好覆盖一条完整的链路：确认数据进得来、算出走哪条线、算出走多快、最后把指令变成方向盘和油门。四次作业不是四个孤立的实验，它们对应课程正文里的具体章节：路径规划和速度规划对应第 5 章的路径规划，车辆控制对应第 2 章的纵向控制与横向控制，作业一用到的 cyber 消息总线与数据流对应第 2 章的软件架构。

```mermaid
flowchart LR
  A["Dreamview / cyber<br/>作业一：数据进得来"] --> B["Planning 路径规划<br/>作业二：走哪条线"]
  B --> C["Planning 速度规划<br/>作业三：走多快"]
  C --> D["Control<br/>作业四：怎么执行"]
  D --> E["车辆"]
```

### 四次作业与课程章节的对应

| 作业 | Apollo 模块 | 课程章节 | 讲次材料 | 这一环要回答的问题 |
| --- | --- | --- | --- | --- |
| 一 | cyber / Dreamview / PNC Monitor | 第 2 章 无人车基本框架（软件架构与数据控制流） | 第 2 讲 [第2讲 无人车基本框架.pptx · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx) | 数据有没有真的进到模块里 |
| 二 | Planning 路径规划 / 开放空间规划 | 第 5 章 路径规划 | 第 5 讲 [第5讲 路径规划.pptx · p.9](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC5%E8%AE%B2%20%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.pptx) | 走哪条线，路边界在哪 |
| 三 | Planning 速度规划 | 第 5 章 路径规划（轨迹生成与最优轨迹选择） | 第 5 讲 [第5讲 路径规划.pptx · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC5%E8%AE%B2%20%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.pptx) | 走多快，为什么慢下来 |
| 四 | Control（纵向 PID / 横向 LQR） | 第 2 章 无人车基本框架（纵向控制与横向控制） | 第 2 讲 [第2讲 无人车基本框架.pptx · p.49](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx) | 规划指令怎么变成方向盘和油门 |

作业一是整条链路的入口检查，作业四是出口检查，中间两次作业都在 Planning 内部，一次管横向一次管纵向。

### 作业一：Dreamview 与 PNC Monitor —— 把车变成可观测量

**任务**是三件事：起 Dreamview、用 cyber 调试工具打开数据包、用 PNC_monitor 观察车辆运动参数 [作业一：基于百度APOLLO自动驾驶平台的车辆规划可视化与调试.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%80%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E8%87%AA%E5%8A%A8%E9%A9%BE%E9%A9%B6%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E8%A7%84%E5%88%92%E5%8F%AF%E8%A7%86%E5%8C%96%E4%B8%8E%E8%B0%83%E8%AF%95.docx)。**Apollo 模块**上，它不碰任何感知或规划逻辑，只提供调试通道：Dreamview 是基于 Web 的可视化前端，cyber 是 Apollo 内部的消息总线，PNC_monitor 是挂在 Planning 与 Control 上的曲线面板。这一点正对着第 2 讲讲软件架构的那几页——模块之间不直接调用，全靠 cyber 上的一组 topic 交换消息 [第2讲 无人车基本框架.pptx · p.16](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx)。

先起 Dreamview。`bash scripts/bootstrap_neo.sh` 会在终端打印一行 `Dreamview is running at http://localhost:8888`，浏览器打开这个地址就是可视化界面。启动脚本会在 Docker 里拉起全部 Apollo 节点，终端输出比网页加载早几十秒，看到这行日志再去开浏览器。

再回放离线数据包。`cyber_recorder play -f demo_3.5.record -l` 把录制好的 `.record` 文件灌回系统。`-l` 是循环播放，回放期间 Dreamview 的 3D 视图里能看到车辆沿车道行驶，感知输出的路沿和车道线以绿色线条叠在场景上。

![回放离线数据包时的 Dreamview 界面：3D 视图里自车沿青色车道前进，绿色长线是感知输出的路沿，黄色虚线是车道中心线，右上角显示 32 km/h、Brake 0%、Accelerator 0%、挡位 P，底部 Module Delay 表里 Chassis、Control、Localization、Planning 等模块耗时全部标红，Console 刷出 Monitor timed out 与 You haven't selected a vehicle yet!](images/fig-hw-01.png)

这张截图暴露了第一个坑。回放时 Module Delay 表里每个模块都是秒级量级的红色超时，Console 里交替刷 `Monitor timed out` 和 `You haven't selected a vehicle yet!`。这不是规划算错了，是数据包回放和模块启动的时序没对齐：录制时用的是实车，模块订阅的 topic 在回放环境里没有对应发布者。查这个问题要从通道层入手，而不是去看规划曲线。

第三步用 `cyber_monitor` 看通道。Apollo 的每个模块都通过 cyber 的 topic 通信，cyber_monitor 列出所有通道名和它们的 FrameRatio。

![cyber_monitor 列出的通道表：绿色通道名有发布者，红色表示没有发布者，右列 FrameRatio 大多为 0.00，只有 /apollo/monitor 是 0.67、/apollo/hmi/status 是 0.37](images/fig-hw-02.png)

图里几十个通道的 FrameRatio 都是 0.00，`/apollo/planning`、`/apollo/control`、`/apollo/perception/obstacles` 这些关键通道同样为 0。FrameRatio 为 0 表示这条通道上收不到任何消息，模块自然不会产出结果。**这是排查的第一站**：曲线为空时，先确认通道有数据，再去看模块。

通道通了之后，PNC_monitor 才有东西可看。它有三个页签 Planning、Control、Latency。作业报告要求从 PNC_monitor 里挑三个参数曲线做分析，报告选的是轨迹、速度、加速度。轨迹是车辆在平面上的位置变化，速度和加速度是时间序列。速度曲线显示车辆从约 13 m/s 一路爬到 58 m/s，再回落到 46 m/s。

![Speed 曲线：横轴时间从 1545095776 到 1545095786，青色 plan 曲线从约 13 m/s 升到 58 m/s 再回落到 46 m/s，蓝色 real 与 plan 基本重合](images/fig-hw-04.png)

**real 与 plan 几乎完全重合**，说明这段数据里车辆的实际运动跟规划输出的目标值一致，控制环节跟得上。

### 作业二：路径规划 —— 走哪条线

**任务**是对 Planning 模块建立概念，并在 Dreamview 的仿真环境里跑多个场景，观察路径规划、速度规划、开放空间规划三个过程 [作业二：基于百度apollo平台的多场景下路径规划.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%BA%8C%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6apollo%E5%B9%B3%E5%8F%B0%E7%9A%84%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.docx)。**Apollo 模块**上，这一章对应 Planning 的两个子任务之一：路径规划（横向决策，决定走哪条线），另一个是速度规划（纵向决策），留到作业三。第 5 讲把这件事拆成全局路径规划与局部运动规划：全局给出起点到终点的无碰撞路径，局部根据感知反馈的障碍信息在运动过程中随时重算避障路径 [第5讲 路径规划.pptx · p.3](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC5%E8%AE%B2%20%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.pptx)。

搭环境分几步。起 Dreamview 之后，在顶栏把 vehicle 下拉选成 Mkz Standard Debug，地图选 Apollo Virtual Map，Others 面板里打开 PNC Monitor 和 Sim Control。等屏幕中央出现 Mkz 车模和地图，就算进入仿真模式。

这里有第二个坑。**Module Delay 表里 Control 和 PerceptionObstacle 显示成横杠**，意思是这两个模块没跑起来。仿真模式不接真实传感器，感知模块没有数据源，自然不启动。随后的 Console 反复刷 `Control triggers safe mode`，原因是控制模块没上线，安全机制一直拦着。解决它要在左侧 Module Controller 面板里手动启动 Planning、Prediction、Routing 三项，同时注意 Components 列表里的 GPS、Lidar 128、Recorder 报 FATAL 在仿真里是正常的，Localization 报 ERROR 才需要单独确认——定位没有输出，Planning 就拿不到参考线。

模块起来之后，场景从 Profile 页签的 Scenario Profiles 里选，course 场景集提供了自车道内行驶、绕行障碍物、借道绕行、换道这几个案例，选好场景点开始仿真。想看得更清楚，切到 Layer Menu，把 Point of View 选成 Map，切到俯视图。

![俯视图里的规划结果：红色是自车轨迹，黄色是规划路径，路径向上绕过橙色的 CAUTION 障碍框，青色是路径边界带](images/fig-hw-09.png)

俯视图里能看清三个要素：**黄色是规划出来的路径**，**青色是路径边界**（规划允许车辆走的车道范围），**红色是车辆已经走过的轨迹**。规划路径必须落在边界带内，超出边界的部分无效。作业报告还要求修改路径规划参数再观察路径变化，以及调整目标点位置看规划器如何重算。

借道绕行和开放空间规划是两个更复杂的场景。借道绕行指的是借用相邻车道通过；开放空间规划（Open Space）指的是在没有车道线约束的区域（比如停车场）里找一条可行轨迹，它用完全不同的算法。

![Open Space Trajectory Optimizer 可视化：图例列出 Bdr1 到 Bdr4 四条候选边界、Smooth 平滑轨迹和 WarmStart 热启动初值，横轴 x 从 -13.6 到 19.3 米，纵轴 y 从 -7.0 到 26.6 米](images/fig-hw-12.png)

这张图是开放空间规划的调试视图。**Bdr1 到 Bdr4 是四条候选边界**，每条边界围出一个可行的搜索区域，算法在区域内找轨迹。**Smooth 是最终输出的平滑轨迹**。**WarmStart 是热启动初值**，上一帧的解被拿来当初值。带 WarmStart 的算法每次迭代从上一次的解出发，比冷启动快。

完整跑通泊车流程，Dreamview 里的状态变化会在 Scenario History 里留下记录。

![泊车过程的 Dreamview 全景：左侧 3D 视图里车模倒进泊车位（-4 km/h，挡位 R），青色是规划轨迹；右侧 PNC Monitor 的 Scenario History 依次记 LANE_FOLLOW、PARK_AND_GO CHECK、VALET_PARKING APPROACHING_PARKING_SPOT、VALET_PARKING PARKING，下方嵌着 Open Space Trajectory Optimizer 小图](images/fig-hw-13.png)

Scenario History 从 `LANE_FOLLOW`（沿车道行驶）切到 `PARK_AND_GO`（泊车代客模式）再到 `VALET_PARKING`（代客泊车），最后 `APPROACHING_PARKING_SPOT` 和 `PARKING` 表示车已对准车位、正在泊入。**这条序列把一次泊车要经历哪些决策状态完整列了出来**，状态机是理解泊车行为的关键。

开放空间规划需要额外的模块：除了 Planning、Prediction、Routing，还要启动 TaskManager。发泊车请求走 Route Editing，在 PNC Monitor 里选 Routing Editing，起点选停车位前主路上的一点，终点选一个车位，然后 Send Routing Request。

> [!WARNING] **仿真模式下的三个常见误判**
>
> - Module Delay 表里某模块显示横杠，是该模块没启动，不是超时。
> - Components 里 GPS、Lidar、Recorder 报 FATAL 在仿真里是正常的，Localization 报 ERROR 才需要查。
> - Console 里的 `Control triggers safe mode` 说明控制或安全模块没上线，路径算得再对车也不会动。

### 作业三：速度规划 —— 走多快

**任务**是在加速超车、减速让行、掉头这类场景下观察速度规划、采集速度数据、修改速度规划参数看效果 [作业三：基于百度APOLLO平台多场景下的速度规划与数据采集.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E4%B8%89%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E5%A4%9A%E5%9C%BA%E6%99%AF%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E8%A7%84%E5%88%92%E4%B8%8E%E6%95%B0%E6%8D%AE%E9%87%87%E9%9B%86.docx)。**Apollo 模块**上对应 Planning 的第二个子任务，在课程里是第 5 讲「轨迹生成与轨迹选择」那一段：已知当前速度与目标速度，要生成一条平滑的速度曲线，避免急加速、急减速 [第5讲 路径规划.pptx · p.11](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC5%E8%AE%B2%20%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.pptx)。速度规划要考虑的不只是目标速度，还有车辆动力学、道路限速、交通情况、安全距离。

场景在 course 场景集里，选减速让行和加速超车两个，再切到弯道掉头。速度规划的结果画在 S-T 图上。**S 是沿路径的纵向距离（米），T 是时间（秒）**，把空间和时间分开画，于是「走多远」和「什么时候到」变成一条曲线上的两个维度。这个 s 正是第 5 讲里的 Frenet 坐标系的纵向分量。

![笛卡尔坐标系与 Frenet 坐标系的对照：左边是 x 与 y 两个笛卡尔方向，参考线穿过原点；右边 s 沿参考线方向，d 是垂直于参考线的偏出距离](images/fig-l0-l5-09.png)

换成 Frenet 之后，路径上每一点都有两个坐标：沿参考线走了多远 s，横向偏出参考线多少 d。路径规划在 d 上做决策，速度规划在 s 上做决策，两层共用同一条参考线。

![加速超车场景的 Planning S-T Graph：横轴 t 从 -2 到 10 秒，纵轴 s 从 -10 到 220 米，两幅图都有红色 DEST_STOP 边界矩形；上图障碍物是灰色离散采样点、标签 8839_0 UNKNOWN，下图多出一条连续白色自车曲线、标签变成 8839_0 OVERTAKE](images/fig-hw-15.png)

这张图分成上下两幅。**上面一幅只有障碍物**，被离散采样成一串灰点，每个点带一个决策标签，这里标的是 `8839_0 UNKNOWN`。**下面一幅多出一条连续的白色自车曲线**，同时障碍物的标签变成 `8839_0 OVERTAKE`，意思是规划器决定超车。红色 `DEST_STOP` 矩形是硬约束：曲线撞进这个区域就得停车。减速让行场景的差别体现在同一个标签变成 `8839_0 YIELD`，曲线先压到 s=-10 再缓慢爬升。

**同一个障碍物，决策标签从 OVERTAKE 变成 YIELD，整条速度曲线就得重画**。这解释了速度规划为什么必须在 S-T 空间做：在空间图上「绕过去」和「停下来等」看起来只是两个不同的轨迹点，但在 S-T 图上它们对应的是完全不同的曲线族。

S-T 曲线怎么算出来的，Apollo 提供了一个离线工具。`modules/planning/tools/plot_st_nlp.py` 读 planning.INFO 日志，按时间戳找到那一帧的规划数据并画出来。

![Jupyter notebook 里跑 plot_st_nlp.py 的结果：命令为 run modules/planning/tools/plot_st_nlp.py -f /opt/apollo/neo/data/log/planning.INFO -t 17:19:29，frame seq num 3842，下方五张子图依次是 s-t、v-t、a-t、v-s 上界和 sk-s 曲率曲线](images/fig-hw-17.png)

时间戳 `17:19:29` 来自 Data Recorder 记录的时刻，不是随手写的。工具输出的五张图里，**v-s 上界曲线**最值得看：速度上界在 s=70 突然从 16.8 掉到 8 m/s，同位置的曲率曲线冲到峰值 0.155。原因是前方有个弯，规划器必须先减速再过弯。**速度规划不只看障碍物，弯道曲率也会压住速度上界**。这也正是第 5 讲讲的约束条件：不能碰撞障碍物、不能超过允许的最大速度与最大加速度、转弯半径或者说曲率不能超过允许的最大值 [第5讲 路径规划.pptx · p.12](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC5%E8%AE%B2%20%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92.pptx)。

作业报告还要求改参数看效果。速度规划的可调项包括最大速度限制、最小速度限制、加速度限制、目标速度、安全距离、道路限速：把最小速度限制调高，车辆会加速得更积极；把安全距离调小，车会更贴近前车。

### 作业四：车辆控制结果分析 —— 怎么执行

**任务**是回放实测数据包，用控制分析工具看不同控制参数下的效果，理解纵向 PID 和横向 LQR 两个控制器的作用 [作业四：基于百度APOLLO平台的车辆控制结果分析.docx · p.1](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E4%BD%9C%E4%B8%9A/%E4%BD%9C%E4%B8%9A%E5%9B%9B%EF%BC%9A%E5%9F%BA%E4%BA%8E%E7%99%BE%E5%BA%A6APOLLO%E5%B9%B3%E5%8F%B0%E7%9A%84%E8%BD%A6%E8%BE%86%E6%8E%A7%E5%88%B6%E7%BB%93%E6%9E%9C%E5%88%86%E6%9E%90.docx)。**Apollo 模块**上对应 Control 模块，它的位置是「理论数学模型与车辆实际执行机构之间的偏差」这一环，对应课程第 2 章讲的纵向控制与横向控制：纵向控制把目标状态里的速度、加速度落到发动机、电机、制动器上，横向控制把目标状态里的方向盘转角落到线控转向上 [第2讲 无人车基本框架.pptx · p.48](http://10.188.31.55:5244/Nextcloud/School/Auto-Drive-Intro/%E3%80%8A%E6%99%BA%E8%83%BD%E9%A9%BE%E9%A9%B6%E5%AF%BC%E8%AE%BA%E3%80%8BPPT-2020/%E7%AC%AC2%E8%AE%B2%20%E6%97%A0%E4%BA%BA%E8%BD%A6%E5%9F%BA%E6%9C%AC%E6%A1%86%E6%9E%B6.pptx)。

回放用的是同一个 `cyber_recorder play -f demo_3.5.record -l`，但控制模块要看的是自己的输出，得单独打开。在 Tasks 面板里关掉 Lock Task Panel，打开 PNC Monitor，再切到 Control 页签。

![PNC Monitor 的 Control 页：Speed 里 real 从 9.04 m/s 阶梯降到 8.84，plan 与 target 保持 8.85；Acceleration 里 real 在 -0.6 到 0.4 m/s² 之间抖动；Curvature 里 plan 曲率在 1545095758 附近形成 0.001 m⁻¹ 波峰](images/fig-hw-18.png)

Control 页的三张图对应控制器的三个输出：**Speed**（纵向）、**Acceleration**（纵向）、**Curvature**（横向，路径的曲率）。这张图上 real（实际值）和 plan（规划值）几乎平行，速度差在 0.2 m/s 以内，说明控制精度不错。灰色 autoModeZone 覆盖的那段是自动驾驶未激活的区间。

横向控制的结构在第 2 讲给的是一张框图。

![横向控制基本结构框图：目标状态进入整车控制器，整车控制器分出线控转向、电机控制、制动控制三支，分别驱动转向系统、电动机与刹车系统，三路汇入车辆；车辆状态（横摆角速度、侧偏角、侧向-垂向-纵向加速度）经传感器送回整车控制器](images/fig-l0-l2-49.png)

框图左侧进的是目标状态，右侧是车辆，底部的传感器把车辆状态送回整车控制器。**横向控制是一个闭环**：整车控制器根据横向误差算出转向指令，线控转向执行，传感器再把实际的车身姿态送回来。Apollo 的横向控制器用 LQR 实现。纵向控制那张框图结构类似，只是执行机构换成发动机、电机和制动器，传感器反馈的是位移、速度和加速度。

要看更细的误差分解，得用 `control_info` 工具。它是个 Jupyter notebook，在 `modules/tools/control_info/` 目录下打开，新建 python3 cell 跑命令。纵向参数在配置文件 `modules/control/conf/control_conf.pb.txt` 里。这个文件把控制模块的时间节奏写清楚了：**`control_period` 是 0.01 秒（100 Hz），`trajectory_period` 是 0.1 秒（10 Hz）**。控制器跑得比轨迹刷新快十倍，所以每个规划周期内它要基于同一条参考轨迹连续修正十次。`active_controllers` 同时列出 `LAT_CONTROLLER` 和 `LON_CONTROLLER`，横向和纵向控制器是并联的两套。

横向参数在 `lat_controller_conf` 块里，其中 `matrix_q` 是 LQR 的状态权重矩阵，四行分别是 0.05、0.0、1.0、0.0，倒车的 `reverse_matrix_q` 是 0.5、0.0、0.4、0.0。这四个数字对应四个误差项：**q0 是横向位置误差的权重，q1 是横向位置误差变化率的权重，q2 是航向误差的权重，q3 是航向误差变化率的权重**。

LQR 控制器选择转向角时用的代价函数写成四项误差的加权和，用来在「把车拉回参考线」和「让车头指向参考线方向」之间取舍。

$$
J = q_0 e_y^2 + q_1 \dot e_y^2 + q_2 e_\psi^2 + q_3 \dot e_\psi^2
$$

配成 0.05 / 0.0 / 1.0 / 0.0 意味着：位置误差权重 0.05，位置误差变化率不管，航向误差权重 1.0，航向误差变化率不管。**这份默认配置主要盯着航向**，位置只做弱约束。倒车的 `reverse_matrix_q` 换成 0.5 / 0.0 / 0.4 / 0.0，位置权重提到 0.5、航向降到 0.4——倒车时更依赖把车摆正而不是对准方向。

改完参数重启 control 模块，误差和转向的变化用 `control_info` 的四联图看。纵向的四张图是这条链路的四个环节：**Speed Info**（当前速度、预览速度参考、速度参考）、**Station information**（纵向位置误差）、**Speed error**（速度误差及其限幅）、**Acc Info**（加加速度输出）。速度在 65 到 78 秒之间掉到 0，说明这段是停车让行；`station_error_limited` 在 78 秒冲到 2.4 m，是这个位置误差被限幅了——**限幅值触顶意味着控制器已经把修正力度拉满**。

最后看轨迹。`Trajectory` 图把规划路径和实际轨迹叠在一起。

![Trajectory 轨迹对比：横轴 x 从 587550 到 587750，蓝虚线是 planning path，红线是 localization pose，绿线是 planning path remain；车从 x=587675、y=250 直线爬升，x=587690 处右转，弯道顶点到 x=587720、y=480，红线与绿线几乎重合](images/fig-hw-23.png)

三条线的含义：**蓝虚线 `planning path` 是规划出的完整路径，红线 `localization pose` 是定位给出的实际行驶轨迹，绿线 `planning path remain` 是还没走完的规划段**。红线与绿线几乎完全重合，说明车辆严格贴着规划路径走完，横向控制的跟踪误差可以忽略。
