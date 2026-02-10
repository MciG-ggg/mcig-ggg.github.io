---
title: ROS2 CLI
date: 2026-02-10
timestamp: 2026-02-10T15:59:27+08:00
slug: ros2-cli
description: ROS2-CLI ``cardlink url: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html title: "Beginner: CLI tools — ROS 2 Documentation: Jazzy documentation" host: docs.ros.org ` Node `shell // r...
category: note
tags:
  - Area/AI/EmbodiedAI/ROS2
---

# ROS2-CLI


```cardlink
url: https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html
title: "Beginner: CLI tools — ROS 2 Documentation: Jazzy  documentation"
host: docs.ros.org
```

## Node
![](./images/ROS2-节点-ros架构.png)

```shell
// ros2 run <工具包名> <可执行文件名>
// 启动 /turtlesim 节点
ros2 run turtlesim turtlesim_node

// 启动 /teleop_turtle 节点(键盘操控turtle)
ros2 run turtlesim turtle_teleop_key

// 启动rqt_graph(查看各节点之间的关系)
ros2 run rqt_graph rqt_graph
```

![](./images/ROS2-CLI-rqt-graph.png)

## Topic
Topic 基于**发布订阅**模式
![](./images/ROS2-CLI-Topic.png)
### Show Topic List
```shell
// show topic list
ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

// with the topic type appended in brackets:
ros2 topic list -t
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]

/turtle1/cmd_vel [geometry_msgs/msg/Twist]
│                │  │              │
│                │  │              └── 具体消息名称(message type/Tonic type)
│                │  └── 包名（package）
│                └── 类型类别（msg = 消息，srv = 服务，action = 动作）
└── Topic 名称

```

| 类比                                   | 解释                                       |
| ------------------------------------ | ---------------------------------------- |
| **Topic** = 快递单号                     | `/turtle1/cmd_vel`                       |
| **Topic Type / Message Type** = 包裹规格 | `geometry_msgs/msg/Twist`（规定是 " 速度指令 " 这种包裹） |
| **Message 数据** = 具体包裹内容              | `{linear: {x: 1.0}, angular: {z: 0.5}}`  |

| 中文       | 英文                 | 缩写        |
| -------- | ------------------ | --------- |
| 消息类型     | Message Type       | `.msg`    |
| 服务类型     | Service Type       | `.srv`    |
| 动作类型     | Action Type        | `.action` |
| **接口类型** | **Interface Type** | 三者统称      |


### 查看 Topic 发送出来的 Data
```shell
// see the topic data
// At first, this command won’t return any data. That’s because it’s waiting for `/teleop_turtle` to publish something.
ros2 topic echo <topic_name>

linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0

此时再看rqt_graph， 发现/turtle1/cmd_vel 这个topic后面又多了一个节点接收数据:/_ros2cli_26646 数字26646为签名执行 topic echo的进程的pid

```

![](./images/ROS2-CLI-readtopic.png)

### 查看 Topic Info
```shell
ros2 topic info /turtle1/cmd_vel (可--verbose)
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 2
```

### 查看 Message Type 具体定义
```shell
ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.
    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```

### 现在知道了消息结构，可以直接在命令行构造消息发布
```shell
ros2 topic pub <topic_name> <msg_type> '<args>'

// 具体如下：
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

// 还可以发布空消息（发布消息类型的默认值，频率为 1 Hz）
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist
// 上面的等价于 ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 1 
```

### 检查 Topic 发布的速率, 带宽
```shell
// 速率
ros2 topic hz /turtle1/pose
average rate: 59.354
  min: 0.005s max: 0.027s std dev: 0.00284s window: 58
  
// 带宽
ros2 topic bw /turtle1/pose
Subscribed to [/turtle1/pose]
1.51 KB/s from 62 messages
    Message size mean: 0.02 KB min: 0.02 KB max: 0.02 KB
```

## Service

服务是 ROS 图中节点的另一种通信方式。服务基于 **call-and-response** 模式，而非发布者 - 订阅者模式。虽然主题允许节点订阅数据流并获得持续更新，但服务只有在客户端特别调用时才会提供数据。

![](./images/ROS2-CLI-Service.png)
```shell
ros2 service list
...
/teleop_turtle/describe_parameters
/teleop_turtle/get_parameter_types
/teleop_turtle/get_parameters
/teleop_turtle/list_parameters
/teleop_turtle/set_parameters
/teleop_turtle/set_parameters_atomically
...
```
你会看到两个节点都有相同的六个服务，且名称中带有 `参数` 。ROS 2 中几乎每个节点都有这些基础设施服务，参数就是基于这些服务构建的。

### 查看服务类型
```shell
// ros2 service type <service_name>
ros2 service type /clear
std_srvs/srv/Empty
```
说明这个 `/clear` 服务是 `Empty` 类型，即*在发送请求时不发送数据，接收响应时也不接收数据*。

### 查看服务列表
同理，也可以直接 list 出服务列表，显然 `-t` 可以显示服务类型。
```shell
ros2 service list -t
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
...
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
...
```

### 其他命令
和 Topic 大差不差，直接列出
```shell
ros2 service info <service_name>

ros2 service find <type_name>

ros2 interface show <type_name>
// ros2 interface show std_srvs/srv/Empty
// ---
// 上面：`---` 将请求结构（上图）与响应结构（下文）分开， 这里`---`上面下面都没东西，说明request和response都为空

ros2 service call <service_name> <service_type> <arguments>
// ros2 service call /clear std_srvs/srv/Empty
```

### Service Echo
```shell
ros2 service echo <service_name | service_type> <arguments>
// `ROS2 服务 Echo` 依赖于服务客户端和服务器的服务introspection，而该内省默认是被禁用的。要启用此功能，用户必须在创建服务客户端或服务器后调用 `configure_introspectio`

ros2 launch demo_nodes_cpp introspect_services_launch.py

// 启用introspection
ros2 param set /introspection_service service_configure_introspection contents
ros2 param set /introspection_client client_configure_introspection contents

// 用echo观察这两个service的通信
ros2 service echo --flow-style /add_two_ints
 info:
   event_type: REQUEST_SENT
   stamp:
     sec: 1709408301
     nanosec: 423227292
   client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 21, 3]
   sequence_number: 618
 request: [{a: 2, b: 3}]
 response: []
 ---
 info:
   event_type: REQUEST_RECEIVED
   stamp:
     sec: 1709408301
     nanosec: 423601471
   client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 20, 4]
   sequence_number: 618
 request: [{a: 2, b: 3}]
 response: []
 ---
 info:
   event_type: RESPONSE_SENT
   stamp:
     sec: 1709408301
     nanosec: 423900744
   client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 20, 4]
   sequence_number: 618
 request: []
 response: [{sum: 5}]
 ---
 info:
   event_type: RESPONSE_RECEIVED
   stamp:
     sec: 1709408301
     nanosec: 424153133
   client_gid: [1, 15, 0, 18, 250, 205, 12, 100, 0, 0, 0, 0, 0, 0, 21, 3]
   sequence_number: 618
 request: []
 response: [{sum: 5}]
 ---
```

## Parameters

### 查看节点参数
```shell
ros2 param list
/teleop_turtle:
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```

### 查看参数类型和值
```shell
// ros2 param get <node_name> <parameter_name>
ros2 param get /turtlesim background_g
Integer value is: 86
```

### 更改参数类型和值
```shell
// ros2 param dump <node_name>
// 可以把输出重定向到yaml文件里
ros2 param dump /turtlesim 
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
    qos_overrides:
      /parameter_events:
        publisher:
          depth: 1000
          durability: volatile
          history: keep_last
          reliability: reliable
    use_sim_time: false
```

### 节点运行时加载参数文件
```shell
// ros2 param load <node_name> <parameter_file>
ros2 param load /turtlesim turtlesim.yaml
Set parameter background_b successful
Set parameter background_g successful
Set parameter background_r successful
Set parameter qos_overrides./parameter_events.publisher.depth failed: parameter 'qos_overrides./parameter_events.publisher.depth' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.durability failed: parameter 'qos_overrides./parameter_events.publisher.durability' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.history failed: parameter 'qos_overrides./parameter_events.publisher.history' cannot be set because it is read-only
Set parameter qos_overrides./parameter_events.publisher.reliability failed: parameter 'qos_overrides./parameter_events.publisher.reliability' cannot be set because it is read-only
Set parameter use_sim_time successful
```

注意，只读参数无法在运行时修改，所以上面有报错。要使只读参数从文件中加载得来，只能用下面的方法，从启动时就设置好

### 节点启动时加载参数文件
```shell
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

## Action

Action 是 ROS 2 中的一种通信类型，旨在执行长期任务。它们由三个部分组成：目标、反馈和结果。

Action 基于 Topic 和 Service。，但 Action 可以被取消。它们还能提供稳定的反馈，而服务通常只返回一个 response。

![](./images/ROS2-CLI-Action.png)

### 展示 Action 信息
```shell
ros2 node info /turtlesim
···
···
Action Servers:
	/turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
Action Clients:
  
ros2 node info /turtle_teleop_key  
···
···
Action Servers:
Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
```

### 展示 Action 列表
```shell 
// ros2 action list
// -t 显示Action Type
ros2 action list -t
/turtle1/rotate_absolute [turtlesim/action/RotateAbsolute]
```

### 展示 Action 类型
```shell
// 也可以直接用下面的方式展示action type
ros2 action type /turtle1/rotate_absolute
turtlesim/action/RotateAbsolute
```

### 展示 Action 信息
```shell
ros2 action info /turtle1/rotate_absolute
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim
```

### 展示 Action 的结构
```shell
ros2 interface show turtlesim/action/RotateAbsolute
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

## Launch

启动文件允许你同时启动并配置包含 ROS 2 节点的多个可执行文件

```shell
ros2 launch turtlesim multisim.launch.py
```

launch file 如下：
```python title:multisim.launch.py
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace='turtlesim1', package='turtlesim',
            executable='turtlesim_node', output='screen'),
        launch_ros.actions.Node(
            namespace='turtlesim2', package='turtlesim',
            executable='turtlesim_node', output='screen'),
    ])
```

## Bag

`ROS2 Bag` 可以记录发布到 Topic 的消息数据, 以及重新回放 (play)

| 功能     | 命令                 | 用途                     |
| ------ | ------------------ | ---------------------- |
| **录制** | `ros2 bag record`  | 保存话题数据到 `.db3` 文件      |
| **回放** | `ros2 bag play`    | 按原始时间戳发布录制的数据          |
| **查看** | `ros2 bag info`    | 查看 bag 文件内容摘要          |
| **转换** | `ros2 bag convert` | 格式转换（如 sqlite3 → mcap） |


```shell
// ros2 bag record <topic_name>
ros2 bag record /turtle1/cmd_vel
[INFO] [rosbag2_storage]: Opened database 'rosbag2_2019_10_11-05_18_45'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```