---
title: ROS2人脸识别using pixi
date: 2026-02-11
timestamp: 2026-02-11T18:12:15+08:00
slug: ros2using-pixi
category: note
tags:
  - Area/AI/EmbodiedAI/ROS2
---

# ROS2 人脸识别 Using Pixi


参考了 pixi 的 tutorial，以及一些别的教程
仓库链接是：[GitHub - MciG-ggg/face\_detect\_ros\_project](https://github.com/MciG-ggg/face_detect_ros_project)

```cardlink
url: https://pixi.prefix.dev/latest/tutorials/ros2/
title: "ROS 2 - Pixi"
description: "Pixi Documentation — Next-gen package manager for reproducible development setups"
host: pixi.prefix.dev
favicon: ../../assets/pixi.png
```

## 准备

### 初始化项目
先初始化项目。

```shell
// -c <channel> 加入指定的channel(理解为要添加的依赖包的来源)
pixi init face_detect_ros_project -c robostack-humble -c conda-forge
cd face_detect_ros_project
```

再加入 ROS 的 包
```shell
pixi add ros-humble-desktop ros-humble-turtlesim
```

可以用下面的方式运行 turtlesim 检查 ros2 环境是否下好
```shell 
pixi run ros2 run turtlesim turtlesim_node

// 或者
pixi shell
ros2 run turtlesim turtlesim_node
```

### 初始化 Interface 和 Service 模块
 
```shell
❯ pixi run ros2 pkg create interface --build-type ament_cmake --dependencies rosidl_default_generators sensor_msgs --license Apache-2.0 --destination-directory src

❯ pixi run ros2 pkg create face_detect_service --build-type ament_python --dependencies rclpy interfaces --license Apache-2.0 --destination-directory src
```

目前的目录结构如下：

```shell
face_detect_ros_project [ main][?][🧚 v0.63.2 default]
❯ eza --tree                                                                
.                                -- workspace name
├── pixi.lock
├── pixi.toml
└── src
   ├── face_detect_service       -- package name
   │  ├── face_detect_service    -- python module name
   │  │  └── __init__.py
   │  ├── LICENSE
   │  ├── package.xml
   │  ├── resource
   │  │  └── face_detect_service
   │  ├── setup.cfg
   │  ├── setup.py
   │  └── test
   │     ├── test_copyright.py
   │     ├── test_flake8.py
   │     └── test_pep257.py
   └── interface
      ├── CMakeLists.txt
      ├── include
      │  └── interface
      ├── LICENSE
      ├── package.xml
      └── src

```

### 编写 Srv 文件定义 Service 内容

在 `src/interface` 下创建 `srv/FaceDetector.srv`, 在这里编写 service 的定义

``` title:FaceDectector.srv
sensor_msgs/Image image
# Request
---
# Response
int16 number # 人脸数
float32 use_time # 人脸检测耗时
int32[] top # 人脸在图中的位置
int32[] right
int32[] bottom
int32[] left
```

然后还要修改 Cmake 和 package.xml

```cmake title:src/interface/Cmakelists.txt
// 添加下面的内容
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FaceDetector.srv"
  DEPENDENCIES sensor_msgs
)
```

上面这条语句是由 `rosidl_default_generators` 提供的宏（Macro），它的作用是告诉构建系统：**“请把这个 `.srv` 文件翻译成 C++ 和 Python 能够直接调用的代码库。”**

```xml title:src/sinterface/package.xml
// 在<package>里面，<export>外面添加这个
<member_of_group>rosidl_interface_packages</member_of_group>
```

ROS 2 使用一种名为 **Ament** 的构建系统。为了处理一些具有共性的包，Ament 允许将功能包划分为不同的“组”。

**Ament 是“包内”的：** 决定了**单个包**如何被构建（你写在 `CMakeLists.txt` 或 `setup.py` 里的代码）。

**Colcon 是“包外”的：** 决定了**一堆包**按什么顺序构建（你在终端输入的命令）。

当你写下 `<member_of_group>rosidl_interface_packages</member_of_group>` 时，你实际上是在对构建系统说：

> “我这个包是一个**接口定义包**，请把它当作 `rosidl_interface_packages` 这个大集体的一员。”

构建好了之后，就可以查看 Interface 了
```shell
❯ ros2 interface show interface/srv/FaceDetector                                                                               (face_detect_ros_project) 
sensor_msgs/Image image
        std_msgs/Header header #
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
                                     # Header frame_id should be optical frame of camera
                                     # origin of frame should be optical center of cameara
                                     # +x should point to the right in the image
                                     # +y should point down in the image
                                     # +z should point into to plane of the image
                                     # If the frame_id here and the frame_id of the CameraInfo
                                     # message associated with the image conflict
                                     # the behavior is undefined
        uint32 height                #
        uint32 width                 #
        string encoding       #
                              # taken from the list of strings in include/sensor_msgs/image_encodings.hpp
        uint8 is_bigendian    #
        uint32 step           #
        uint8[] data          #
# Request
---
# Response
int16 number # 人脸数
float32 use_time # 人脸检测耗时
int32[] top # 人脸在图中的位置
int32[] right
int32[] bottom
int32[] left

```

正和我们定义的一样。

## 实现人脸识别 Server
### 准备要识别的图片

下载图片，放到 `src/face_detect_service/resource` 下，命名为 `default.png`

构建可执行文件在 `face-detect-ros-project/install` 下，为了图片能被可执行文件找到，图片也需要复制过去

于是需要配置 `src/face_detect_service/setup.py` 

```python title:src/face_detect_service/setup.py
data_files=[
		···
        ('share/' + package_name + '/resource', ['resource/default.png']),
    ],
```

加入上面这行，意思是会把 `face_detect_service/resource` 下的 `default.png` 复制到 `install/face_detect_service/share` 下

还要配置 setup.py，以便定义的 node 能被找到
```python title:setup.py
entry_points={
        'console_scripts': [
            'face_detect_server = face_detect_service.face_detect_server:main',
        ],
    },
```


### 实现 Server 逻辑

大概的逻辑如下：
1. 从 request 中获取图片数据
2. 先用 `cv_bridge` 库，把 ROS 的 image type msg 变成 opencv 支持的图片
3. 再用 `face_recogition` 以及 `cv` 库对图片进行处理，获得 `face_locations`
4. 整理后返回 response

在代码中有：`for (top, right, bottom, left) in face_locations:`
其中的含义如下：

| 值        | 含义            | 原点方向          |
| -------- | ------------- | ------------- |
| `top`    | 人脸区域上边缘的 Y 坐标 | 从图像**顶部**开始向下 |
| `right`  | 人脸区域右边缘的 X 坐标 | 从图像**左侧**开始向右 |
| `bottom` | 人脸区域下边缘的 Y 坐标 | 从图像**顶部**开始向下 |
| `left`   | 人脸区域左边缘的 X 坐标 | 从图像**左侧**开始向右 |
启动 server 并检验
```shell

face_detect_ros_project [ main][!?][🧚 v0.63.2 default][⏱ 1m17s]
❯ ros2 service call /face_detect interface/srv/FaceDetector                                                        
requester: making request: interface.srv.FaceDetector_Request(image=sensor_msgs.msg.Image(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), height=0, width=0, encoding='', is_bigendian=0, step=0, data=[]))

response:
interface.srv.FaceDetector_Response(number=24, use_time=0.0, top=[588, 588, 593, 93, 77, 1084, 1646, 118, 1084, 1646, 1628, 593, 1108, 1604, 614, 1089, 1628, 118, 1108, 1604, 93, 77, 593, 1130], right=[1356, 860, 1790, 1356, 324, 1356, 1831, 2306, 860, 820, 365, 2780, 2817, 2297, 324, 2326, 1356, 885, 365, 2760, 2792, 1810, 2285, 1810], bottom=[811, 811, 779, 316, 263, 1307, 1832, 304, 1307, 1832, 1851, 779, 1331, 1827, 800, 1274, 1851, 341, 1331, 1790, 316, 263, 779, 1316], left=[1133, 637, 1604, 1133, 139, 1133, 1645, 2120, 637, 634, 142, 2595, 2594, 2074, 139, 2141, 1133, 662, 142, 2574, 2569, 1625, 2099, 1625])

```

## 实现人脸识别 Client

大概逻辑如下：
1.  初始化，读取本地图片为 opencv 的格式
2. 先用 `cv_bridge` 库，把 opencv 支持的图片 变成 ROS 的 image type msg 
3. send_request, 传输 msg
4. 得到 response，然后展现人脸范围

一样，配置 `setup.py` 的 entry_points 和 data_files