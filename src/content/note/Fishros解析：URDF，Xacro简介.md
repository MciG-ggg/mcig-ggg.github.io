---
title: Fishros解析：URDF，Xacro简介
date: 2026-02-13
timestamp: 2026-02-13T12:08:09+08:00
slug: fishrosurdfxacro
category: note
tags:
  - AI
  - EmbodiedAI
  - ROS2
---

# Fishros 解析：URDF，Xacro 简介

## URDF 格式

fishros 里先给了一个简单例子介绍 URDF 格式

```xml
<?xml version="1.0"?>
<robot name="first_robot">
    <!-- 机器人身体部分 -->
    <link name="base_link">
        <!-- 部件外观描述 -->
        <visual>
            <!-- 沿自己几何中心的偏移与旋转量 -->
            <origin xyz="0 0 0" rpy="0 0 0" />
            <!-- 几何形状 -->
            <geometry>
                <!-- 圆柱体，半径0.1m，高度 0.12m -->
                <cylinder length="0.12" radius="0.10" />
            </geometry>
            <!-- 材质子标签-蓝色 -->
            <material name="blue">
                <color rgba="0.1 0.1 1.0 0.5" />
            </material>
        </visual>
    </link>
    <!-- 机器人IMU部件 -->
    <link name="imu_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <geometry>
                <box size="0.02 0.02 0.02" />
            </geometry>
        </visual>
        <material name="black">
            <color rgba="0 0 0 0.5" />
        </material>
    </link> 
    <!-- 机器人关节 -->
    <joint name="imu_joint" type="fixed">
        <!-- 父部件 -->
        <parent link="base_link" />
        <!-- 子部件 -->
        <child link="imu_link" />
        <!-- 子部件相对父部件的平移和旋转 -->
        <origin xyz="0 0 0.03" rpy="0 0 0" />
    </joint>
</robot>
```

### URDF 基本语法
我们可以从中学到最基本的 URDF 格式内容：
- `<robot name=/>`: 定义 robot (一个机器人由若干连杆 (link) 和连接连杆的关节 (joint) 构成)
- `<link name=/>`: 定义连杆
	- `<visual></visual>`: 定义连杆看上去长什么样子
		- `<geometry></geometry>`: 定义连杆的几何形状
		- `<material></material>`: 定义连杆的材质
	- 此外还有 `<collision></collision>` 标签: 定义碰撞箱， 碰撞箱也可以用 `<geometry></geometry>`,  `<material></material>` 这两个标签
- `<joint name= type= >`: 定义关节，其中 **type** 定义关节的类型，一个关节一般是定义为父部件连子部件
	- `<parent link= />`: 定义父部件是哪个 link
	- `<child link= />`: 定义子部件是哪个 link
	- `<origin xyz= rpy= />`: 定义**子部件相对父部件的平移和旋转**

### 常见的关节类型：
1. **Revolute (旋转关节)**：绕轴旋转，有角度限制（如：机械臂的肘部）。
2. **Continuous (连续旋转关节)**：绕轴无限旋转（如：小车的轮子）。
3. **Prismatic (平移关节)**：沿轴线滑动（如：电梯或伸缩杆）。
4. **Fixed (固定关节)**：两个 Link 锁死，不能相对运动（如：安装在底座上的摄像头）。

## Xacro (XML Macros) 格式

在 ROS 2 的开发中，如果说 URDF 是机器人的“蓝图”，那么 **Xacro (XML Macros)** 就是这张蓝图的“编程语言”。

单纯的 URDF 就像写死代码（Hardcoding），而 Xacro 引入了变量、数学运算和宏定义，极大地降低了代码的冗余，是具身智能和复杂机器人建模的标准方式。

---

### 变量与常量 (`property`)

你可以定义一个变量（如轮子半径），然后在整个文件中引用它。

```xml
<xacro:property name="wheel_radius" value="0.1" />
<xacro:property name="wheel_width" value="0.05" />

<geometry>
  <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
</geometry>
```

### 数学运算

Xacro 支持在 `${}` 中直接进行简单的四则运算，这在计算偏移量（Origin）时极其方便。

```xml
<origin xyz="0 0 ${base_height / 2 + wheel_radius}" rpy="0 0 0"/>
```

### 宏定义 (`macro`) —— 最强大的功能

如果你的机器人有 4 个一模一样的轮子，你不需要复制粘贴 4 次 Link 和 Joint，只需要写一个宏：

```xml
<xacro:macro name="wheel" params="prefix suffix x_reflect y_reflect">
  <link name="${prefix}_${suffix}_wheel">
    <visual>
      <geometry><sphere radius="${wheel_radius}"/></geometry>
    </visual>
  </link>
  </xacro:macro>

<xacro:wheel prefix="front" suffix="left" x_reflect="1" y_reflect="1" />
<xacro:wheel prefix="front" suffix="right" x_reflect="1" y_reflect="-1" />
```

### 支持条件判断
```xml
<xacro:if value="${use_sim_gpu}">
</xacro:if>
```

---

### 如何从 Xacro 变成 URDF 格式？

ROS 2 不能直接“读” Xacro，必须先将其解析为标准的 URDF 文本。

#### 方式 A：命令行手动转换

```shell
xacro robot.urdf.xacro > robot.urdf
```

#### 方式 B：在 Python Launch 脚本中动态转换（推荐）

在 Launch 脚本中，通常这样加载 Xacro：

```python 
import xacro
from launch_ros.actions import Node

def generate_launch_description():
    # 路径处理
    xacro_file = 'path/to/robot.urdf.xacro'
    # 解析 xacro
    robot_description_config = xacro.process_file(xacro_file).toxml()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        )
    ])
```