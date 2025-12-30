---
title: Modern Robotics笔记 Chapter2-3
timestamp: 2025-11-21 12:58:28+08:00
tags:
  - conclusion/robotics
  - robotics
---

# Modern Robotics笔记 Chapter2-3

---

## Chapter2

### 机器人关节自由度与缩写

| 英文名称        | 中文翻译 | 常用缩写       | 自由度 | 运动类型  |
| ----------- | ---- | ---------- | --- | ----- |
| Revolute    | 旋转关节 | R, Rev     | 1   | 旋转    |
| Prismatic   | 移动关节 | P, Prism   | 1   | 平移    |
| Cylindrical | 圆柱关节 | C, Cyl     | 2   | 旋转+平移 |
| Spherical   | 球形关节 | S, Sph     | 3   | 三轴旋转  |
| Universal   | 万向节  | U, Univ    | 2   | 二轴旋转  |
| Planar      | 平面关节 | Pl, Planar | 3   | 平面运动  |
| Helical     | 螺旋关节 | H, Hel     | 1   | 螺旋运动  |
| Screw       | 螺旋关节 | Scr        | 1   | 螺旋运动  |

---

## Chapter3

### 3.2 刚体旋转

#### 3.2.1 旋转矩阵

$$
R^TR = I
$$

进一步，在右手系下有:

$$
detR = 1
$$

3\*3旋转矩阵的集合称为 *特殊正交群* $SO(3)$ （李群）
同理，2\*2 为 $SO(2)$

##### 正交群的性质

- 封闭： $AB\in SO(3)$ （关于“乘法”封闭）
- 结合律
- 存在单位元： $\exists I, AI = IA = A$ (联想，乘法里的 1，单位元，群的特性)
- 可逆性： $\exists A^-1, A A^{-1} = A^{-1}A = I$

#### 3.2.2 角速度

一个固定参考 $s$ 中的刚体转动，可以考察每时每刻和刚体重合的静止参考系 $b$ , 此时旋转矩阵 $R(t)$ 可以视作刚体每一时刻 $t$ 的位姿。
那可以相应的定义旋转矩阵的微分 $\dot{R}$ ，就是 $R$ 各个参数对时间的导数
（积分即为一段时间内刚体位姿变化）

下面考察角速度和 $\dot{R}$ 的关系

![](./images/)

由图可得， $\dot{r_{i}} = \omega_{s} \times r_{i}, i=1,2,3$ ,其中i 取 1，2,3 的时候分别对应 x, y, z 轴
整合成矩阵形式有：

$$
\dot{R} = [\omega_{s} \times r_{1}, \omega_{s} \times r_{2}, \omega_{s} \times r_{3},]
$$

上述方程可写为：

$$
\dot{R} = [ \boldsymbol{\omega} ]_{\times} \cdot R 
$$

其中 $[\omega]_{\times}$ 属于so(3)（李代数）(而不是 $\dot{R} \in so(3)$ )

> [!NOTE] 李群和李代数
> 粗略理解（？）： 李群是表示绕某一个轴旋转一个角度，而李代数是绕一个轴旋转一个无限小角度

为将上式右边的叉积消去，我们引入了一种新的运算，即将 $\boldsymbol{\omega}\times R$ 重新写成 $[\boldsymbol{\omega}] R$ 的形式，其中 $[\boldsymbol{\omega}]$ 为向量 $\boldsymbol{\omega}\in\mathbb{R}^3$ 对应的 $3\times 3$ 反对称（skew-symmetric）矩阵。

**定义 3.7**
给定一向量 $\mathbf{x}=[x_1,x_2,x_3]^{\sf T}\in\mathbb{R}^3$ ，定义

$$
[\mathbf{x}]_\times =
\begin{bmatrix}
0 & -x_3 & x_2 \\
x_3 & 0 & -x_1 \\
-x_2 & x_1 & 0
\end{bmatrix}
$$

矩阵 $[\mathbf{x}]_\times$ 就是与向量 $\mathbf{x}$ 对应的 $3\times 3$ 反对称矩阵，因此满足

$$
[\mathbf{x}]_\times^{\sf T} = -[\mathbf{x}]_\times
$$

旋转矩阵与反对称矩阵之间具有一个非常有用的特性，如下所述。

**命题 3.8**
给定任意 $\boldsymbol{\omega}\in\mathbb{R}^3$ 和 $R\in SO(3)$ ，总能满足

$$
R\,[\boldsymbol{\omega}]_\times R^{\sf T} = [R\boldsymbol{\omega}]_\times
\tag{3.31}
$$

**证明**：令 $\mathbf{r}_i$ 表示 $R$ 的第 $i$ 列，因此我们有

$$
\begin{aligned}
R\,[\boldsymbol{\omega}]_\times R^{\sf T}
&=
\begin{bmatrix}
\mathbf{r}_1^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_1) &
\mathbf{r}_1^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_2) &
\mathbf{r}_1^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_3) \\[2mm]
\mathbf{r}_2^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_1) &
\mathbf{r}_2^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_2) &
\mathbf{r}_2^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_3) \\[2mm]
\mathbf{r}_3^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_1) &
\mathbf{r}_3^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_2) &
\mathbf{r}_3^{\sf T}(\boldsymbol{\omega}\times\mathbf{r}_3)
\end{bmatrix} \\[4mm]
&=
\begin{bmatrix}
0 & -\mathbf{r}_3^{\sf T}\boldsymbol{\omega} & \mathbf{r}_2^{\sf T}\boldsymbol{\omega} \\[2mm]
\mathbf{r}_3^{\sf T}\boldsymbol{\omega} & 0 & -\mathbf{r}_1^{\sf T}\boldsymbol{\omega} \\[2mm]
-\mathbf{r}_2^{\sf T}\boldsymbol{\omega} & \mathbf{r}_1^{\sf T}\boldsymbol{\omega} & 0
\end{bmatrix} \\[4mm]
&= [R\boldsymbol{\omega}]_\times
\end{aligned}
$$

#### 3.2.3 转动的指数坐标表示

$$
\dot p = [\boldsymbol{\omega}]_\times p
\tag{3.50}
$$

初始条件为 $p(0)$。该方程满足前面讨论过的线性微分方程 $\dot x = A x$ 的形式，因此其解可写成

$$
p(t) = e^{t[\boldsymbol{\omega}]_\times} p(0)
$$

由于 $t$ 和 $\theta$ 可以互换，上面的方程又可写成

$$
p(\theta) = e^{\theta[\hat{\boldsymbol{\omega}}]_\times} p(0)
$$

下面对方程中的矩阵指数 $e^{\theta[\hat{\boldsymbol{\omega}}]_\times}$ 进行级数展开。
注意到 $[\hat{\boldsymbol{\omega}}]_\times^3 = -[\hat{\boldsymbol{\omega}}]_\times$，因此级数展开式中的 $[\hat{\boldsymbol{\omega}}]_\times^3$ 用 $-[\hat{\boldsymbol{\omega}}]_\times$ 代替，$[\hat{\boldsymbol{\omega}}]_\times^4$ 用 $-[\hat{\boldsymbol{\omega}}]_\times^2$ 代替，$[\hat{\boldsymbol{\omega}}]_\times^5$ 用 $[\hat{\boldsymbol{\omega}}]_\times$ 代替，诸如此类，由此得到

$$
e^{\theta[\hat{\boldsymbol{\omega}}]_\times}
= I + \theta[\hat{\boldsymbol{\omega}}]_\times
+ \frac{\theta^2}{2!}[\hat{\boldsymbol{\omega}}]_\times^2
+ \frac{\theta^3}{3!}[\hat{\boldsymbol{\omega}}]_\times^3
+ \cdots
$$

再来回顾一下 $\sin\theta$ 和 $\cos\theta$ 的级数展开形式：

$$
\sin\theta = \theta - \frac{\theta^3}{3!} + \frac{\theta^5}{5!} - \cdots,\qquad
\cos\theta = 1 - \frac{\theta^2}{2!} + \frac{\theta^4}{4!} - \cdots
$$

因此，指数 $e^{\theta[\hat{\boldsymbol{\omega}}]_\times}$ 可简化为如下形式。

**命题 3.11**
给定向量 $\boldsymbol{\omega} = \theta\hat{\boldsymbol{\omega}}\in\mathbb{R}^3$，其中 $\theta$ 为任一标量，而 $\hat{\boldsymbol{\omega}}\in\mathbb{R}^3$ 为一单位向量，$[\hat{\boldsymbol{\omega}}]_\times\in\mathfrak{so}(3)$ 的矩阵指数为

$$
\mathrm{Rot}(\hat{\boldsymbol{\omega}},\theta)
= e^{\theta[\hat{\boldsymbol{\omega}}]_\times}
= I + \sin\theta[\hat{\boldsymbol{\omega}}]_\times
+ (1-\cos\theta)[\hat{\boldsymbol{\omega}}]_\times^2
\;\in\; SO(3)
$$

式 (3.51) 通常也被称为 **罗德里格斯公式**（Rodrigues formula）。

> [!NOTE]
> 以上给出了当给定特定转轴 $\hat{\omega}$ 和转角 $\theta$ 的时候，如何通过矩阵指数构建旋转矩阵
不是简单地把 $\mathrm{Rot}(\hat{\boldsymbol{\omega}},\theta)$ 等同于“那个旋转矩阵 $R$ ”，而是：

1. $\mathrm{Rot}(\hat{\boldsymbol{\omega}},\theta)$ **本身就是**一个旋转矩阵
   它把“绕单位轴 $\hat{\boldsymbol{\omega}}$ 转 $\theta$”这件事映射到 $SO(3)$ 里的一个具体元素：

$$
\mathrm{Rot}(\hat{\boldsymbol{\omega}},\theta)=e^{\theta[\hat{\boldsymbol{\omega}}]_\times}\in SO(3)
$$

2. 但“旋转矩阵 $R$ ”可以是**任意** $R\in SO(3)$ ，不限于单轴单角。
   只有当 $R$ 恰好是“绕 $\hat{\boldsymbol{\omega}}$ 转 $\theta$”时，才有

$$
R=\mathrm{Rot}(\hat{\boldsymbol{\omega}},\theta)
$$

#### 3.2.4 对数映射

任意 $R$ 也能找到对应的一对 $(\hat{\boldsymbol{\omega}},\theta)$——这就是**对数映射**：

$$
\theta=\arccos\!\bigl(\tfrac{\mathrm{tr}(R)-1}{2}\bigr),\quad
[\hat{\boldsymbol{\omega}}]_\times=\frac{1}{2\sin\theta}(R-R^{\sf T})
$$

注意: 当 $sin(\theta) = 0$ 时，有

$$
R = I + 2[\hat{\omega}]^2_x
$$

可得:

$$
[\hat{\omega}]^2_x = \frac{R-I}{2}
$$

解得:

$$
\omega_x = \sqrt{\frac{r_{11}  + 1}{2}} 
$$

$$
\omega_y = \sqrt{\frac{r_{22} + 1}{2}} 
$$

$$
\omega_z = \sqrt{\frac{r_{33} + 1}{2}}
$$

其中符号需要通过非对角线元素确定:
- 若 $r_{12} > 0$ , 则 $\omega_x$ 与 $\omega_y$ 符号相反
- 若 $r_{13} > 0$ , 则 $\omega_x$ 与 $\omega_z$ 符号相反
- 若 $r_{23} > 0$ , 则 $\omega_y$ 与 $\omega_z$ 符号相反

$\mathrm{Rot}$ 可看成“参数化”或“生成”旋转矩阵的一种手段，而不是旋转矩阵的唯一名字。

一句话：
$\mathrm{Rot}(\hat{\boldsymbol{\omega}},\theta)$ **是**旋转矩阵，但旋转矩阵**不全是**$\mathrm{Rot}(\hat{\boldsymbol{\omega}},\theta)$——它只是“单轴旋转”那一子集。

##### 对数映射的 $\theta$ 还有一种求法

Frobenius范数定义为矩阵所有元素平方和的平方根。对于斜对称矩阵 $[\mathbf{\omega}]$ ，其Frobenius范数计算如下：

$$
 \| [\mathbf{\omega}] \|_F = \sqrt{2 (\omega_x^2 + \omega_y^2 + \omega_z^2)} 
$$

由于 $\mathbf{u}$ 是单位向量，所以 $u_x^2 + u_y^2 + u_z^2 = 1$ ，从而：

$$
 \| [\mathbf{u}] \|_F = \sqrt{2} 
$$

### 旋转角度的计算

旋转角度 $\theta$ 与斜对称矩阵的关系为：

$$
 \log(R) = \theta [\mathbf{\omega}] 
$$

因此, $\log(R)$ 的Frobenius范数为：

$$
 \| \log(R) \|_F = \| \theta [\mathbf{\omega}] \|_F = \theta \| [\mathbf{\omega}] \|_F = \sqrt{2} \theta 
$$

在Python中，可以使用以下代码计算Frobenius范数：

```python
import numpy as np
import math

def frobenius_norm(matrix):
    return np.sqrt(np.sum(np.square(matrix)))

# 示例
u = np.array([0, 0, 1])  # 旋转轴
theta = 2  # 旋转角度
log_R = theta * np.array([
    [0, -u[2], u[1]],
    [u[2], 0, -u[0]],
    [-u[1], u[0], 0]
])

print("旋转角度 theta:", frobenius_norm(log_R))

u = np.array([0, 0, 1])  # 旋转轴
theta = 1  # 旋转角度
log_R = theta * np.array([
    [0, -u[2], u[1]],
    [u[2], 0, -u[0]],
    [-u[1], u[0], 0]
])

print("旋转角度 theta:", frobenius_norm(log_R) / math.sqrt(2))
```

#### 3.2.4 转动的矩阵对数

在 Obsidian 里可直粘的「SO(3) ↔ 指数坐标」速查卡

- 算法：SO(3) → 指数坐标
输入 $R\in SO(3)$，输出 $(\hat{\boldsymbol{\omega}},\theta)$ 使 $R=e^{\theta[\hat{\boldsymbol{\omega}}]_\times}$

1. **平凡情况**
   $R=I \Rightarrow \theta=0,\ \hat{\boldsymbol{\omega}}$ **不确定**（任意单位向量）。

2. **转 π 情况**（$\mathrm{tr}\,R=-1$）
   $\theta=\pi$，$\hat{\boldsymbol{\omega}}$ 可在下列 **三选一** 里任取：

$$
   \hat{\boldsymbol{\omega}}=\dfrac{1}{\sqrt{2(1+r_{33})}}
   \begin{bmatrix}r_{13}\\r_{23}\\1+r_{33}\end{bmatrix}
   \quad\text{或}\quad
   \dfrac{1}{\sqrt{2(1+r_{22})}}
   \begin{bmatrix}r_{12}\\1+r_{22}\\r_{32}\end{bmatrix}
   \quad\text{或}\quad
   \dfrac{1}{\sqrt{2(1+r_{11})}}
   \begin{bmatrix}1+r_{11}\\r_{21}\\r_{31}\end{bmatrix}
$$

   （分母为零就换下一组）

3. **通用情况**（$\mathrm{tr}\,R\ne -1$）

$$
   \theta=\arccos\!\Bigl(\frac{\mathrm{tr}\,R-1}{2}\Bigr)\in(0,\pi),\quad
   [\hat{\boldsymbol{\omega}}]_\times=\frac{R-R^{\sf T}}{2\sin\theta}
$$

- 几何图景：半径为 π 的实心球（ $\mid\mid \hat{\omega} \theta\mid\mid \leq \pi$ ）
	- 球面上两点 $\mathbf{r}$ 与 $-\mathbf{r}$（$\theta=\pi$）**映射到同一** $R$（$\mathrm{tr}\,R=-1$）。
	- 球内一点 $\mathbf{r}$ 对应唯一旋转：
	  $\hat{\boldsymbol{\omega}}=\dfrac{\mathbf{r}}{|\mathbf{r}|},\ \theta=|\mathbf{r}|,\ R=e^{[\mathbf{r}]_\times}$
	- 球心 $\mathbf{r}=0$ 对应恒等旋转 $I$。

> [!NOTE]
> - **球内点**：对于 $\theta < \pi$ ，每个点 $(\boldsymbol{\omega}, \theta)$ 对应一个唯一的旋转矩阵，因为旋转角度小于 $\pi$ 时，旋转是唯一的。
> - **球面上点**：对于 $\theta = \pi$ ，球面上的点 $(\boldsymbol{\omega}, \pi)$ 和 $(-\boldsymbol{\omega}, \pi)$ 映射到同一个旋转矩阵，这解释了为什么 $\mathrm{tr}(R) = -1$ 的旋转矩阵有两个对应的指数坐标。

### 3.3 刚体运动与运动旋量

#### 3.3.1 齐次变换矩阵

旋转矩阵 $R$ 表示 $\{b\}$ 相对于 $\{s\}$ 姿态，和向量 $p$ 表示 $\{b\}$ 相对于 $\{s\}$ 的坐标，把他们两个合在一起有:
**特殊欧式群** $SE(3)$ ，又叫*刚体运动群*，*齐次变换群*

$$
T =
\begin{bmatrix}
R & p \\
0 & 1 \\
\end{bmatrix}
=
\begin{bmatrix}
r_{11} & r_{12} & r_{13} &p_{1} \\
r_{21} & r_{22} & r_{23} &p_{2} \\
r_{31} & r_{32} & r_{33} &p_{3} \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$

也可以写成

$$
T = (R,p)
$$

##### SE(3)的性质

- 齐次变换矩阵 $T$ 的逆矩阵也是齐次变换矩阵
- 两个齐次变换矩阵的乘积还是齐次变换矩阵
- 结合律（不一定有交换律）
- 变换的等距性： $\left\| Tx-Ty \right\| = \left\| x-y \right\|$
- 变换的保角性：对于所有的 $z \in \mathbb{R}^3$ ，都满足 $\langle Tx - Tz, Ty - Tz \rangle = \langle x - z, y - z \rangle$ 。式中 $\langle \cdot, \cdot \rangle$ 表示向量在 $\mathbb{R}^3$ 中的标准内积， $\langle x, y \rangle = x^\mathrm{T} y$ 。

#### 3.3.2 运动旋量

物体坐标系 $\{b\}$ 相对于静止参考系 $\{s\}$ 的位形的导数

$$
\dot{T} =
\begin{bmatrix}
\dot{R} & \dot{p} \\
0 & 0 \\
\end{bmatrix}
$$

> [!NOTE] 物体系
> 物体系就是物理中的自然坐标系

考察对 $\dot{T}$ 左乘 $T^{-1}$ 的结果

$$
T^{-1}\dot T =
\begin{bmatrix}
R^{\sf T} & -R^{\sf T}\mathbf p\\[2mm]
\mathbf 0 & 1
\end{bmatrix}
\begin{bmatrix}
\dot R & \dot{\mathbf p}\\[2mm]
\mathbf 0 & 0
\end{bmatrix}
=
\begin{bmatrix}
R^{\sf T}\dot R & R^{\sf T}\dot{\mathbf p}\\[2mm]
\mathbf 0 & 0
\end{bmatrix}
$$

- 左上角 $R^{\sf T}\dot R=[\boldsymbol{\omega}_b]_\times$
  → 物体**自身角速度** $\boldsymbol{\omega}_b$，但写成**物体系分量**。
- 右上角 $R^{\sf T}\dot{\mathbf p}=\mathbf v_b$
  → 物体**原点线速度**，同样写成**物体系分量**。

$T^{-1}\dot T$ 把“静系看到的运动”扳回“物体自己看到的运动”：

> 动坐标系 $\{b\}$ 相对于**瞬时与自己重合的静系**的角速度与线速度，
> 并以**物体系坐标**写成 $4\times4$ Twist 矩阵：

$$
> T^{-1}\dot T = \begin{bmatrix} [\boldsymbol{\omega}_b]_\times & \mathbf v_b\\[2mm] \mathbf 0 & 0 \end{bmatrix} \in\mathfrak{se}(3)
>
> 
$$

因此它正是**物体坐标系下的空间速度**（body-twist）。

可以定义物体坐标系中的速度，简称为物体运动旋量

- **六维向量形式**
   把角速度 $\boldsymbol{\omega}_b$ 与线速度 $\mathbf{v}_b$ 合在一个向量里：

$$
   \mathcal{V}_b = \begin{bmatrix}\boldsymbol{\omega}_b \\ \mathbf{v}_b\end{bmatrix}\in\mathbb{R}^6
   \tag{3.70}
$$

- 4\*4 矩阵形式

$$
   T^{-1}\dot T = [\mathcal{V}_b] =
   \begin{bmatrix}
   [\boldsymbol{\omega}_b]_\times & \mathbf{v}_b \\
   \mathbf{0} & 0
   \end{bmatrix}
   \in\mathfrak{se}(3)
   \tag{3.71}
$$

> 所有满足该结构的 $4\times4$ 矩阵集合就是 $\mathfrak{se}(3)$，它与刚体位形 $SE(3)$ 一一对应，称为**矩阵形式的运动旋量**。

下面考察 $\dot{T}T^{-1}$
- **数学表达**

$$
   \dot T T^{-1} =
   \begin{pmatrix}
   \dot R & \dot p \\
   0 & 0
   \end{pmatrix}
   \begin{pmatrix}
   R^{\sf T} & -R^{\sf T} p \\
   0 & 1
   \end{pmatrix}
   =
   \begin{pmatrix}
   \dot R R^{\sf T} & \dot p - \dot R R^{\sf T} p \\
   0 & 0
   \end{pmatrix}
   \tag{3.72}
$$

   进一步简化为：

$$
   = \begin{bmatrix}
   [\boldsymbol{\omega}_s] & \mathbf{v}_s \\
   0 & 0
   \end{bmatrix}
$$

- **线速度的解释**
   - 线速度 $\mathbf{v}_b$ 的物理意义：假设刚体的尺寸足够大，$\mathbf{v}_b$ 可看作是刚体上与固定坐标系原点相重合的点的瞬时速度，并在固定坐标系中度量。
   - 数学表达：

$$
     \mathbf{v}_s = \dot p - \boldsymbol{\omega}_s \times p = \dot p + \boldsymbol{\omega}_s \times (-p)
     \tag{3.73}
     
$$

- **六维向量形式**
   将 $\boldsymbol{\omega}_b$ 和 $\mathbf{v}_b$ 合成为一个六维向量形式：

$$
   \mathcal{V}_b = \begin{bmatrix} \boldsymbol{\omega}_b \\ \mathbf{v}_b \end{bmatrix} \in \mathbb{R}^6
   \tag{3.74}
$$

- **矩阵形式**
   扩展定义，写成 $4 \times 4$ 矩阵形式：

$$
   [\mathcal{V}_s] = \begin{bmatrix}
   [\boldsymbol{\omega}_s] & \mathbf{v}_s \\
   0 & 0
   \end{bmatrix} = \dot T T^{-1} \in \mathfrak{se}(3)
$$

如果我们将移动刚体想象成无穷大，那么在 $\mathcal{V}_s \equiv (\boldsymbol{\omega}_s, \mathbf{v}_s)$ 与 $\mathcal{V}_b \equiv (\boldsymbol{\omega}_b, \mathbf{v}_b)$ 之间存在着非常有吸引力和自然的对称。

1. $\boldsymbol{\omega}_b$ 是表示在 $\{b\}$ 中的角速度，而 $\boldsymbol{\omega}_s$ 是表示在 $\{s\}$ 中的角速度。
2. $\mathbf{v}_b$ 是表示在 $\{b\}$ 中与 $\{b\}$ 原点重合点的线速度，而 $\mathbf{v}_s$ 是表示在 $\{s\}$ 中与 $\{s\}$ 系原点重合点的线速度。

##### 运动旋量转换关系

1. **计算 $\mathcal{V}_b$ 和 $\mathcal{V}_s$：**

$$
   [\mathcal{V}_b] = T^{-1} \dot T = T^{-1} [\mathcal{V}_s] T
$$

   反过来：

$$
   [\mathcal{V}_s] = T [\mathcal{V}_b] T^{-1}
$$

2. **展开式 (3.76)：**

$$
   [\mathcal{V}_s] =
   \begin{bmatrix}
   R(\omega_b)R^{\sf T} & -R(\omega_b)R^{\sf T} p + R\mathbf{v}_b \\
   0 & 0
   \end{bmatrix}
$$

3. **利用 $R[\omega]R^{\sf T} = [\mathcal{R}\omega]$（命题 3.8）和 $[\omega]p = -[p]\omega$（$p, \omega \in \mathbb{R}^3$），简化得到 $\mathcal{V}_b$ 与 $\mathcal{V}_s$ 之间的关系式：**

$$
   \mathcal{S_i}
   =
   \begin{bmatrix}
   R & 0 \\
   [p]R & R
   \end{bmatrix}
   \mathcal{B_i}
$$

4. **伴随变换矩阵（Adjoint Representation）[定义 3.20]：**
   给定 $T = (R, p) \in SE(3)$，其伴随变换矩阵 $[Ad_T]$ 为：

$$
   [Ad_T] =
   \begin{bmatrix}
   R & 0 \\
   [p]R & R
   \end{bmatrix}
   \in \mathbb{R}^{6 \times 6}
$$

> $[Ad_{T}]$ 是6\*6矩阵，所以和 $\mathcal{V_{s}}$ (或者 $\mathcal{S}$ )(6维向量)相乘
> $T$ 是 4\*4 矩阵，所以和 $[\mathcal{V}]$ (4\*4矩阵)相乘

##### 运动旋量的螺旋释义

![](./images/)

如图，刚体运动可以分解为：*绕某固定轴的一次旋转 + 沿该轴的一次平移*

将运动旋量 $\mathcal{V}=(\omega, v)$ 与螺旋运动的各个参数 $\{q, \hat{s}, h\}$ 对应起来，有：

$$
\mathcal{V} =
\begin{bmatrix}
\omega \\
v \\
\end{bmatrix}
=
\begin{bmatrix}
\hat{s}\dot{\theta}\\
-\hat{s}\dot{\theta}\times q+h\dot{\theta}\hat{s} \\
\end{bmatrix}
$$

> $q$ : 为轴上任意一点
> $\hat{s}$ : 为表示螺旋轴方向的单位向量
> $h$ : 为螺旋的节距， 数值上为沿轴方向线速度/绕轴角速度
> $\dot{\theta}$ : 绕螺旋轴旋转角速度

注意到，线速度 $v$ 为两项之和： $-\hat{s}\dot{\theta} \times q$ 为转动带来的原点位置变化，而 $h\dot{\theta}\hat{s}$ 为沿轴线速度
- 若 $\omega = 0$，则 $h$ 无限大。这种情况下，$\hat{s} = v / \|v\|$，$\theta = \|v\|$。
- ==由于 $h$ 可能无穷大， $q$ 也不具备唯一性（ $q$ 可在螺旋轴上任取），因此我们决定不采用 $(q, \hat{s}, h)$ 来描述螺旋运动，而采用运动旋量正交化的形式来描述。==

所以，考察从 $\mathcal{V}$ 到**螺旋视角**的变换 ：
1. 若 $\omega \ne 0 , S = \mathcal{V} / \left\|\mathbb{\omega}\right\| = (\omega/\left\|\omega \right\|, v/ \left\|\omega \right\|)$ , 螺旋轴 $\mathcal{S}$ 只需要简单的正则化 $\mathcal{S}$ , 角速度 $\dot{\theta} = \left\|\omega \right\|, \mathcal{S}\dot{\theta} = \mathcal{V}$
2. 若 $\omega = 0, \mathcal{S} = \mathcal{V}/\left\|v \right\| = (0, v/\left\|v \right\|)$ , 线速度 $\dot{\theta} = \left\|v \right\|, \mathcal{S}\dot{\theta} = \mathcal{V}$
3. **螺旋轴的矩阵表示**
   给定参考坐标系，螺旋轴 $S$ 可写成：

$$
   S = \begin{bmatrix} \omega \\ v \end{bmatrix} \in \mathbb{R}^6
$$

   其中：
   - $\|\omega|| = 1$
   - $\omega = 0, \|v\| = 1$。若满足，则 $v = -\omega \times q + h\omega$，式中 $q$ 为轴上任一点，$h$ 为螺旋的节距（若为纯转动，则 $h=0$）；若满足，则螺旋的节距无穷大，对应的运动旋量为纯移动，参数中只有 $v$。

5. **定义 3.24**
   给定参考坐标系，螺旋轴 $S$ 可写成：

$$
   S = \begin{bmatrix} \omega \\ v \end{bmatrix} \in \mathbb{R}^6
$$

   式中，$[\omega]$ 的下面一行全为 0。而且两个不同坐标系 $\{a\}$ 与 $\{b\}$ 中所描述的螺旋轴之间的映射关系可用下式来表达：

$$
   S_a = [\mathrm{Ad}_{T_{ab}}] S_b, \quad S_b = [\mathrm{Ad}_{T_{ba}}] S_a
$$

其中：

$$
Ad_{Tab}= \begin{bmatrix}
R & 0 \\
p \times R & 1 \\
\end{bmatrix}
$$

$$
Ad_{Tab}[ξ] = \begin{bmatrix}
R_{ab} \omega \\
R_{ba} - p_{ba} \times R_{ba} \omega
\end{bmatrix}
$$

$$
Ad_{Tba}[ξ] = \begin{bmatrix}
R_{ba} \omega \\
R_{ab} + p_{ab} \times R_{ab} \omega
\end{bmatrix}
$$

> $Ad_{Tab}$ ​​ 是一个抽象的操作，不直接以数值形式存在。
 $[Ad_{Tab}​​]$ 是一个具体的数值矩阵，可以直接用于矩阵乘法等数值计算。

#### 3.3.3 刚体运动的指数坐标表达

##### 矩阵指数

【命题 3.25】令 $S = (\omega, v)$ 为螺旋轴，

- 若 $\|\omega\| = 1$ ，则对于任意沿螺旋轴的距离 $\theta \in \mathbb{R}$ ，都有

$$
e^{\theta [S]} = 
\begin{bmatrix}
R & p \\
\mathbf{0} & 1
\end{bmatrix},
\quad 
\begin{cases}
R = I + \sin\theta\,[\omega]_\times + (1-\cos\theta)\,[\omega]_\times^{2} \\[6pt]
p = \theta v + (1-\cos\theta)\,\omega\times v + (\theta-\sin\theta)\,\omega\times(\omega\times v)
\end{cases}
$$

- 若 $\omega = 0, \|v\| = 1$ ，则

$$
e^{\theta [S]} = 
\begin{bmatrix}
I_{3} & \theta v \\
\mathbf{0} & 1
\end{bmatrix}
$$

上述推导过程实质上给出了对 **Chasles-Mozzi 定理** 的证明过程，即给定任意的 $(R, p) \in SE(3)$，总能找到与之相对应的螺旋轴 $S = (\omega, v)$ 和标量 $\theta$，满足

$$
\begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} = \exp\left( \theta \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix} \right)
$$

式中，矩阵

$$
\theta \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix} = \theta [S] \in \mathfrak{se}(3)
$$

是 $T = (R, p)$ 的矩阵对数形式。

##### 矩阵对数

1. $R \ne I$(含旋转运动)

$$
\begin{cases}
\displaystyle \theta = \arccos\left(\frac{\operatorname{tr}(R)-1}{2}\right) \in (0,\pi] \\[10pt]
\displaystyle [\omega]_\times = \frac{1}{2\sin\theta}(R - R^\top) \\[10pt]
\displaystyle v = G(\theta)^{-1}p \\[10pt]
\displaystyle G(\theta) = \theta I_3 + (1-\cos\theta)[\omega]_\times + (\theta-\sin\theta)[\omega]_\times^2
\end{cases}
$$

2. $R = I$(纯平移运动)

$$
\begin{cases}
\displaystyle \theta = \|p\| \\[8pt]
\displaystyle \omega = \mathbf{0} \\[8pt]
\displaystyle v = \frac{p}{\|p\|}
\end{cases}
$$

### 3.4 力旋量

把力和力矩合起来:

$$
\mathcal{F_a} =
\begin{bmatrix}
m_a \\
f_a \\
\end{bmatrix}
\in \mathbb{R^6}
$$

根据不同系下功率不变，可以列出：

$$
\mathcal{V^T_b F_b = V^T_a F_a}
$$

由：

$$
\mathcal{V_a} = [Ad_{T_{ab}}] \mathcal{V_b}
$$

得：

$$
\mathcal{F_b} = [Ad_{Tab}]^T \mathcal{F_a}
$$

同理：

$$
\mathcal{F_a} = [Ad_{Tba}]^T \mathcal{F_b}
$$

---

## 第三章总结：刚体转动vs一般刚体运动

| 概念类别      | 刚体转动                                                                                                                               | 一般刚体运动                                                                                         |
| --------- | ---------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| **数学结构**  | **特殊正交群** $SO(3)$                                                                                                                  | **特殊欧式群** $SE(3)$                                                                              |
| **矩阵表示**  | 旋转矩阵 $R \in \mathbb{R}^{3\times3}$                                                                                                 | 齐次变换矩阵 $T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in \mathbb{R}^{4\times4}$            |
| **基本性质**  | $R^TR = I$, $\det R = 1$                                                                                                           | $T^{-1} = \begin{bmatrix} R^T & -R^T p \\ 0 & 1 \end{bmatrix}$                                 |
| **群性质**   | 封闭性、结合律、单位元、可逆性                                                                                                                    | 封闭性、结合律、单位元、可逆性                                                                                |
| **李代数**   | $\mathfrak{so}(3) = \{[\omega]_\times : \omega \in \mathbb{R}^3\}$                                                                 | $\mathfrak{se}(3) = \left\{\begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix}\right\}$ |
| **速度描述**  | 角速度 $\omega \in \mathbb{R}^3$                                                                                                      | 运动旋量 $\mathcal{V} = \begin{bmatrix} \omega \\ v \end{bmatrix} \in \mathbb{R}^6$                |
| **运动学方程** | $\dot{R} = [\omega]_\times R$                                                                                                      | $\dot{T} = [\mathcal{V}] T$ 或 $T^{-1}\dot{T} = [\mathcal{V}_b]$                                |
| **指数坐标**  | $R = e^{\theta[\hat{\omega}]_\times}$ (罗德里格斯公式)                                                                                    | $T = e^{\theta[S]}$, $S = \begin{bmatrix} \omega \\ v \end{bmatrix}$                           |
| **几何表示**  | 绕固定轴的旋转                                                                                                                            | 绕螺旋轴的旋转+沿轴平移                                                                                   |
| **坐标变换**  | $R_{ab}R_{bc} = R_{ac}$                                                                                                            | $T_{ab}T_{bc} = T_{ac}$                                                                        |
| **微分关系**  | $R^T\dot{R} = [\omega_b]_\times$                                                                                                   | $T^{-1}\dot{T} = \begin{bmatrix} [\omega_b]_\times & v_b \\ 0 & 0 \end{bmatrix}$               |
| **反对称矩阵** | $[\omega]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}$ | $[\mathcal{V}] = \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix}$                   |
| **自由度**   | 3个旋转自由度                                                                                                                            | 6个自由度(3旋转+3平移)                                                                                 |
| **对数映射**  | 从$R$求$(\hat{\omega},\theta)$                                                                                                       | 从$T$求$(S,\theta)$                                                                              |
| **参数化**   | 轴角表示、欧拉角、四元数                                                                                                                       | 螺旋轴表示、指数坐标                                                                                     |

[Modern Robotics笔记-Chapter4](./Modern Robotics笔记-Chapter4/)
