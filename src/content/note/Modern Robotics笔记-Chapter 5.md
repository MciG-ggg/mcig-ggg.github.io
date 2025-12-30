---
title: Modern Robotics笔记-Chapter 5
timestamp: 2025-11-14 17:04:48+08:00
tags:
  - conclusion/robotics
  - robotics
---

# Modern Robotics笔记-Chapter 5

空间雅可比与乘积指数公式是机器人运动学中两个密切相关但**功能不同**的核心概念。以下是它们的区别与联系：

---

## 1. 乘积指数公式（Product of Exponentials, PoE）

### 1.1 功能

**计算末端执行器的位姿**（位置和姿态），即**正向运动学（FK）**。

### 1.2 数学形式（空间形式）

末端的齐次变换矩阵为：

$$
\textcolor{blue}{T(\theta)} = e^{[\mathcal{S}_1]\theta_1} e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} M
$$

- **输出**：

$$
4\times4
$$

 的

$$

SE(3)

$$

 矩阵
- **物理意义**：从关节变量

$$

\theta \in \mathbb{R}^n

$$

 到**位形空间**的映射

---

## 2. 空间雅可比矩阵（Spatial Jacobian）

### 2.1 功能

**将关节速度映射为末端的空间速度**，用于速度分析和力传递。

### 2.2 数学定义

末端空间速度 $\mathcal{V}_s$ 与关节速度$\dot{\theta}$的关系：

$$

\mathcal{V}_s = \begin{bmatrix} \omega_s \\ v_s \end{bmatrix} = \textcolor{red}{J_s(\theta)} \dot{\theta}

$$

其中

$$

J_s(\theta) \in \mathbb{R}^{6\times n}

$$

 为空间雅可比。

### 2.3 构造方式

空间雅可比的第$i$列为：

$$

\textcolor{red}{J_{s,i}(\theta)} = \text{Ad}_{e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_{i-1}]\theta_{i-1}}}(\mathcal{S}_i)

$$

即：将第$i$个关节的空间螺旋轴$\mathcal{S}_i$，通过**伴随变换**转换到当前位形下对末端的贡献。

---

## 3. 核心区别对比

| 特性 | **乘积指数公式（FK）** | **空间雅可比** |
|------|------------------------|----------------|
| **功能** | 计算**位姿**（几何量） | 计算**速度映射**（微分量） |
| **输出类型** |$SE(3)$矩阵（4×4） |$\mathbb{R}^{6\times n}$矩阵 |
| **变量** | 关节位置$\theta$| 关节速度$\dot{\theta}$|
| **物理意义** | 离散的位置/姿态 | 瞬时速度传递关系 |
| **应用** | 轨迹规划、碰撞检测 | 速度控制、力控制、阻抗控制 |

---

## 4. 内在联系：微分关系

**空间雅可比是FK对时间的微分结果**！

对$T(\theta)$求时间导数：

$$

\dot{T} = \frac{d}{dt}\left(e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_n]\theta_n} M\right)

$$

利用链式法则和矩阵指数导数性质，可推导出：

$$

[\mathcal{V}_s] = \dot{T}T^{-1} = \sum_{i=1}^n \left(\frac{\partial T}{\partial \theta_i} \dot{\theta_i}\right)T^{-1} = \sum_{i=1}^n J_{s,i}(\theta) \dot{\theta}_i

$$

**关键桥梁**：**雅可比的列向量 = FK中对应关节的偏导数**

---

## 5. 几何直观

### 5.1 PoE的几何意义

每个$e^{[\mathcal{S}_i]\theta_i}$是**刚体运动**——将连杆$i$从零点运动到$\theta_i$


### 5.2 空间雅可比的几何意义

第$i$列$J_{s,i}(\theta)$表示：当只有关节$i$以单位速度运动时，**末端产生的空间速度**。

---

## 6. 数值实现对比

### 6.1 FKinSpace（乘积指数）

```python
T = M.copy()
for i in reversed(range(n)):
    T = MatrixExp6(Slist[:, i] * theta[i]) @ T
```

### 6.2 空间雅可比计算

```python
T = np.eye(4)
J = np.zeros((6, n))
for i in range(n):
    # 每列计算伴随变换
    J[:, i] = Adjoint(T) @ Slist[:, i]
    T = T @ MatrixExp6(Slist[:, i] * theta[i])
```

**区别**：
- FK只关心**最终位姿**
- 雅可比需要**累积中间变换**（伴随矩阵）

---

## 7. 总结

- **乘积指数公式**是**静态的几何映射**（$\theta \to T$）
- **空间雅可比**是**动态的微分映射**（$\dot{\theta} \to \mathcal{V}_s$）
- **本质关系**：雅可比是FK的**导数**，其列向量由PoE公式中的螺旋轴经伴随变换得到

两者共同构成了机器人运动学的完整描述：**位置层（FK）+ 速度层（Jacobian）**。
