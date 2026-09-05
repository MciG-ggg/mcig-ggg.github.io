---
title: Flow and Diffusion Model
date: 2026-03-12
timestamp: 2026-03-12T16:58:40+08:00
slug: flow-and-diffusion-model
category: note
tags:
  - AI
  - ML
  - Generative-Model
  - Course
  - MIT6S184
series: Diffusion-and-Flowmatching
---

## Flow是什么

流本身是一个抽象的数学概念
在抽象层面上，一个**流（Flow）** 本质上是**实数加法群 $(\mathbb{R}, +)$ 在某个空间 $M$（比如流形或 $\mathbb{R}^d$）上的连续（或光滑）群作用。**
> 如果我们把**群 (Group)** 比作一个装满变换指令的“工具箱”，那么**群作用 (Group Action)** 就是这些指令具体作用在某个“对象”上的过程

用数学语言表达就是：

给定一个空间 $M$，一个流是一个映射 $\phi : \mathbb{R} \times M \to M$，通常简记为 $\phi(t, x)$ 或 $\phi_t(x)$，它表示将空间中的点 $x$ 经过时间 $t$ 映射到另一个点

既然流本身是一个“群作用”，它就必须绝对服从两个黄金公理。这两个公理赋予了流完美的代数结构：

### 公理一：单位元作用（Identity / Initial Condition）

对于空间 $M$ 中的任意一点 $x$，必须满足：

$$\phi_0(x) = x$$

**直观理解：** “时间经过了 0 秒”，等于什么都没发生，空间中的每一个点都留在原地。

### 公理二：群同态 / 复合公理（Composition / Group Law）

对于任意时间 $s, t \in \mathbb{R}$ 以及空间中的任意一点 $x \in M$，必须满足：

$$\phi_{s+t}(x) = \phi_s(\phi_t(x))$$

**直观理解（时间平移不变性）：** 这就像是“时间旅行的加法”。你让系统先演化 $t$ 秒到达 $\phi_t(x)$，然后再把这个新位置作为起点继续演化 $s$ 秒。这和你直接让系统从头开始一口气演化 $s+t$ 秒，**到达的终点必须是一模一样的**。

### 核心推论：流的“可逆性”

由上面两个公理，我们可以立刻推导出一个极其重要的性质——**每一个时间 $t$ 的流动，都存在一个完美的逆变换**。

令 $s = -t$，代入公理二：

$$\phi_{-t}(\phi_t(x)) = \phi_{0}(x) = x$$

这意味着 $\phi_{-t}$ 就是 $\phi_t$ 的逆映射（$\phi_t^{-1}$）。如果你能顺着流走 $t$ 秒，你就一定能严格地倒退回去。这在数学上说明，对于每一个确定的时间 $t$，$\phi_t$ 都是空间 $M$ 上的一个**微分同胚（Diffeomorphism）**（即双射且两边都光滑）。
## Flow 模型的定义

我们的目标是把一个简单的分布 $p_{init}$ 转换成一个复杂的分布 $p_{data}$

而**概率分布也可以理解为一种流形（？）**
所以可以用 **流** 这个 **群操作**，对概率分布进行操作

> [!note] 流形和群作用之间的关系
> 1. 动词与名词的关系 (Transformations vs. Space)
> - 流形（名词）： 是一个点集。比如球面 $S^2$。它定义了点与点之间的邻里关系（拓扑）和弯曲程度（度量）。
> - 群作用（动词）： 是一种“操作”。比如“旋转”。
> - 结合： 当群 $G$ 作用在流形 $M$ 上时（记作 $G \times M \to M$），它定义了空间的一种对称性。
> 	- 例子： 你旋转一个完美的球体，球体看起来没变。这意味着“旋转群 $SO(3)$”在“球面流形 $S^2$”上的作用保持了流形的几何特性。
> 2. 李群：当“动词”本身也是“名词”
> 这是两者关系最紧密的地方。有些群本身由于太过于“平滑”，以至于它们自己就是一个光滑流形。这就是李群（Lie Group）。
> - 双重身份： 李群既是一个符合群公理的代数结构，又是一个可以做微积分的光滑流形。
> - 直观理解： 所有的旋转动作构成了一个空间。在这个“旋转空间”里，你可以从一个旋转平滑地过渡到另一个旋转。


Flow 模型的定义如下
$$
\begin{aligned}
    X_{0} = \phi_t(x_{0})&\sim p_{\text{init}} && \blacktriangleright \text{ random initialization} \\
    \frac{\text{d}}{\text{d}t} X_{t}= \frac{\partial}{\partial t} \phi_t(x) &= u_t^\theta(\phi_t(x)) = u_t^\theta(X_t) && \blacktriangleright \text{ ODE}
    \\
    X_t &= \phi_t(X_0)
\end{aligned}
$$
> 先从 $p_{init}$ 进行初始化

其中向量场 $u_{t}^{\theta}$ 是一个由参数 $\theta$ 参数化的神经网络，即
$$u_{t}^{\theta}: \mathbb{R}^{d} \times [0, 1] \rightarrow \mathbb{R}^{d} $$
可以理解为：$$(x, t) \mapsto v$$
虽然叫做 **Flow模型** ，但是**实际上神经网络参数化的是向量场，而非流本身**


### 模拟 ODE

> [!abstract] Algorithm 1: Euler Sampling
> **Require:** Vector field $u_t^\theta$, steps $n$
> 1. Set $t = 0$
> 2. Set $h = 1/n$
> 3. Sample $X_0 \sim p_{\text{init}}$
> 4. **for** $i = 1$ to $n-1$ **do**:
>     - $X_{t+h} = X_t + h u_t^\theta(X_t)$
>     - $t = t + h$
> 5. **return** $X_1$

Flow 模型是基于 **常微分方程(ODE)** 的， 因为从 $p_{init}$ 到 $p_{data}$ 的轨迹是一条确定性轨迹
## 扩散模型的定义

而扩散模型是基于 **随机微分方程(SDE)** 的， 其中轨迹是随机的，随机的轨迹又被成为 **随机过程**$(X_{t})_{0 \leq t \leq 1}$

$$
\begin{align}
&X_{t} \text{ is a random variable for every } 0 \leq t \leq 1 \\
X: [0, 1]\rightarrow \mathbb{R}^{d}, t \mapsto X_{t} &\text{is a random trajectory for every draw of X}
\end{align}
$$

### 布朗运动

布朗运动 $W = (W_t)_{0 \le t \le 1}$ 是 SDE 的核心随机项，具有以下性质和模拟公式：
- **正态增量：** $W_t - W_{t'} \sim \mathcal{N}(0, (t-t')I_d)$，即增量服从高斯分布，方差随时间线性增加。
- **独立增量：** 在任意不同时间段内的增量相互独立。
- **离散模拟公式：**$$W_{t+h} = W_t + \sqrt{h}\epsilon_t, \quad \epsilon_t \sim \mathcal{N}(0, I_d)$$通过设置步长 $h > 0$，可以用此公式近似模拟布朗运动的轨迹。

### ODE 到 SDE 的转变
#### 常微分方程 (ODE) 的表达
- **导数形式：** $\frac{\mathrm{d}}{\mathrm{d}t}X_t = u_t(X_t)$
- **无穷小更新形式：** $X_{t+h} = X_t + h u_t(X_t) + h R_t(h)$
    > 其中 $u_t(X_t)$ 是确定性方向，沿该方向迈出一小步。
    > 无穷小更新形式通过导数形式利用导数定义无穷小展开得来
    
#### 随机微分方程 (SDE) 的表达

而在 ODE 基础上加上来自布朗运动的随机扰动，则得到 SDE：
- **离散更新形式：**
    $$
    X_{t+h} = X_t + \underbrace{h u_t(X_t)}_{\text{deterministic}} + \underbrace{\sigma_t (W_{t+h} - W_t)}_{\text{stochastic}} + \underbrace{h R_t(h)}_{\text{error term}}
    $$
- **正式符号表示（微分形式）：**
    $$
    \begin{align}
	    \mathrm{d}X_t = u_t(X_t)\mathrm{d}t+ \sigma_t  \mathrm{d}&W_{t}\\ 
		\text{其中} &W_{t}-W_{t'} \sim  \mathcal{N}(0, t - t') \text{ 遵循布朗运动} \\
		
	\end{align}
    $$
    - $X_0 = x_0$ （初始条件）
    - $\sigma_t \ge 0$ 称为**扩散系数**。

如果我们只看 $t$ 时刻的 $W_t$，它的分布是固定的：
$$W_t \sim \mathcal{N}(0, t)$$


> [!example] Ornstein-Uhlenbeck(OU) 过程
> 令 $u_{t}(x) = -\theta x, \theta > 0$, 得到如下 SDE: 
> $$
> dX_{t} = - \theta X_{t}dt + \sigma_{t} dW_t
> $$

#### 模拟 SDE

>[!abstract] Algorithm 2: Euler-Maruyama Sampling
这是模拟 SDE 最简单的方法，其作用相当于 ODE 中的欧拉方法。
**Require:** Neural network $u_t^\theta$, steps $n$, diffusion coefficient $\sigma_t$
>1. **Set** $t = 0$
>2. **Set** step size $h = 1/n$
>3. **Sample** $X_0 \sim p_{\text{init}}$
>4. **for** $i = 1$ to $n-1$ **do**:
>    - Sample $\epsilon \sim \mathcal{N}(0, I_d)$
>    - $X_{t+h} = X_t + h u_t^\theta(X_t) + \sigma_t \sqrt{h} \epsilon$
>    - $t = t + h$
>5. **return** $X_1$ 

### 扩散模型

用神经网络来参数化 $u_{t}$, 有：
$$
\begin{align}
X_0 &\sim p_{\text{init}} && \blacktriangleright \text{ random initialization} \\
    \frac{\text{d}}{\text{d}t} X_t &= u_t^{\theta}(X_{t})dt+ \sigma_{t}dW_{t} && \blacktriangleright \text{ SDE}
\end{align}
$$

如果 $\sigma_{t} = 0$，则退化成 Flow 模型


> [!important] 扩散模型的所指
> - **狭义的扩散模型**： 通常指基于分数匹配（Score Matching）或去噪（Denoising）的模型。它们往往**被限制在特定的“加噪路径”上**（比如高斯路径），且推理过程依赖于模拟 SDE。
> 	- 因为**加噪加的都是高斯噪声**！
> - **广义的扩散模型**：即基于 SDE 的模型，没有具体随机路径限制
> 
> - **Flow Matching (FM)**： 它的核心思想是“匹配一个向量场”。它不仅可以匹配 SDE 产生的轨迹，还可以匹配直线轨迹（Optimal Transport/Rectified Flow）。
> 
> - 结论： Flow Matching 提供了一个更统一的数学框架。它告诉我们：只要能定义一条从噪声到数据的轨迹（无论这条轨迹是直的、弯的、随机的还是确定的），我们都可以通过训练一个向量场来模拟它。