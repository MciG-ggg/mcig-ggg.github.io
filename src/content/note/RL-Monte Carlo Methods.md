---
title: RL-Monte Carlo Methods
tags:
  - conclusion/rl
  - conclusion/embodied-ai
publish: "true"
---

# RL-Monte Carlo Methods

## 说在前面

这是接触的第一个 model free 的 rl 算法, model free的意思就是没有前面 model base算法时已知的:

$$
状态转移预测P(s'|s, a), 奖励预测P(r|s, a)
$$

model free 算法主要有三类:
1. 基于价值的方法 (Value-Based): 学习 $V(s)$ 或者 $Q(s, a)$
2. 基于策略的方法 (Policy-Based): 直接学习**随机策略 $\pi(a|s)$** 或者**确定性策略 $a = \pi(s)$**
3. 两者结合 (Actor-Critic): **Actor** 负责做动作，**Critic** 负责给动作打分。

> [!NOTE] 策略的分类
> 策略有两种,随机策略和确定性策略
> 1. 随机策略: 给出的是一个状态下选择各个动作的概率, 输出是概率, 一般用于动作是离散的时候, 随机性策略本身就具有内在的随机性,有利于探索
> 2. 确定性策略: 给出的是一个状态下选择什么动作, 输出是一个具体的动作, 一般用于动作是连续的时候, 没有内在的随机性

> 没有模型就需要数据,没有数据就需要模型

所以 model free 算法需要大量的采样数据, 采样数据必须是i.i.d (独立同分布), 否则没有以下的性质:
1. 大数定理 (LLN): 样本均值 $\bar{X}$ 几乎处处收敛到期望 $\mu$ , $P(\lim_{n \to \infty} \bar{X}_n = \mu) = 1$ , 若失效,则均值估计会有偏差（Bias）。你以为某个动作能得 10 分，其实是因为你刚好在那个局部区域连走了好几步，这叫**相关性偏差**。
2. 中心极限定理 (CLT):大量样本的均值分布都会趋向于正态分布（高斯分布）。若失效, 则 SGD (随机梯度下降不稳定)

![[Pasted image 20251228093821.png]]

## 1. MC Basic

这个算法把 model-based 的 policy iteration中的 policy evaluation 换成了 model-free 的 MC estimation

policy iteration有两步: 1. policy evaluation (估计 $q_{\pi_k}(s,a)$ ) 2. policy improvement (改进$\pi$)

MC basic就是采用了蒙特卡罗的办法, 用采样数据的期望来估计*动作值* $q$ , 有:

$$
q_{\pi_k}(s,a) = \mathbb{E}[G_t|S_t=s, A_t=a] \approx \frac{1}{n} \sum_{i=1}^{n}g_{\pi_k}^{(i)}(s,a)
$$

当 n 很大的时候,我们可以认为等式最右边就是$q_{\pi_k}(s,a)$

### 算法实现

![[Pasted image 20251228170248.png]]
1. policy evaluation
采样一整个episode, 然后用这个 episode 里每一步的 reward, 乘上 discounted rate ( $\gamma$ ), 最终得到整个 episode 的 return, 用这个 return 去估计 $q_{\pi_k}(s,a)$ (k为第k次迭代)
2. policy improvement
对一个状态的动作, 选择 $a_k^*(s) = argmax_a q_k(s,a)$ , 即 $\pi(a|s) =1$ if $a = a^*_k$ and $\pi(a|s) = 0$ otherwise

## 2. MC Exploring Starts

是 MC Basic的改进

### 提高sample的利用率

之前 MC Basic 采样一个 episode 的时候, 只用这一个 episode 去估计最开始的那个 (s, a) 的 $q_{\pi_k}(s,a)$ ,但是实际上, 一个 episode 对访问到很多对 (s, a)(state-action pair), 可以用这个episode 中不同的 (s, a) 做起点, 来得到更多的subepisode 如:

$$
\begin{align*}
s_1 \xrightarrow{a_2} s_2 \xrightarrow{a_4} s_1 \xrightarrow{a_2} s_2 \xrightarrow{a_3} s_5 \xrightarrow{a_1} \dots \quad & \text{[original episode]} \\
s_2 \xrightarrow{a_4} s_1 \xrightarrow{a_2} s_2 \xrightarrow{a_3} s_5 \xrightarrow{a_1} \dots \quad & \text{[subepisode starting from } (s_2, a_4)\text{]} \\
s_1 \xrightarrow{a_2} s_2 \xrightarrow{a_3} s_5 \xrightarrow{a_1} \dots \quad & \text{[subepisode starting from } (s_1, a_2)\text{]} \\
s_2 \xrightarrow{a_3} s_5 \xrightarrow{a_1} \dots \quad & \text{[subepisode starting from } (s_2, a_3)\text{]} \\
s_5 \xrightarrow{a_1} \dots \quad & \text{[subepisode starting from } (s_5, a_1)\text{]}
\end{align*}
$$

这样的利用方法有两种:
1. first-visit strategy: 只使用一个 (s, a) pair在一个episode中访问第一次来估计这个 (s,a) 对应的 $q_{\pi_k}(s,a)$
2. every-visit strategy:一个(s, a) pair在一个episode被访问的所有次数都拿来估计这个 (s,a) 对应的 $q_{\pi_k}(s,a)$

### 提高更新策略的效率

- MC Basic 的策略（低效）： 在进行策略改进前，必须针对同一个状态-动作对 $(s, a)$ 收集所有预设的回合，计算平均回报。这种“批处理”方式导致智能体必须等待大量数据采集完成后才能更新一次策略。
- MC ES 的策略（高效）： 采用“逐回合更新”（Episode-by-episode）。只要完成一个回合的采样，就立即利用该回合的回报来更新 $Q$ 值并改进策略。

### 算法实现

![[Pasted image 20251229100954.png]]
- 重点是 exploring starts condition, 随机选择开始的 (s, a), 保证每一个 (s, a) pair 都被很好的遍历到, 提高估计的准确性

> [!NOTE]
> - exploring starts condition 实际情况下很难满足, 因为首先初始状态不一定能随机设置, 很多都是由环境自然产生, 然后就是如果状态空间很大, 甚至连续, 即使运行很多的 episode, 也会有不少的pair没有被访问到, 还有就是物理系统中强制探索是危险或者昂贵的,比如说可能导致机器人损伤等等
> - 要提升每一个 (s, a) pair 被访问的次数, 除了随机选择episode的起点, 还有别的方法, 比如说下面的 epsilon greedy

### MC epsilon-greedy

为了在不依赖随机起点的情况下实现充分探索，算法引入了**“软策略” (Soft Policy)**：

$\epsilon$ -greedy 逻辑：
- 以绝大多数概率执行当前的最优动作（贪心动作）。
- 以极小的概率 $\epsilon$ 均匀地尝试所有可能的动作。
- 数学分配：最优动作的概率为 $1 - \frac{|\mathcal{A}(s)|-1}{|\mathcal{A}(s)|}\epsilon$ ，其余动作概率各为 $\frac{\epsilon}{|\mathcal{A}(s)|}$ 。
