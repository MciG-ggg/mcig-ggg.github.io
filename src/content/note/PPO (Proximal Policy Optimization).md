---
title: PPO (Proximal Policy Optimization)
date: 2026-02-06
timestamp: 2026-02-06T22:48:06+08:00
slug: ppo-proximal-policy-optimization
description: PPO (Proximal Policy Optimization) 理论 PPO 是为了解决 A2C 训练不稳定的问题。它最核心的贡献是限制了策略更新的“步长”。 数学公式 PPO 不直接优化 $\log \pi$...
category: note
---

# PPO (Proximal Policy Optimization)

## 理论
PPO 是为了解决 A2C 训练不稳定的问题。它最核心的贡献是限制了策略更新的“步长”。

### 数学公式

PPO 不直接优化 $\log \pi$，而是优化新旧策略的比率 $r_t(\theta) = \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}$。

为了防止 $r_t(\theta)$ 剧烈波动，它使用了 **Clipped Objective**：

$$L^{CLIP}(\theta) = \mathbb{E} \left[ \min(r_t(\theta) A_t, \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon) A_t) \right]$$
其中：`clip(x, min, max)` ：
- 如果数值在区间内，保持不变。 
- 如果数值超过最大值，就让它等于最大值。
- 如果数值小于最小值，就让它等于最小值。



### 理解 Loss 的梯度截断
![](./images/PPO-(Proximal-Policy-Optimization)Clip.png)
- **关键点**：
    - 当优势 $A_t > 0$ 时，如果新策略比旧策略强太多（$p_{t}(\theta)$ 比率超过 $1+\epsilon$），梯度截断，不再更新。**防止一次性过度增加好策略的概率**
    - 当优势 $A_t < 0$ 时，如果新策略比旧策略弱太多（$p_{t}(\theta)$ 比率低于 $1-\epsilon$），梯度截断，不再更新。**防止一次性过度降低坏策略的概率**

但是 PPO 的 Loss 不能只包含这一部分。
PPO 的 Loss 就像是为一个“追求进步但又害怕步子太大”的智能体设计一套平衡机制。

#### 1. 数学公式

PPO 的总损失函数（Total Loss）由三部分加权组成：**策略损失**（引导动作）、**价值损失**（精准打分）和**熵增益**（鼓励探索）。

$$L_t^{PPO}(\theta) = \mathbb{E}_t [ \underbrace{L_t^{CLIP}(\theta)}_{\text{策略项}} - c_1 \underbrace{L_t^{VF}(\theta)}_{\text{价值项}} + c_2 \underbrace{S[\pi_\theta](s_t)}_{\text{熵项}} ]$$

##### ① 策略项 (Clipped Surrogate Objective)

这是 PPO 的核心，通过“剪裁”限制更新幅度：

$$L_t^{CLIP}(\theta) = \min \left( \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)} \hat{A}_t, \text{clip}\left(\frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}, 1-\epsilon, 1+\epsilon\right) \hat{A}_t \right)$$

- **作用**：如果动作好（$\hat{A}_t > 0$），增加概率但不准超过 $1+\epsilon$；如果动作坏，减少概率但不准低于 $1-\epsilon$。

##### ② 价值项 (Value Function Loss)

通常使用均方误差（MSE）：

$$L_t^{VF} = (V_\theta(s_t) - V_{target})^2$$

- **作用**：让 Critic 预测的分数 $V(s)$ 尽可能接近环境给出的真实奖励。

##### ③ 熵项 (Entropy Bonus)

$$S = -\sum \pi(a|s) \log \pi(a|s)$$

- **作用**：防止模型太快“认死理”，强制保留一点随机性去探索。

---
### 2. 参数调节直觉表

|**成分**|**符号**|**核心作用**|**调大该系数的影响**|
|---|---|---|---|
|**Clip 范围**|$\epsilon$|控制更新步长|步子迈得更大，训练更快但也更容易崩盘。|
|**价值权重**|$c_1$|训练打分员|模型对“好坏”的判断会更灵敏，但可能干扰动作学习。|
|**熵权重**|$c_2$|强制探索|智能体会表现得更“多动”，不容易陷入局部最优，但收敛变慢。|

## 实现细节
来自 

```cardlink
url: https://iclr-blog-track.github.io/2022/03/25/ppo-implementation-details/
title: "The 37 Implementation Details of Proximal Policy Optimization · The ICLR Blog Track"
host: iclr-blog-track.github.io
```