---
title: MiniMind DPO
date: 2026-02-22
timestamp: 2026-02-22T18:24:32+08:00
slug: minimind-dpo
category: note
tags:
  - AI
  - LLM
---

# MiniMind DPO

## RLHF 与 DPO

- **传统 RLHF (比如 InstructGPT/ChatGPT 早期版本):**
    
    1. **SFT（监督微调）**：教模型说话。
        
    2. **训练 RM（奖励模型）**：让人类标注好坏，训练一个专门的“裁判模型”。你给它一段对话，它输出一个具体的分数（比如 8.5 分，2.1 分）。
        
    3. **PPO（强化学习）**：让 SFT 模型不断生成回答，RM 给它打分，模型根据分数来更新参数。这个过程极其吃显存，且极度不稳定。
        
- **DPO (直接偏好优化):**
    
    1. **SFT（监督微调）**：教模型说话（和上面一样）。
        
    2. **DPO 训练**：**直接**拿人类标注的好坏数据（Chosen 和 Rejected），通过一个巧妙的数学公式，一步到位更新模型参数。**没有独立的 RM，也没有 PPO。**

## DPO

### 第一部分：DPO 理论极简入门

在微调大模型时，SFT（监督微调）教会了模型“如何说话”，但我们还需要让模型知道“说什么话更好”（比如更礼貌、更安全、更有逻辑）。

传统的 RLHF 需要先训练一个“奖励模型（Reward Model）”来打分，然后用强化学习算法（PPO）来更新模型。

**DPO 的核心思想是：不需要单独的奖励模型。**

它通过数学推导证明了：**语言模型本身就可以隐式地当作奖励模型。** 我们只需要给模型提供成对的数据：一个 Prompt（提示词），一个 Chosen（好回答），一个 Rejected（坏回答）。

DPO 的目标是最大化这二者的奖励差，其核心数学公式如下：

$$L_{\text{DPO}} = - \log \sigma \left( \beta \log \frac{\pi_\theta(y_c | x)}{\pi_{\text{ref}}(y_c | x)} - \beta \log \frac{\pi_\theta(y_r | x)}{\pi_{\text{ref}}(y_r | x)} \right)$$

**公式里的变量在代码中对应什么？**

- $x$: 输入的提示词 (Prompt)。
    
- $y_c$: 被选中的好回答 (Chosen)。
    
- $y_r$: 被拒绝的差回答 (Rejected)。
    
- $\pi_\theta$: 我们正在训练的**策略模型 (Policy Model)**，代码里的 `model`。
    
- $\pi_{\text{ref}}$: 训练开始前的**参考模型 (Reference Model)**，作为基准，代码里的 `ref_model`。
    
- $\beta$: 控制模型偏离基准多远的超参数，代码里的 `beta`。
    
- $\sigma$: Sigmoid 函数。

**打分逻辑**：相比于一开始（参考模型），现在的模型有多么 **“更想”** 说这句话？

- 如果现在的模型比以前**更倾向于**说这句话，比值大于 1，取对数后就是一个**正数（正奖励）**。
    
- 如果现在的模型比以前**更不想**说这句话，比值小于 1，取对数后就是一个**负数（负惩罚）**。
    

---

### 第二部分：核心代码深度拆解

#### 1. `logits_to_log_probs`：提取真实Token的概率

大模型输出的是词表中每个词的得分（Logits）。我们需要知道它生成**真实回答**里那些词的概率有多大。

```python
def logits_to_log_probs(logits, labels):
    # 将原始得分转为对数概率 (Log-Softmax)
    log_probs = F.log_softmax(logits, dim=2)
    # 核心：使用 gather 沿着词表维度，把 labels（实际发生的词）对应的概率提取出来
    log_probs_per_token = torch.gather(log_probs, dim=2, index=labels.unsqueeze(2)).squeeze(-1)
    return log_probs_per_token
```

#### 2. `dpo_loss`：实现核心公式

这是整段代码的灵魂，完美复刻了上面的数学公式。

```python
def dpo_loss(ref_log_probs, policy_log_probs, mask, beta):
    # 1. 过滤 Padding（填充符）：只计算有真实Token的地方（mask=1）
    seq_lengths = mask.sum(dim=1, keepdim=True).clamp_min(1e-8)
    # 取平均每个Token的对数概率
    ref_log_probs = (ref_log_probs * mask).sum(dim=1) / seq_lengths.squeeze()
    policy_log_probs = (policy_log_probs * mask).sum(dim=1) / seq_lengths.squeeze()

    # 2. 切分数据：前一半是 Chosen，后一半是 Rejected
    batch_size = ref_log_probs.shape[0]
    chosen_ref_log_probs = ref_log_probs[:batch_size // 2]
    reject_ref_log_probs = ref_log_probs[batch_size // 2:]
    chosen_policy_log_probs = policy_log_probs[:batch_size // 2]
    reject_policy_log_probs = policy_log_probs[batch_size // 2:]

    # 3. 计算对数概率比值 (Log Ratios)
    # 正在训练的模型对“好/坏”回答的偏好差
    pi_logratios = chosen_policy_log_probs - reject_policy_log_probs
    # 参考模型对“好/坏”回答的偏好差
    ref_logratios = chosen_ref_log_probs - reject_ref_log_probs
    
    # 4. 计算最终的 DPO Loss（结合 beta 和 Sigmoid）
    logits = pi_logratios - ref_logratios
    loss = -F.logsigmoid(beta * logits)
    return loss.mean()
```

#### 3. `train_epoch`：训练循环的精妙设计

在这个函数中，最值得关注的是**数据拼接技巧**和**梯度阻断**。

```python
# 技巧1：把 Chosen 和 Rejected 拼接到一起（dim=0），一次性送进模型，节省显存和通信开销。
x = torch.cat([x_chosen, x_rejected], dim=0)
y = torch.cat([y_chosen, y_rejected], dim=0)

# 技巧2：参考模型不需要训练！所以用 torch.no_grad() 包裹，不计算它的梯度。
with torch.no_grad():
    ref_outputs = ref_model(x)
    ref_logits = ref_outputs.logits
ref_log_probs = logits_to_log_probs(ref_logits, y)

# 策略模型（正在训练的）则正常进行前向传播和计算
outputs = model(x)
logits = outputs.logits
policy_log_probs = logits_to_log_probs(logits, y)
```

随后代码执行了 `scaler.scale(loss).backward()`，这是使用了混合精度训练（AMP）的标准反向传播流程，能大幅降低显存占用并加速训练。

#### 4. `__main__`：工程化与分布式设置

这部分是典型的 PyTorch 工业级训练脚本模板：

- **模型克隆**：初始化了两个完全一样的模型。`model` 交给优化器训练；`ref_model` 执行 `.eval()` 和 `.requires_grad_(False)` 进行彻底冻结。
    
- **DDP (Distributed Data Parallel)**：`DistributedDataParallel(model, ...)` 用于多显卡并行训练。
    
- **Wandb 日志**：集成了 `swanlab` / `wandb` 来实时监控训练曲线（Loss 应该下降，模型对 Chosen 和 Rejected 的打分差距应该拉大）。

### 一些细节

#### 学习率设置

在 DPO 训练中，`learning_rate`（学习率）为什么要设置得比 SFT 时小得多

这背后的原因主要有三点：
 
##### 1. DPO 是在“精雕细琢”，而不是“大刀阔斧”

- **SFT 的任务**是让模型学会基本的语言规则、对话格式和知识。这需要模型对参数进行较大幅度的调整，也就是“从无到有”或“从乱到整”。
    
- **DPO 的任务**是在模型已经懂礼貌、会说话的基础上，微调它的**偏好倾向**。
    
    - 这就好比：SFT 是在教一个学生写作文（整体结构、语法）；而 DPO 是在教这个学生如何写得更优雅、更符合读者的胃口。
        
    - 如果学习率过大，模型会“用力过猛”，导致它为了迎合好坏对比，把 SFT 阶段辛辛苦苦学到的语言组织能力给破坏了，也就是常说的**灾难性遗忘**。
        

##### 2. 维持“参考模型”与“策略模型”之间的约束

DPO 的公式中隐藏着一个约束：它希望模型在变好的同时，**不要偏离参考模型（Reference Model）太远**。

- 公式里的 $\beta$ 参数和学习率共同控制着这种“偏离程度”。
    
- 如果学习率太高，模型会迅速冲向能够最大化奖励差值的方向，导致权重剧烈震荡。这种震荡会让模型生成一些虽然“看起来得分高”但逻辑混乱的代码或文字（这在强化学习中叫 **Reward Hacking**）。
    

##### 3. DPO 损失函数的敏感度

DPO 的损失函数（Log-Sigmoid 结构）对 Logits 的变化非常敏感。

- 大模型在经过 SFT 后，对某些 Token 的预测概率已经很高了。
    
- 在 DPO 训练中，我们要计算 $\log \frac{\pi_\theta}{\pi_{\text{ref}}}$。概率的微小变动在对数空间里会被放大。
    
- **极小的学习率（如 $10^{-8}$ 级别）**能保证模型在梯度的引导下，像走钢丝一样小心翼翼地移动，确保 `Chosen` 的概率一点点上升，`Rejected` 的概率一点点下降，而不会导致模型坍缩。

## DPODataset

这个 `DPODataset` 类是 DPO 训练中的“加工厂”。它的核心任务只有一件：**明确告诉模型，哪些词是用户问的（不计入 Loss），哪些词是模型回答的（需要计算概率并优化）。**

作为 RL 新手，理解这个类的关键在于理解 **Loss Mask（损失掩码）**。

---

### 1. 核心逻辑：谁才是“偏好”？

在 DPO 数据集中，每条数据都有 `chosen` 和 `rejected` 两个版本。

- **Prompt (问题)**：两个版本是一模一样的。
    
- **Answer (回答)**：一个好，一个坏。
    

**关键点：** 我们只关心模型生成“回答”部分的概率。如果把用户问的问题也拉进来算分，模型就会学偏（去预测用户会问什么，而不是学好坏）。

---

### 2. `generate_loss_mask`：手术刀般的精准标记

这是代码中最巧妙的地方。它通过扫描 `input_ids`，寻找 `assistant` 回答的起始和结束标志。

- **`self.bos_id`**：对应模板中回答的开始（例如 `<|im_start|>assistant\n`）。
    
- **`self.eos_id`**：对应回答的结束（例如 `<|im_end|>\n`）。
    

**逻辑过程：**

1. 默认 `loss_mask` 全是 `0`（不计算损失）。
    
2. 从头扫到尾，一旦发现 `bos_id`，说明**模型回答开始了**。
    
3. 从这个点开始，把 `loss_mask` 标记为 `1`，直到遇见 `eos_id`。
    
4. **结果**：只有 `assistant` 说的话被标记为 `1`，前面的 `user` 提问部分依然是 `0`。
    

---

### 3. `__getitem__`：数据的左右对齐

这个方法把原始文本变成了 PyTorch 训练需要的张量。

#### 文本转 ID

它使用了 `apply_chat_template`。这很重要，因为模型训练时必须带着特殊的 Token（如 `<|user|>` 等），否则模型无法分辨谁在说话。

#### 错位预测（Next Token Prediction）

注意这几行代码：

```python
x_chosen = torch.tensor(chosen_input_ids[:-1], dtype=torch.long)
y_chosen = torch.tensor(chosen_input_ids[1:], dtype=torch.long)
mask_chosen = torch.tensor(chosen_loss_mask[1:], dtype=torch.long)
```

这是语言模型训练的标准操作：

- **`x` (Input)**：从第 1 个字到倒数第 2 个字。
    
- **`y` (Target)**：从第 2 个字到最后一个字。
    
- **意义**：给模型看第 $t$ 个字，让它预测第 $t+1$ 个字。所以 `y` 相比 `x` 向后错开了一位。
    
- **`mask`**：同样错开一位，确保我们只在模型尝试预测“好/坏回答”的字符时，才计算 Loss。
    

---

### 4. 总结：它为 DPO 准备了什么？

当你从这个 Dataset 取出一个 Batch 时，你得到了：

1. **`x_chosen` / `x_rejected`**：输入给模型的序列。
    
2. **`y_chosen` / `y_rejected`**：模型应该预测出的正确词 ID。
    
3. **`mask_chosen` / `mask_rejected`**：一把“尺子”，划定了哪些位置是 `assistant` 的回答。
    

**回到 DPO 公式：**

我们在前一个回答里提到的 $\log \pi_\theta(y|x)$，在代码实现时，就是把模型输出的概率与 `y_chosen` 对比，并且只看 `mask` 为 `1` 的部分。