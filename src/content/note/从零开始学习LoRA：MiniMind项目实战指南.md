---
title: ���零开始学习LoRA：MiniMind项目实战指南
date: 2026-02-22
timestamp: 2026-02-22T16:29:46+08:00
slug: loraminimind
category: note
tags:
  - AI
  - LLM
---

# 从零开始学习LoRA：MiniMind项目实战指南

## 理论基础

### 什么是LoRA？

LoRA（Low-Rank Adaptation）是一种高效的参数微调方法，由微软在2021年提出。它的核心思想是：**冻结预训练模型权重，只训练少量低秩矩阵来适配新任务**。

想象一下，你要让一个会弹钢琴的人去学拉小提琴。传统的做法是重新教他所有技巧（全参数微调），而LoRA的做法是：保持他原有的钢琴技艺不变，只教他几个特殊的指法技巧（低秩适配）。

### 为什么选择LoRA？

1. **参数量极少**：对于一个40B模型，rank=8时LoRA参数只占0.1-1%
2. **训练速度快**：只需优化少量低秩参数
3. **内存效率高**：无需存储完整的模型副本
4. **可组合性强**：可以在同一个模型上加载多个LoRA适配器

## 核心架构

### LoRA网络结构

让我先从最核心的LoRA类开始分析：

```python
class LoRA(nn.Module):
    def __init__(self, in_features, out_features, rank):
        self.rank = rank  # 低秩秩值（通常很小，如4-16）
        self.A = nn.Linear(in_features, rank, bias=False)   # 降秩矩阵 r×d
        self.B = nn.Linear(rank, out_features, bias=False)  # 升秩矩阵 d'×r
```

**关键设计点：**

- **低秩分解**：原始权重矩阵 W（d×d'）被分解为 W + BA，其中 BA 的秩为 r（r << d）
  - 数学原理：任何矩阵都可以分解为低秩矩阵的乘积
  - 实际效果：用少量的参数就能表示原始权重的"小幅度调整"

- **初始化策略**：矩阵A使用高斯初始化（std=0.02），矩阵B初始化为零
  - A矩阵：高斯分布提供良好的梯度流动
  - B矩阵：零初始化确保训练初期=W，避免突然改变

- **无偏置**：两个矩阵都使用bias=False，避免参数冗余

**为什么选择低秩？**
想象一下你要调整一张照片的亮度：
- 全参数微调：重新调整每个像素（太昂贵）
- LoRA：只调整一个"亮度滑块"（成本低，效果好）

rank值选择参考：
- rank=4-8：小模型，适合简单任务
- rank=8-16：中等模型，通用性强
- rank>16：大模型，接近全微调效果

### Module name的来源

这是我学习过程中遇到的一个难点：当你调用`model.named_modules()`时，PyTorch会递归地遍历模型中定义的所有子模块，并根据你在`__init__`中给它们起的变量名生成一条路径。

举个简单的例子：
```python
class MyModel(nn.Module):
    def __init__(self):
        super().__init__()
        self.transformer = TransformerBlock() # 变量名是 transformer

class TransformerBlock(nn.Module):
    def __init__(self):
        super().__init__()
        self.q_proj = nn.Linear(512, 512) # 变量名是 q_proj
```

当你运行`apply_lora(model)`后，`q_proj`内部被挂载了一个名为`lora`的子模块。此时，`named_modules()`返回的name就会是：

```
transformer
transformer.q_proj
transformer.q_proj.lora（这就是你要找的名字！）
```


### 参数化改造

```python
def apply_lora(model, rank=8):
    for name, module in model.named_modules():
        if isinstance(module, nn.Linear) and module.weight.shape[0] == module.weight.shape[1]:
            # 只为方形权重矩阵添加LoRA
            lora = LoRA(module.weight.shape[0], module.weight.shape[1], rank=rank)
            setattr(module, "lora", lora)
            # 修改forward函数
            original_forward = module.forward
            def new_forward(x, original_forward=original_forward, lora=lora):
                return original_forward(x) + lora(x)
            module.forward = new_forward
```

**改造策略：**

- 只处理方形矩阵（transformer中的注意力层通常是方阵）
- 在每个线性层内部添加LoRA模块
- 通过monkey patching修改forward函数：`output = original_forward(x) + lora(x)`

## 训练流程

### 训练配置

```python
# 关键超参数
args.learning_rate = 1e-4      # 较低的学习率，避免破坏预训练权重
args.epochs = 50               # 训练轮数
args.batch_size = 32           # 批次大小
args.rank = 8                  # LoRA秩值
```

### 参数冻结策略

```python
# 冻结非LoRA参数，只训练LoRA参数
for name, param in model.named_parameters():
    if 'lora' in name:
        param.requires_grad = True
        lora_params.append(param)
    else:
        param.requires_grad = False
```

**优势：**

- **参数量极少**：对于一个40B模型，rank=8时LoRA参数只占0.1-1%
  - 举例：BERT-large (355M参数)，LoRA rank=8时只需约1.4M参数
  - 存储空间：一个40B模型，LoRA权重可能只有几百MB

- **训练速度快**：只需优化少量低秩参数
  - 训练时间：通常比全微调快5-10倍
  - 收敛速度：LoRA参数少，更容易收敛

- **内存效率高**：无需存储完整的模型副本
  - 推理时：一个模型 + 多个小LoRA文件
  - 部署时：可以动态切换不同的LoRA适配器

### 权重管理

#### LoRA权重保存

```python
def save_lora(model, path):
    state_dict = {}
    for name, module in model.named_modules():
        if hasattr(module, 'lora'):
            lora_state = {f'{name}.lora.{k}': v for k, v in module.lora.state_dict().items()}
            state_dict.update(lora_state)
    torch.save(state_dict, path)
```

#### LoRA权重加载

```python
def load_lora(model, path):
    state_dict = torch.load(path, map_location=model.device)
    for name, module in model.named_modules():
        if hasattr(module, 'lora'):
            lora_state = {k.replace(f'{name}.lora.', ''): v for k, v in state_dict.items()
                         if f'{name}.lora.' in k}
            module.lora.load_state_dict(lora_state)
```

#### 文件结构：

主模型：`./checkpoints/model_name_hidden_size.pth`
LoRA权重：`./out/lora/lora_name_hidden_size.pth`

## 推理部署

### 动态加载LoRA

```python
if args.lora_weight != 'None':
    apply_lora(model)              # 应用LoRA结构
    load_lora(model, lora_path)    # 加载LoRA权重
```

### 实时切换能力

可以在同一个主模型上加载不同的LoRA权重
- 支持零样本提示（少样本学习）
- 每个LoRA文件代表一个特定任务的适配

**实际应用场景：**
比如你可以有一个基础的MiniMind模型，然后针对不同的任务训练不同的LoRA适配器：
- `lora_translation_768.pth` - 中文翻译任务
- `lora_qa_768.pth` - 问答任务
- `lora_summary_768.pth` - 文本总结任务

推理时只需要加载对应的LoRA文件即可，无需重新加载整个模型，这大大节省了内存和推理时间。