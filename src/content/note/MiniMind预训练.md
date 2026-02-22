---
title: MiniMind预训练
date: 2026-02-22
timestamp: 2026-02-22T16:27:16+08:00
slug: minimind
category: note
tags:
  - AI
  - LLM
---

# MiniMind预训练


## 1. Tokenizer：大语言与人类语言的桥梁

### 1.1 如何理解Tokenizer

理解Tokenizer的关键在于把它看作一个**智能的字符编码器**，而非简单的字符串分割工具：

**核心思维转变：**
- **从字符串到数字**：不只是切分单词，而是建立字符到数字的映射关系
- **上下文感知**：同一个词在不同上下文中可能有不同的编码
- **效率优先**：在保持语义完整的前提下，追求最短的编码长度

**Tokenizer的工作哲学：**
```python
# 直观理解：把语言"压缩"成数字
text = "我爱自然语言处理"
tokens = tokenizer.tokenize(text)  # ['我', '爱', '自然', '语言', '处理']
input_ids = tokenizer.convert_tokens_to_ids(tokens)  # [251, 872, 1532, 2341, 8765]
```

### 1.2 核心定位

Tokenizer是模型与人类语言之间的"翻译字典"，其重要性体现在：

- **词表大小决定Embedding维度**：Tokenizer必须在预训练之前完全定型
- **数据预处理的第一步**：将原始文本转换为模型可理解的数字序列
- **性能关键因素**：Tokenizer直接影响模型的训练效率和最终表现

### 1.3 训练原理

Tokenizer本质上是基于统计学的分词算法，核心逻辑包括：

- **频率统计**：在海量文本中寻找出现频率最高的字符组合
- **合并策略**：将高频字符组合合并为Token（如Byte Pair Encoding）
- **效率优化**：优先使用Rust实现的Fast Tokenizer进行性能加速

```python
# AutoTokenizer自动选择最优实现
from transformers import AutoTokenizer

# 自动判断并加载极速的Rust版本
tokenizer = AutoTokenizer.from_pretrained("model_path")
```

### 1.4 关键特殊Token

| Token | 含义 | 作用 |
|-------|------|------|
| `<s>` / `[BOS]` | 序列开始标识 | 标记文本起始位置 |
| `</s>` / `[EOS]` | 序列结束标识 | 教模型学会何时停止生成 |
| `[PAD]` | 填充符 | 对齐Batch中不同长度的序列 |
| `[UNK]` | 未知字符 | 处理训练中未见的字符 |

## 2. 预训练核心技巧

### 2.1 梯度累计与梯度缩放

#### 梯度缩放的原理：用"放大镜"看小数字

想象你在用显微镜观察：直接看可能看不见，但放大后就清晰了。梯度缩放就是这个原理：

**直观理解：**
```python
# 原理：就像用放大镜看微小的东西
# 问题：梯度太小 → float16装不下 → 变成0
gradient = 0.00001  # 这个数在float16中会被"吃掉"变成0

# 解决方案：先把梯度"放大"
scaled_gradient = gradient * 65536  # 放大65536倍
# 现在scaled_gradient = 0.65536，float16能表示了！

# 更新时再"缩小"回来
actual_gradient = scaled_gradient / 65536  # 又变回0.00001
```

**为什么梯度会这么小？**
- 网络很深，梯度经过多层反向传播会越来越小
- Batch size大，梯度被平均后更小
- 学习率本身也是一个很小的数

#### PyTorch中的实现

```python
# 初始化"放大镜"
scaler = torch.cuda.amp.GradScaler(enabled=(dtype == 'float16'))

# 训练中的自动处理
with autocast_ctx:
    res = model(input_ids, labels=labels)
    loss = res.loss + res.aux_loss
    loss = loss / accumulation_steps  # 先除以累积步数

# scaler自动处理缩放
scaler.scale(loss).backward()  # 自动放大loss，反向传播得到放大后的梯度

# 更新时自动缩小
scaler.unscale_(optimizer)     # 把梯度恢复到原始大小
torch.nn.utils.clip_grad_norm_(model.parameters(), grad_clip)
scaler.step(optimizer)         # 正常更新参数
scaler.update()                # 更新缩放因子
```

> `scaler.scale(loss).backward()` 只是算梯度， 只有当`scaler.step(optimizer)`时，才真正动手更新网络参数

### 2.2 混合精度训练：精度与效率的平衡术

**核心思想：不同的运算用不同的"精度工具"**

**直观比喻：**
- **矩阵乘法**：就像大量数字的加减运算，用半精度（float16）就够了，又快又省空间
- **Loss计算**：就像最后汇总成绩，需要用单精度（float32）保证准确不丢失精度

**为什么这样设计？**
```python
# 自动精度切换的智慧
with torch.cuda.amp.autocast(dtype=dtype):
    # 前向传播：自动选择合适的精度
    # - 大矩阵运算 → float16（快！）
    # - 小规模运算 → 可能保持float32（准！）
    res = model(input_ids, labels=labels)

    # Loss计算：自动用更高精度
    # 避免反向传播时数值精度不够
    loss = res.loss + res.aux_loss
```

**精度选择策略：**
```python
# bfloat16 vs float16的选择逻辑
device_type = "cuda" if "cuda" in args.device else "cpu"
dtype = torch.bfloat16 if args.dtype == "bfloat16" else torch.float16

# bfloat16：数值范围更大，更稳定，但稍慢
# float16：更快，更省显存，但容易出现溢出
```

**实战中的配置：**
```python
# 智能配置混合精度
autocast_ctx = nullcontext() if device_type == "cpu" else torch.cuda.amp.autocast(dtype=dtype)

# 在训练循环中，PyTorch自动处理
with autocast_ctx:
    # 这里的运算会根据情况自动选择精度
    outputs = model(input_ids, attention_mask=attention_mask)
    loss = loss_fn(outputs, labels)
```

### 2.3 掩码损失

对于填充 tokens，我们不希望它们参与损失计算：

```python
# 自动处理：PyTorch会自动忽略填充位置的损失
# 在训练时只需要确保labels正确设置即可
res = model(input_ids, labels=labels)  # labels中PAD位置的损失会被自动掩码
loss = res.loss
```

> PyTorch 的交叉熵损失函数（`CrossEntropyLoss`）有一个默认参数 `ignore_index=-100`。 只要标签是 `-100`，计算 Loss 时该位置的权重就是 0，直接被跳过。

## 3. 模型保存与恢复：训练的"存档系统"

> `model.eval()` 暂时关闭 Dropout 和 Batch Norm 的随机性

### 3.1 模型"脱壳"：获取纯净的参数

**为什么需要"脱壳"？**
想象俄罗斯套娃：模型被各种包装包裹着，我们需要拿到最里面的那个：

```python
# 模型可能的"外壳"
# 1. DDP分布式包装
# 2. torch.compile编译包装
# 3. 自定义的包装

# 脱壳三部曲
raw_model = model.module if isinstance(model, DistributedDataParallel) else model
raw_model = getattr(raw_model, '_orig_mod', raw_model)
state_dict = raw_model.state_dict()
```

**解释：**
- `model.module`：从DDP包装中取出原始模型(**DDP包装后的样子：** `module.layer1.weight`， 取出来之后：`layer1.weight`)
- `getattr(raw_model, '_orig_mod', raw_model)`：从torch.compile中取出原始模型
- 如果都没有，直接使用当前模型

>[!note] 
> `state_dict`本质上是一个 **Python 字典 (OrderedDict)**。
>- **Key（键）：** 模型每一层的名字（如 `layer1.weight`, `layer1.bias`）。
>- **Value（值）：** 这一层对应的**张量（Tensor）**，即具体的参数数值。

### 3.2 权重压缩：75%的空间节省术

**保存策略的核心思路：**推理时才需要高精度，训练时可以"压缩"

```python
# 对state_dict里的k,v 压缩，复制一份到CPU后储存到磁盘
torch.save(
    {k: v.half().cpu() for k, v in state_dict.items()},
    checkpoint_path
)
```

**逐步解析：**
```python
# 1. 转半精度：从32位→16位，体积减半
v.half()  # torch.float32 → torch.float16

# 2. 移到CPU：释放GPU内存
v.cpu()   # 从显存移到内存

# 3. 保存：最终体积只有原来的25%
```

**为什么这样设计？**
- **存储效率**：一个7B模型，FP32需要28GB，FP16只需要14GB
- **推理兼容**：加载时自动转换到所需精度
- **备份友好**：文件更小，更容易备份和传输

### 3.3 完整状态保存：训练的"时间胶囊"

```python
# 保存不只是权重，还有训练的全部状态
checkpoint_data = {
    'model': model.state_dict(),      # 模型参数
    'optimizer': optimizer.state_dict(), # 优化器状态（动量等）
    'scaler': scaler.state_dict(),   # 梯度缩放器状态
    'epoch': epoch,                   # 训练进度
    'step': step,                     # 步数
    'wandb_id': wandb_id,            # 实验ID
    'config': lm_config               # 模型配置
}

# 一键恢复所有状态
if ckp_data:
    model.load_state_dict(ckp_data['model'])
    optimizer.load_state_dict(ckp_data['optimizer'])
    scaler.load_state_dict(ckp_data['scaler'])
    start_epoch = ckp_data['epoch'] + 1  # 从下一轮开始
```

**为什么需要保存这么多状态？**
- **优化器状态**：Adam的动量信息，突然恢复会导致学习率突变
- **梯度缩放器**：维护的动态缩放因子，防止梯度爆炸/消失
- **进度标记**：知道训练到哪里，避免重复训练

### 3.4 训练断点恢复

```python
# 保存完整训练状态
checkpoint_data = {
    'model': model.state_dict(),
    'optimizer': optimizer.state_dict(),
    'scaler': scaler.state_dict(),
    'epoch': epoch,
    'step': step,
    'wandb_id': wandb_id if wandb else None
}

# 恢复训练状态
if ckp_data:
    model.load_state_dict(ckp_data['model'])
    optimizer.load_state_dict(ckp_data['optimizer'])
    scaler.load_state_dict(ckp_data['scaler'])
    start_epoch = ckp_data['epoch']
    start_step = ckp_data.get('step', 0)
```

## 4. 生产级训练系统架构

### 4.1 分布式训练：多GPU的"协同作战"

**核心挑战：**
- 多个GPU要同步计算梯度
- 避免重复计算
- 保证每个GPU处理不同的数据

**DDP（DistributedDataParallel）的实现智慧：**

```python
# 1. 初始化分布式环境
local_rank = init_distributed_mode()
if dist.is_initialized():
    args.device = f"cuda:{local_rank}"  # 每个GPU一个ID

# 2. 包装模型
if dist.is_initialized():
    # 告诉DDP哪些参数不需要同步
    model._ddp_params_and_buffers_to_ignore = {"freqs_cos", "freqs_sin"}
    # 包装模型，自动处理梯度同步
    model = DistributedDataParallel(model, device_ids=[local_rank])
```

**为什么需要忽略某些参数？**
```python
# 位置编码等参数在不同GPU上应该是相同的
"freqs_cos": 这是预计算的位置编码，所有GPU共享
# 如果同步这些参数，会造成不必要的通信开销
```

### 4.2 梯度累积：模拟大batch的"时间魔法"

**问题：**显存不够，但又想用大的effective batch size

**解决方案：**分多次计算，累积梯度后再更新

```python
# 梯度累积的数学智慧
loss = loss / accumulation_steps  # 平均化，防止梯度爆炸

# 累积过程
accumulated_gradients = None
for i in range(accumulation_steps):
    loss = forward_pass()  # 计算当前batch的loss
    scaled_loss = loss / accumulation_steps

    # 累积梯度
    scaled_loss.backward()

    if accumulated_gradients is None:
        accumulated_gradients = model.parameters().grad.clone()
    else:
        accumulated_gradients += model.parameters().grad

# 累积完成后统一更新
optimizer.step()
optimizer.zero_grad()
```

**代码中的实现：**
```python
loss = loss / args.accumulation_steps  # 先除以累积步数
scaler.scale(loss).backward()          # 反向传播

# 累积到指定步数才更新
if (step + 1) % args.accumulation_steps == 0:
    scaler.step(optimizer)
    optimizer.zero_grad(set_to_none=True)  # 更高效的梯度清零
```

### 4.3 学习率调度：训练的"节奏控制器"

**为什么需要动态学习率？**
- 训练初期：需要较大的步子快速下降
- 训练中期：需要较小的步子精细调整
- 训练后期：需要更小的步子避免震荡

**余弦退火：优雅的训练曲线**

```python
def get_lr(step, total_steps, base_lr):
    """
    余弦退火学习率调度
    从base_lr线性下降到0，再上升，形成平滑的余弦曲线
    """
    return base_lr * 0.5 * (1 + torch.cos(torch.pi * step / total_steps))

# 应用学习率
lr = get_lr(epoch * iters + step, args.epochs * iters, args.learning_rate)
for param_group in optimizer.param_groups:
    param_group['lr'] = lr
```

**学习率变化图：**
```
学习率
↑
|     ╭╮
|    ╭  ╮
|   ╭    ╮
|  ╭      ╮
| ╭        ╮
|╭          ╮
+─────────────→ 训练步数
```

## 5. 实战代码解析

### 5.1 训练主循环：八大步骤的精密协作

```python
def train_epoch(epoch, loader, iters, start_step=0, wandb=None):
    start_time = time.time()
    for step, (input_ids, labels) in enumerate(loader, start=start_step + 1):

        # ===<mark> 步骤1：数据准备 </mark>===
        input_ids = input_ids.to(args.device)  # 移到GPU
        labels = labels.to(args.device)        # 标签也移到GPU

        # ===<mark> 步骤2：学习率调度 </mark>===
        # 全局步数 = 轮数 × 每轮步数 + 当前步数
        lr = get_lr(epoch * iters + step, args.epochs * iters, args.learning_rate)
        for param_group in optimizer.param_groups:
            param_group['lr'] = lr  # 更新所有参数组的学习率

        # ===<mark> 步骤3：前向传播 </mark>===
        with autocast_ctx:  # 自动精度切换
            # 模型计算，返回结果
            res = model(input_ids, labels=labels)
            # 总损失 = 主损失 + MoE辅助损失
            loss = res.loss + res.aux_loss
            # 平均化，考虑梯度累积
            loss = loss / args.accumulation_steps

        # ===<mark> 步骤4：反向传播 </mark>=<mark>
        # 自动缩放loss，反向传播得到缩放后的梯度
        scaler.scale(loss).backward()

        # </mark>=<mark> 步骤5：梯度检查点 </mark>=<mark>
        # 累积到指定步数才更新
        if (step + 1) % args.accumulation_steps </mark> 0:

            # ===<mark> 步骤6：梯度处理 </mark>=<mark>
            scaler.unscale_(optimizer)     # 恢复梯度原始大小
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip)  # 裁剪

            # </mark>=<mark> 步骤7：参数更新 </mark>===
            scaler.step(optimizer)         # 更新参数
            scaler.update()                # 更新缩放因子
            optimizer.zero_grad(set_to_none=True)  # 清空梯度

        # ===<mark> 步骤8：状态监控 </mark>=<mark>
        if step % args.log_interval </mark> 0 or step == iters - 1:
            spend_time = time.time() - start_time
            current_loss = loss.item() * args.accumulation_steps
            current_aux_loss = res.aux_loss.item() if res.aux_loss is not None else 0.0
            current_logits_loss = current_loss - current_aux_loss
            current_lr = optimizer.param_groups[-1]['lr']

            # 计算剩余时间
            eta_min = spend_time / (step + 1) * iters // 60 - spend_time // 60

            Logger(f'Epoch:[{epoch + 1}/{args.epochs}]({step}/{iters}), '
                   f'loss: {current_loss:.4f}, logits_loss: {current_logits_loss:.4f}, '
                   f'aux_loss: {current_aux_loss:.4f}, lr: {current_lr:.8f}, '
                   f'epoch_time: {eta_min:.1f}min')

            if wandb:
                wandb.log({
                    "loss": current_loss,
                    "logits_loss": current_logits_loss,
                    "aux_loss": current_aux_loss,
                    "learning_rate": current_lr,
                    "epoch_time": eta_min
                })
```

### 5.2 模型保存策略：训练的"存档点"

**保存时机的智慧：**
```python
# 保存条件：定期保存或最后一轮
if (step % args.save_interval <mark> 0 or step </mark> iters - 1) and is_main_process():
    model.eval()  # 切换到评估模式

    # ===<mark> 智能文件名生成 </mark>===
    moe_suffix = '_moe' if lm_config.use_moe else ''
    ckp = f'{args.save_dir}/{args.save_weight}_{lm_config.hidden_size}{moe_suffix}.pth'

    # ===<mark> 模型"脱壳" </mark>===
    raw_model = model.module if isinstance(model, DistributedDataParallel) else model
    raw_model = getattr(raw_model, '_orig_mod', raw_model)
    state_dict = raw_model.state_dict()

    # ===<mark> 权重压缩存储 </mark>=<mark>
    torch.save({k: v.half().cpu() for k, v in state_dict.items()}, ckp)

    # </mark>=<mark> 完整状态保存 </mark>===
    lm_checkpoint(lm_config, weight=args.save_weight, model=model,
                  optimizer=optimizer, scaler=scaler, epoch=epoch,
                  step=step, wandb=wandb, save_dir='../checkpoints')

    model.train()  # 切换回训练模式
```

**保存策略详解：**

1. **双保存机制**：
   - 只保存权重（.pth）：用于推理
   - 完整状态（checkpoint）：用于续训

2. **分布式控制**：
   ```python
   if is_main_process():  # 只有主进程保存，避免冲突
   ```

3. **智能命名**：
   - 包含模型大小：`hidden_size`
   - 区分架构：`_moe`后缀
   - 可追溯：`save_weight`前缀

4. **内存管理**：
   - 保存前切换到eval模式
   - 保存后立即切换回train模式
   - 及时清理中间变量
   ```python
   del state_dict  # 释放内存
   ```

## 6. 最佳实践总结

### 6.1 内存优化：显存的"精打细算"

**1. 梯度缩放 - 防止"数字消失"**
```python
# 问题：float16太小 → 梯度变0 → 无法更新
# 解决：自动放大→计算→缩小
scaler = torch.cuda.amp.GradScaler()
scaler.scale(loss).backward()  # 自动放大
scaler.step(optimizer)         # 自动缩小
```

**2. 混合精度 - "该省则省，该精则精"**
```python
# 矩阵运算：半精度（快！省！）
# 损失计算：单精度（准！稳！）
with torch.cuda.amp.autocast():
    outputs = model(input_ids)
    loss = loss_fn(outputs, labels)
```

**3. 梯度累积 - "时间换空间"**
```python
# 显存不够？分多次计算！
loss = loss / accumulation_steps  # 平均化
loss.backward()                    # 累积梯度
if step % accumulation_steps == 0:
    optimizer.step()              # 统一更新
```

**4. 权重压缩 - "存75%，用25%"**
```python
# 训练完后压缩75%体积
torch.save({k: v.half().cpu() for k, v in state_dict.items()}, ckp)
# 加载时自动解压，毫无感知
```

### 6.2 训练稳定性：模型的"健康守护"

**1. 梯度裁剪 - 防止"失控"**
```python
# 把梯度限制在合理范围内
torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
```

**2. 学习率调度 - "平缓降落"**
```python
# 余弦退火：从高到低，平滑过渡
lr = base_lr * 0.5 * (1 + cos(π * step / total_steps))
# 避免震荡，稳定收敛
```

**3. 随机种子 - "可复现的魔法"**
```python
# 每次结果都一样
setup_seed(42 + rank)  # 不同GPU用不同种子但固定
```

**4. DDP智能同步 - "减少通信开销"**
```python
# 位置编码等参数不需要同步
model._ddp_params_and_buffers_to_ignore = {"freqs_cos", "freqs_sin"}
```

### 6.3 生产部署：企业级实践

**1. 断点续训 - "永不丢失进度"**
```python
# 保存完整的训练状态
checkpoint = {
    'model': model.state_dict(),
    'optimizer': optimizer.state_dict(),
    'scaler': scaler.state_dict(),
    'epoch': epoch,
    'step': step
}
```

**2. 分布式支持 - "多GPU协同"**
```python
# 自动初始化分布式环境
local_rank = init_distributed_mode()
if dist.is_initialized():
    model = DistributedDataParallel(model, device_ids=[local_rank])
```

**3. 日志监控 - "一切尽在掌握"**
```python
# 实时监控训练指标
wandb.log({
    "loss": current_loss,
    "learning_rate": current_lr,
    "epoch_time": eta_min
})
```

**4. 编译优化 - "极致性能"**
```python
# 一键编译优化
model = torch.compile(model)  # 自动优化计算图
```

## 7. 实用技巧与避坑指南

### 7.1 配置调优经验值

**Batch Size vs 显存权衡：**
```python
# 8GB显存配置示例
batch_size = 16          # 可用batch size
accumulation_steps = 4    # 梯度累积4次，effective batch = 64
hidden_size = 512         # 模型大小
max_seq_len = 512         # 序列长度

# 显存估算：batch_size * max_seq_len * hidden_size * 4 bytes
# ≈ 16 * 512 * 512 * 4 ≈ 16MB （实际会更多，考虑KV cache等）
```

**学习率选择策略：**
```python
# 根据模型大小调整学习率
learning_rate = 5e-4 * (hidden_size / 512)  # 隐藏层越大，学习率稍大
# 基础学习率 5e-4，512隐藏层作为基准
```

### 7.2 常见问题诊断

**1. CUDA out of memory**
```python
# 解决方案
- 减少batch_size
- 增加accumulation_steps
- 使用gradient checkpointing
- 降低hidden_size
```

**2. Loss不下降**
```python
# 检查清单
- 学习率是否合适？
- 梯度裁剪阈值是否太严格？
- 数据是否乱序？
- 模型是否正确初始化？
```

**3. 分布式训练卡住**
```python
# 排查步骤
- 检查NCCL环境变量
- 确认GPU数量与进程数匹配
- 检查防火墙设置
- 查看DDP同步日志
```

### 7.3 性能优化秘籍

**1. torch.compile的使用**
```python
# 简单启用
model = torch.compile(model)

# 高级配置（PyTorch 2.0+）
model = torch.compile(
    model,
    mode="reduce-overhead",  # 减少开销模式
    fullgraph=True,         # 完整图优化
    dynamic=True            # 动态图支持
)
```

**2. 内存优化技巧**
```python
# 启用内存高效的操作
torch.backends.cuda.enable_mem_efficient_attention(True)

# 梯度检查点（节省显存但增加计算）
from torch.utils.checkpoint import checkpoint

def custom_forward(x):
    return model(x)

# 使用检查点
output = checkpoint(custom_forward, input_ids)
```