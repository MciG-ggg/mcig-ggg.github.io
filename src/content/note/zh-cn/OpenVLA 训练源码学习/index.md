---
title: OpenVLA 训练源码学习
date: 2026-02-20
timestamp: 2026-02-20T19:42:22+08:00
slug: openvla
category: note
tags:
  - AI
  - EmbodiedAI
  - VLA
---

# OpenVLA 训练源码学习

OpenVLA 基于 Prismatic-vlm [GitHub - TRI-ML/prismatic-vlms: A flexible and efficient codebase for training visually-conditioned language models (VLMs)](https://github.com/TRI-ML/prismatic-vlms) 这个 VLM 项目

笔者之前对 python 接触较少，只会基本语法，这篇是 python 语法，代码逻辑以及理论知识的整理，但不涉及OpenVLA模型的具体实现。

可以先看下面的博客内容学习下一些 python 的特有语法

```cardlink
url: https://www.laumy.tech/2334.html/python%E8%A1%A5%E4%B9%A0/
title: "python补习 - laumy的学习笔记"
description: "装饰器 函数装饰器 什么是装饰器 装饰器是python的一种高级语法，本质上是函数包装器，可以在不修改函数代码 […]"
host: www.laumy.tech
favicon: https://www.laumy.tech/wp-content/uploads/2024/11/cropped-IMG_6192-scaled-1-32x32.jpg
```


## Pretrain- 训练 Prismatic-vlm

其实这块是Prismatic-vlm的代码，但是顺便看一下吧
### 语法与第三方库

#### 1. DataClass

是 python 内置的装饰器，主要功能是自动生成 `__init__, __repr__, __eq__` 等方法， 经常用于创建配置类。详见 [python补习-内置装饰器@dataclass - laumy的学习笔记](https://www.laumy.tech/2334.html/python%E8%A1%A5%E4%B9%A0/)


> [!NOTE] DataClass 中的 Field 方法
> `field` 函数是 Python dataclasses 模块中的核心组件，用于自定义数据类字段的默认行为。主要作用：
> 
>1. **默认值工厂**：`default_factory` 参数允许创建复杂对象的工厂函数，而不是直接存储静态值
>2. **字段元数据**：可以添加类型提示、验证规则等额外信息
>3. **自定义序列化**：控制字段在序列化时的行为

#### 2. Enum

可以通过继承 `enum.Enum` 来定义一个枚举类：

```python
from enum import Enum

class Color(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3

# 访问方式
print(Color.RED)          # Color.RED
print(Color.RED.name)     # RED (获取成员名称)
print(Color.RED.value)    # 1   (获取成员的值)
```

其中 `RED` 其实是 `Color` 类的一个实例, 详见 [Python Enum里的为什么是instance而不是subclass](/python-enuminstancesubclass), 简单来说就是通过 metaclass,修改继承 `Enum` 类的类 (上面的 `Color` 类) 的初始化过程：
- 先在创建 `Color` 类的时候搜索到有属性 `RED`
- 然后根据 `RED` 创建一个实例
- 最后把这个实例赋值给 `RED`

#### 3. @contextmanager

- **作用**：让一个普通函数支持 `with` 语句，免去编写带有 `__enter__` 和 `__exit__` 魔法方法的复杂类的烦恼。
- **核心机制 (`yield`)**：
    - `yield` **之前**的代码：相当于 `__enter__`（准备工作，如加锁、计时开始）。
    - `yield` **本身**：将控制权交还给 `with` 语句块内部的代码。
    - `yield` **之后**的代码：相当于 `__exit__`（收尾工作，如释放锁、打印耗时）。
- **常见场景**：分布式进程同步（Barrier 罚站）、模型模式切换（如 `inference_mode`）、临时文件清理等。

#### 4. draccus库

在机器学习或大型后端项目中，我们经常需要处理复杂的配置。传统的 `argparse` 写起来很冗长，且缺乏类型检查。

简单来说，**Draccus** 是一个用于处理 **Python 配置对象**的库。它由 Lyft 开发，主要用于将类（Dataclasses）与命令行参数、配置文件（YAML/JSON）进行双向转换。

它的核心理念是：**让你的代码定义配置的形式，而不是让解析器定义它。**

例子如下：
```python 
import draccus
from dataclasses import dataclass

@dataclass
class OptimizerConfig:
    type: str = "adam"
    lr: float = 0.001

@dataclass
class ExperimentConfig:
    name: str = "default_run"
    # 嵌套子配置
    optim: OptimizerConfig = OptimizerConfig()

@draccus.wrap()
def main(cfg: ExperimentConfig):
    print(f"实验: {cfg.name}")
    print(f"优化器类型: {cfg.optim.type}, 学习率: {cfg.optim.lr}")

if __name__ == "__main__":
    main()
```

在命令行通过参数指定构建的类的参数：
```shell
python script.py --name "my_test" --optim.type "sgd" --optim.lr 0.01
```

还可以通过指定配置文件路径：

```YAML
name: "high_res_run"
optim:
  type: "adamw"
  lr: 0.0005
```

```shell
python script.py --config_path config.yaml
```

`
### PretrainConfig 类

```python title:scripts/pretrain.py
@dataclass

class PretrainConfig:

	# fmt: off
	# ModelConfig (`prismatic/conf/models.py`); override with --model.type `ModelRegistry.<MODEL>.model_id`
	
	model: ModelConfig = field(
		default_factory=
			ModelConfig.get_choice_class(ModelRegistry.PRISM_DINOSIGLIP_CONTROLLED_7B.model_id)
	)
	
	# DatasetConfig (`prismatic/conf/datasets.py`); override with --dataset.type `DatasetRegistry.<DATASET>.dataset_id`
	dataset: DatasetConfig = field(
		default_factory=
			DatasetConfig.get_choice_class(DatasetRegistry.LLAVA_V15.dataset_id)
	)
	
	  
	
	# Pretraining Stage in < align (projector-only) | finetune (projector + LLM) | full-finetune (all) >
	# ---
	stage: str = "finetune" # Pretraining Stage in < align | finetune >
	pretrained_checkpoint: Optional[Path] = None # Pretrained Checkpoint to Load (for `finetune`)
	# if None =>> will match on (run_dir / `align`)
	...
	def __post_init__(self) -> None:

	"""Set optimization parameters based on `stage` in {"align", "finetune"}."""
	
	if self.stage == "align":
		...
		
	elif self.stage.endswith("finetune"):
		...	
	
```

其实注释里说的很清楚了
1. 加载`ModelConfig`和`DatasetConfig`，以及一些其他配置
2. 根据训练阶段（`align`|`finetune`），来用`__post_init__`加载不同的配置

---

### pretrain函数

#### 大概逻辑：

1. **环境配置与初始化(The Setup)**
	- **`@draccus.wrap()`**: 这是一个装饰器，用于自动解析命令行参数并映射到 `PretrainConfig` 类型对象中。它保证了配置的严格类型检查。    
	- **分布式握手**:
	    - `overwatch` 在初始化时会自动处理 `torch.distributed`。
	    - `torch.cuda.set_device(...)` 将当前进程绑定到正确的 GPU 上，这是多卡训练不“打架”的关键。
	    - `torch.cuda.empty_cache()` 清理显存碎片，确保接下来的大模型加载有足够的连续空间。

2. **实验追踪与路径管理 (Logging & IO)**
	- **Run ID 生成**: 根据模型 ID、训练阶段（stage）和随机种子自动生成唯一的文件夹名称。
	- **配置备份**:
	    - `overwatch.is_rank_zero()` 确保只有“主进程”负责写文件。
	    - 将 `.yaml` 配置转存为 `.json`，方便后续实验对比和自动化脚本读取。

3. **模型组件加载 (Model Materialization)**
	- 这是最关键的“组装”阶段，代码采取了**模块化加载**策略：
		- **Vision Backbone**: 加载图像处理部分（如 DinoV2 或 SigLIP），此时通常在 CPU 上以全精度加载。
		- **LLM Backbone**: 加载语言模型部分（如 Llama-2），处理 Tokenizer 和特殊字符。
		- **get_vlm**: 将两者通过 **Projector（连接器）** 组装成完整的多模态模型。
		
4. **训练阶段控制 (Freeze & Load)**
	代码通过 `cfg.stage`（通常是 `align` 或 `finetune`）动态调整模型状态：
	- **`vlm.freeze_backbones(cfg.stage)`**:
	    - 如果是 `align` 阶段：冻结视觉和语言骨干，只练中间的 Projector。
	    - 如果是 `finetune` 阶段：解锁 LLM 进行联合训练。
	- **`vlm.load_from_checkpoint(...)`**: 根据当前阶段自动寻找上一个阶段的产物（例如 finetune 必须加载 align 后的权重）。

5. **训练策略与数据准备 (Strategy & Data)**
	- **Dataset & Collator**: 针对当前阶段加载数据。`collator` 负责将图片、文本提示（Prompt）和标签打包成 Batch。
	- **Train Strategy**: 这是计算的大脑。它配置了：
	    - **混合精度 (Mixed Precision)**: 加快速度，减少显存。
	    - **梯度检查点 (Gradient Checkpointing)**: 以计算时间换取显存空间。
	    - **学习率调度**: 处理 Warmup 和衰减策略。

6. **核心循环与收尾 (Execution & Cleanup)**
	- **Metrics**: 初始化分布式监控（如 Weights & Biases），只有主进程会向云端上报数据。
	- **`train_strategy.run_training(...)`**: 进入真实的训练循环（前向传播 -> 计算损失 -> 反向传播 -> 更新权重）。
	- **分布式资源释放**:
	    - `dist.barrier()`: 强制所有卡集合，确保大家都跑完了，防止某些卡提前退出导致死锁。
	    - `dist.destroy_process_group()`: 正常关灯下班，释放网络连接。



#### 具体训练函数`train_strategy.run_training(...)`


一般的模型训练流程都是差不多的，以prismatic-vlm为例子学习

##### 创建Sampler
```python
def run_training(
self,
dataset: Dataset,
collator: PaddedCollatorForLanguageModeling,
metrics: Metrics,
stage: str = "finetune",
batch_construction_strategy: str = "split-modality",
seed: int = 7,
) -> None:

"""Run the training loop for the given `dataset` and `collator`; log losses, results to `metrics`"""
if "finetune" in stage and batch_construction_strategy == "split-modality":
	...
	sampler = SplitModalitySampler(
		dataset,
		modality_lengths,
		global_batch_size=self.global_batch_size,
		num_replicas=overwatch.world_size(),
		rank=overwatch.rank(),
		seed=seed,
		drop_last=False,
	)

else:
	sampler = DistributedSampler(
		dataset,
		num_replicas=overwatch.world_size(),
		rank=overwatch.rank(),
		shuffle=True,
		seed=seed,
		drop_last=False,
	)
```

1. 为什么 `finetune` 需要 `SplitModalitySampler`？

在 OpenVLA 或 Prismatic 的微调阶段，训练集通常不再是单一的“图像-文本”对，而是包含多种模态的混合体（例如：纯文本对话、单图对话、多图指令等）。

**核心痛点：显存浪费与计算低效 (Padding)**
- **问题**：视觉特征（Visual Tokens）通常非常长（如 196 或 576 个 Token），而纯文本 Token 往往很短。
- **后果**：如果使用传统的 `DistributedSampler` 随机混合数据，一个 Batch 里可能既有“长图像流”又有“短文本流”。为了对齐，程序必须给短文本补上大量的 `[PAD]` Token。
- **影响**：这会导致 GPU 在计算大量的无意义 Padding，白白浪费显存和算力，极大降低训练吞吐量。
    

**`SplitModalitySampler` 的解决方案**

它会确保**同一个 Batch 内的样本属于同一种模态**（例如：这个 Batch 全是纯文本，下个 Batch 全是图像+文本）。

- **减少 Padding**：同模态的数据长度相近，填充最少。
- **提高吞吐量**：通过减少无效计算，单卡能跑更大的 Batch Size，训练速度显著提升。
- **梯度平滑**：避免不同模态的损失函数（Loss）在一个 Batch 内剧烈震荡。
    
---

2. 为什么其他阶段（如 `align`）只用 `DistributedSampler`？

在 `align` 阶段，数据集通常比较单一且规整。
- **数据同质化**：`align` 阶段（如 LLaVA-v1.5 558k）绝大部分数据都是“一张图 + 一句短描述”。所有样本的结构几乎一致，Token 长度分布均匀。
- **无需复杂分组**：既然大家都很像，使用标准的 `DistributedSampler` 进行随机打乱（Shuffle）就能获得很好的随机性，且不会产生严重的 Padding 浪费。
- **分布式同步**：`DistributedSampler` 负责将数据集均匀切分到不同显卡（Rank）上，确保每张卡看到的样本不重复。

[OpenVLA Sampler详解](/openvla-sampler)

##### 创建Dataloader
```python
# Create a DataLoader with the initialized sampler, per-device-bsz, and collator
dataloader = DataLoader(
	dataset,
	batch_size=self.per_device_batch_size,
	sampler=sampler,
	collate_fn=collator,
	num_workers=2,
	worker_init_fn=self.worker_init_fn,
)
```

##### 训练train

这段代码是整个训练流程的“心脏”部分。它实现了一个标准的**深度学习训练循环**，但包含了一些处理大模型（VLM）时非常关键的细节：**混合精度（AMP）**、**梯度累积（Gradient Accumulation）和动态进度监控**。

我们可以把这段逻辑拆解为四个关键层次：

---

1. 进度条的“欺骗”逻辑 (tqdm Setup)

在进入循环前，`tqdm`（进度条）需要知道总共要跑多少步：

- **逻辑判断**：
    - 如果没有 `max_steps`，总进度 = $Epochs \times (Batches / 累积步数)$。
    - 如果有 `max_steps`，总进度就直接等于这个预设的步数。
- **`disable=not overwatch.is_rank_zero()`**：这是一个非常重要的分布式细节。**只有 0 号卡负责打印进度条**，否则 8 张卡同时打印 8 个进度条会把终端刷爆。
    

---

2. 混合精度与前向传播 (The Forward Pass)

```python
with torch.autocast("cuda", dtype=self.mixed_precision_dtype, ...):
    output = self.vlm(...)
    loss = output.loss
```

- **混合精度 (AMP)**：大模型通常使用 `bfloat16` 训练。`autocast` 会自动处理哪些算子用 `float32`（保证精度），哪些用 `bf16/fp16`（提速并省显存）。
- **VLM 的契约**：模型内部已经把图像（pixel_values）和文字（input_ids）处理好了，直接返回一个 `loss`。

> [!NOTE] autocast
> `autocast` 是 PyTorch 提供的一个**上下文管理器**。它的核心逻辑是：**自动根据操作的类型，选择最合适的数值精度。**
>- **对于矩阵乘法（Linear/Conv 层）**：这些操作对精度没那么敏感，但计算量极大。`autocast` 会把它们转成 `bfloat16`（或者 `float16`），这样速度能快 2-4 倍，显存占用减半。
>- **对于求和、Softmax、Loss 计算**：这些操作需要极高的数值稳定性。`autocast` 会让它们保持在 `float32`，防止数值溢出或下溢。



> [!NOTE] **BF16 vs. FP16 的终极博弈：**
>- **FP16 (Half Precision)**：指数位短。它虽然精准，但很容易出现**数值溢出（Overflow）或下溢（Underflow）**。用它训练大模型时，你必须配合 `GradScaler`（缩放损失），否则 Loss 经常变成 `NaN`。
>- **BF16 (Brain Floating Point)**：它牺牲了一点精度（尾数位少），但**指数位和标准的 FP32 一模一样**。
    >	- **优势**：它能表示和 FP32 同样大范围的数值。在大模型训练中，你**不需要**复杂的 `GradScaler`，模型也不容易跑崩。
    >	- **门槛**：它需要较新的显卡（如 NVIDIA A100/H100 或 30/40 系列）。

    

---

3. 梯度累积：以时间换空间的魔法 (Gradient Accumulation)

代码通过以下三步实现：

1. **缩放损失**：`normalized_loss = loss / self.grad_accumulation_steps`。
2. **累积梯度**：`normalized_loss.backward()`。注意，此时**并没有更新模型参数**，只是把计算出的梯度“存”在 `.grad` 属性里。
3. **条件触发**：`if (train_idx + 1) % self.grad_accumulation_steps == 0:`。
    - 只有攒够了步数（比如 8 步），才会执行真正的 `optimizer.step()`。
    - **梯度裁剪 (`clip_grad_norm`)**：在更新前检查梯度是否过大（防止爆炸），在大模型训练中这是必选动作。
    - **重置**：`optimizer.zero_grad()` 清空攒下的梯度，开始下一轮累积。
        

---

4. 退出与保存机制

- **中间检查**：如果设置了 `max_steps`，一旦达到步数，**立刻保存并直接 `return`**（这就实现了我们之前说的“短路”）。
- **末尾保存**：如果按 Epoch 跑，则每跑完一个完整轮次保存一次。
- **`dist.barrier()`**：保存完后，所有显卡在这里“集合”，确保文件写完了大家再一起进行下一步。
    
总结如下：

|**环节**|**核心动作**|**意义**|
|---|---|---|
|**前向**|`autocast` (AMP)|显存减半，速度翻倍。|
|**反向**|`backward()`|计算梯度但不立即更新。|
|**累积**|`loss / steps`|模拟大 Batch 效果，适应小显存 GPU。|
|**更新**|`step()`|真正的进化时刻。|
|**退出**|`global_step >= max_steps`|无论 Epoch 剩多少，到点就下班。|



## Train-训练OpenVLA