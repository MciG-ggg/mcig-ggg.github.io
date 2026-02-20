---
title: OpenVLA 训练源码学习
date: 2026-02-20
timestamp: 2026-02-20T09:24:54+08:00
slug: openvla
category: note
tags:
  - AI
  - EmbodiedAI
  - VLA
---

# OpenVLA 训练源码学习

OpenVLA 基于 Prismatic-vlm [GitHub - TRI-ML/prismatic-vlms: A flexible and efficient codebase for training visually-conditioned language models (VLMs)](https://github.com/TRI-ML/prismatic-vlms) 这个 VLM 项目

笔者之前对 python 接触较少，只会基本语法，这篇是 python 语法，代码逻辑以及理论知识的整理，可以先看下面的博客内容学习下一些 python 的特有语法。

```cardlink
url: https://www.laumy.tech/2334.html/python%E8%A1%A5%E4%B9%A0/
title: "python补习 - laumy的学习笔记"
description: "装饰器 函数装饰器 什么是装饰器 装饰器是python的一种高级语法，本质上是函数包装器，可以在不修改函数代码 […]"
host: www.laumy.tech
favicon: https://www.laumy.tech/wp-content/uploads/2024/11/cropped-IMG_6192-scaled-1-32x32.jpg
```


## Pretrain- 训练 Prismatic-vlm

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

### PretrainConfig 类
#### 理论流程

#### 实际代码
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
2. 根据训练阶段（`align`|`finetune`），来加载不同的配置

### pretrain函数

大概逻辑：
- 用`@draccus.wrap()`自动解析命令行参数，构建`PretrainConfig`实例
- 接受`PretrainConfig`，根据`PretrainConfig`注册一些参数
- 
#### 函数签名

```python
@draccus.wrap()
def pretrain(cfg: PretrainConfig) -> None:
```