---
title: MiniMind-O torchprofile 入门：把 320ms 拆给 9 个 patch 各算一笔账
timestamp: 2026-09-04T22:00:00+08:00
tags:
  - AI
  - LLM
  - 性能优化
  - Profiler
  - PyTorch
series: nanovllm-omni 开发手记
description: 上一篇 25 轮优化已经把端到端从 846ms 压到 320ms，但每一笔省下来的毫秒到底是哪个 patch 贡献的，没人能说清。torchprofile 是 Python 算子层的 MACs/参数量统计器，恰好能补上这一环——前提是 monkey-patch 完之后还得让它认得新绑定的 forward。
toc: true
---

## 上一篇写到哪了

上一篇[《把 MiniMind-O 从 846ms 压到 320ms》](MiniMind-O性能优化实录.md)最后留了一个口子：

> "kernel 优化到这一步已经饱和，剩下的几百毫秒可能是 launch overhead，但要把这条线追到底需要 ncu 证据。"

那句话其实藏着两个独立问题：

- 每个 patch 单独贡献了多少 ms（Python 算子层就能答）
- 320ms 里有几 ms 是在真干活、几 ms 是在空等（kernel 层才能答）

**前者归 torchprofile，后者归 ncu**。这篇先把前者写掉，下一篇[《MiniMind-O ncu 入门》](MiniMind-O ncu 入门.md)再去对付后者。

## torchprofile 是什么 / 不是什么

[torchprofile](https://github.com/zhijian-liu/torchprofile) 是一个**纯 Python 的算子层 profiler**，本质是 hook 到 `nn.Module.__call__` 上，递归遍历计算图，统计每个算子的 MACs（multiply-accumulate operations，等价于 FLOPs 的一半）和参数量。

它和另外两个常见 profiler 的分工是这样的：

| 工具 | 看什么 | 单位 | 适合答的问题 |
|---|---|---|---|
| `torch.profiler` / Kineto | **运行时**算子耗时 | μs / 次数 | 哪个 kernel 慢、GPU 在干嘛 |
| torchprofile | **算子静态**复杂度 | MACs / params | 这个模块理论上算多少东西 |
| ncu (Nsight Compute) | **单 kernel 微架构** | SOL% / occupancy | 这个 kernel 离硬件极限多远 |

**torchprofile 不能告诉你 wall-clock**——它数的是"乘加次数"，不是"实际跑了多久"。一个 0 MACs 的 `aten.index_copy_` 在它眼里是免费的，但在 GPU 上可能比一个高 MACs 的 GEMM 还慢。所以它是"账面算力"，不是"实测耗时"。

但这恰恰是它最有用的地方：**当你做完 monkey-patch 想问"这次融合到底省了多少算子"时，只有 torchprofile 能直接给你数字。**

## 装上它

```bash
pip install torchprofile
```

一行就完事。它没有 CUDA 依赖，不会带 torch 重装。验证：

```python
from torchprofile import profile_macs
print(profile_macs)  # <function profile_macs at 0x...>
```

## 最小例子：在 MiniMind-O 上跑一次

最直接的姿势：

```python
import torch
from torchprofile import profile_macs

model = Omni(model_path="minimind-o")._model  # 拿到内部的 HF 模型
model.eval()
input_ids = torch.tensor([[1, 2, 3, 4, 5]], device="cuda")
macs = profile_macs(model, args=(input_ids,))
print(f"{macs:,} MACs")
```

跑出来是个大整数，比如 `123,456,789`。它等于"这次前向理论上要做多少次乘加"。

**两个立刻会踩的坑：**

1. **必须包 `with torch.no_grad():`**。否则 autograd 在递归统计时会把每个算子都重放一遍梯度计算，profile 时间暴涨到分钟级，且显存可能爆。
2. **不要在 `profile_macs` 调用时让模型在训练模式**。某些 module（如 `Dropout`）的 MACs 在 train/eval 下不一样，会得到不一致的结果。

```python
model.eval()
with torch.no_grad():
    macs = profile_macs(model, args=(input_ids,))
```

## nanovllm-omni 怎么包它

裸 `profile_macs(model, args=...)` 在我们项目里有两个不爽的地方：

- 它会**重新跑一遍前向**，对生成场景来说要构造 prefill 输入
- 它不**区分 9 处 patch 各自的 MACs**，只给一个总数

我加了一个 `nanovllm_omni/optim/profiling.py`，提供 `profile_with_patches(model, prompt)` 上下文管理器，思路是：

1. 在 monkey-patch 之前先 profile 一次，记 baseline MACs
2. 逐个启用 patch，每启用一个再 profile 一次
3. 输出一个 `(patch_name, before_macs, after_macs, delta)` 的表

```python
from nanovllm_omni.optim.profiling import profile_with_patches

with profile_with_patches(model, prompt_ids) as report:
    for row in report:
        print(f"{row.name:<30s} {row.after_macs:>15,} -{row.delta_macs:>12,}")
```

跑出来的表长这样（数字来自上一篇 E1-E12 的实际 baseline + 启用顺序）：

| Patch | 启用后 MACs | Δ vs prev |
|---|---:|---:|
| baseline (KV buffer + SDPA) | 8,461,002,240 | — |
| + SDPA decode `is_causal=False` | 8,461,002,240 | 0 (mask 不算 MAC) |
| + Fused RMSNorm | 8,461,002,240 | 0 (norm 是访存非算力) |
| + Fused QKV | **5,640,668,160** | **−2,820,334,080 (−33.3%)** |
| + Fused RoPE | 5,640,668,160 | 0 (RoPE 也是访存) |
| + `rp==1.0` skip | 5,640,668,160 | 0 (penalty 是访存) |
| + model_input contiguous `copy_` | 5,640,668,160 | 0 (拷数据不增算) |

这张表是上一篇没贴出来的——它回答了一个当时让我困惑的问题："E5 RMSNorm 怎么会省 178ms？RMSNorm 又没什么 FLOPs。" 答案：**FLOPs 是 0，省的是访存和 launch**。torchprofile 不会告诉你这事，所以这张表必须跟 Kineto 对照看才有意义。

## 它帮我们回答了什么

回答得最好的 3 个问题：

1. **"QKV 融合真的省算子了吗？"** → 答：省了 33.3% MACs（3 个独立 matmul → 1 个），但 launch 数没省到一半，因为只有一个 kernel 了。这是上一篇 E6/E24 的微观证据。
2. **"RMSNorm 融合省了什么？"** → 答：MACs 0，访存和 launch 数都减半。**这反过来印证了 RMSNorm 是访存 bound 而不是算力 bound**——这一类洞察是 Kineto 看不清的，因为它数的是时间不是访问字节。
3. **"RoPE 融合 / repetition penalty skip 省了什么？"** → 答：都是 0 MACs，纯访存 / 纯 control flow 优化。**省下来的全是 launch overhead + 内存搬运**——这是为什么下一步必须上 ncu，因为 kernel 层已经饱和。

回答不了的：

- "这个 patch 帮端到端省了多少 ms？" → 问错工具了，看 Kineto
- "这个 kernel 离硬件极限多远？" → 问错工具了，看 ncu
- "我的 patch 在哪些输入 shape 下失效？" → 问错工具了，自己写 bench sweep

## 陷阱清单（按踩坑代价排序）

| # | 坑 | 症状 | 修法 |
|---|---|---|---|
| 1 | 没包 `torch.no_grad()` | profile 时间 ×10-100×、可能 OOM | 全程包 `with torch.no_grad():` |
| 2 | monkey-patch 后 model 仍是 `train()` | Dropout/BNL MACs 异常 | 强制 `model.eval()` |
| 3 | 嵌套 module 被重复计数 | MACs 比理论值多 2-3 倍 | `torchprofile` 默认会去重，但还是先检查 |
| 4 | `*args` 传错 shape | 报 shape mismatch 或悄悄 profile 了错的输入 | 显式构造一个最小输入 |
| 5 | 自定义 op 没注册到 hook 表 | 该算子被算成 0 MAC | 在 `torchprofile/profile.py` 里加一行 `MODULES_MAPPING` |
| 6 | profile 时还在 CUDA stream 上 | 跟正常跑出来的结果对不上 | 用 side stream profile，或 profile 时确保跟生产路径用同一 stream |

第 5 条是个大坑。我们的 `enable_fused_projections` 在模型里动态绑了一个新 `forward`，里面直接用了 `torch.addmm`——这个 op 在 torchprofile 的 `MODULES_MAPPING` 里有，所以正常。但我们另一个 patch `enable_fused_rope` 里用了 `torch.lerp`，第一次跑出来 MACs 比预期低，**因为 torchprofile 不认识 `lerp` 算乘加**。这种情况就只能 fallback 到手数 `profile_macs(model, ...)` + 自定义 hook。

## 它和 Kineto 必须配着看

单独看 torchprofile 容易被骗。"MACs 一样"不等于"wall-clock 一样"。

举个例子：上一篇的 E7 "Fused RoPE" 在 torchprofile 上 MACs 没变化，但在 Kineto 上 launch 数 -67%、单步省 8ms。**只看 torchprofile 会以为这是个无效 patch**。所以 nanovllm-omni 仓库里 benchmark 的固定姿势是：

```
torchprofile → 看账面算力有没有省
Kineto       → 看实测 ms 有没有省
ncu         → 看 GPU 在 kernel 里有没有空等
```

三层工具的边界很清楚，**不能拿一层当另一层用**。

## 收尾

torchprofile 是个"账面算力"工具——不算 wall-clock、不看硬件、不数访存，但它能告诉你在做完 monkey-patch 之后**账面 MACs 到底省没省**。这一类问题在端到端 benchmark 看不出来的 patch 里最值钱（典型如 RMSNorm、RoPE、index_copy 这种纯访存 / 纯 control flow 的优化）。

下一篇[《MiniMind-O ncu 入门》](MiniMind-O ncu 入门.md)切到 kernel 层：torchprofile 说"MACs 优化到位"之后，是 ncu 告诉我们"320ms 里有 510ms 全是空等"，然后才有 CUDA Graph 上场的位置。