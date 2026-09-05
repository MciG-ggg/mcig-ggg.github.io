---
title: MiniMind-O ncu 入门：从 320ms 到"510ms 全是空等"的取证路径
timestamp: 2026-09-05T22:00:00+08:00
tags:
  - AI
  - LLM
  - 性能优化
  - CUDA
  - Profiler
  - PyTorch
series: nanovllm-omni 开发手记
description: torchprofile 告诉我们账面算力优化到位、Kineto 告诉我们 wall-clock 还差 510ms，但"这 510ms 到底是谁在等谁"只有 ncu 能给硬证据。这篇记录在 RTX 3050 上用 Nsight Compute 抓 MiniMind-O decode loop 里 top 4 kernel 的全过程，以及怎么从那一堆百分比里读出"该上 CUDA Graph 了"。
toc: true
---

## 上一篇写到哪了

上一篇[《MiniMind-O torchprofile 入门》](MiniMind-O torchprofile 入门.md)把"账面算力"那一层补完了——9 处 monkey-patch 跑下来 MACs 该省的省了，剩下都是 0 MACs 的访存 / launch / control flow 优化。但账面算力和 wall-clock 之间还差一个 kernel 层的取证工具。

torchprofile 答不出 "kernel 在 GPU 里到底有没有空等"。这一步归 **Nsight Compute (ncu)**。

## ncu 是什么 / 不是什么

ncu 是 NVIDIA 官方的**单个 kernel 微架构分析器**。它在每次 kernel launch 时插入硬件 counter 采样（不修改 kernel 本身），跑完之后告诉你这个 kernel：

- **SM 占用率 (achieved occupancy)**：实际同时在跑的 warp 占 SM 能容纳的比例
- **内存吞吐 (memory throughput)**：实际带宽 vs 这块 GPU 理论带宽的百分比
- **计算吞吐 (compute throughput)**：FMA / ALU / tensor core 实际占比
- **SOL (Speed Of Light)**：上述指标的综合评分，每个 kernel 一个 0-100% 的数
- **stall reason**：warp 为什么没跑（memory wait、barrier、instruction issue 等）

它**不是**：

- **不是 timeline profiler**——你要的是"这次 launch 总共多少 ms"，用 `nsys` 或 `torch.profiler`，不是 ncu
- **不是端到端 profiler**——它一次只能盯一个 kernel，看完整个推理的所有 kernel 会爆数据量
- **不是优化建议器**——它告诉你数字，怎么改是你自己的判断

`nsys` 和 ncu 的分工：

| 工具 | 时间尺度 | 输出 |
|---|---|---|
| `nsys profile` | 整次推理 | timeline + 每次 launch 的 ms + API 调用 |
| `ncu --set full` | 单个 kernel | 该 kernel 的微架构指标、stall reason、SOL |
| `torch.profiler` | 整次推理 | 算子级的 ms、参数、显存 |

**先 `nsys` 找出慢的 kernel，再用 `ncu` 看那个 kernel 里发生了什么**。这是默认姿势。

## 装上它

ncu 不在 pip 里——它是 NVIDIA Nsight Compute CLI，跟 CUDA toolkit 一起分发（或独立装）：

```bash
# Linux
sudo apt install nsight-compute-2025.3.0

# macOS 不支持 ncu（CUDA toolkit 在 macOS 上不存在）——这是 ncu 在我们项目里最大的本地限制
# 验证
ncu --version
# 期望: NVIDIA (R) Nsight Compute version 2025.3.0 ...
```

**驱动版本也有下限**。RTX 3050 (Ampere) 需要 ≥ R535。检查：

```bash
nvidia-smi --query-gpu=driver_version --format=csv
# 期望: 535.x 或更高
```

我们项目是 macOS 开发 + Linux/CUDA GPU 跑的，所以 ncu 必须在 GPU 机器上跑。

## 怎么瞄准一个 kernel

最常用的姿势：`--kernel-name` 用 regex 过滤，`--launch-skip` 跳过前 N 次（warmup），`--launch-count` 只看后面 M 次。

```bash
ncu \
  --target-processes all \
  --kernel-name "regex:.*gemm.*|.*attention.*|.*norm.*" \
  --launch-skip 100 \
  --launch-count 5 \
  --section SpeedOfLight \
  --section Occupancy \
  --section MemoryWorkloadAnalysis \
  python -m nanovllm_omni.optim.bench profile
```

这条命令的意思是：

- 跑 `bench profile` 这个目标（你项目里的入口）
- 跳过前 100 次 launch（让模型做完 warmup 到稳态）
- 接下来 5 次匹配 regex 的 kernel 抓微架构指标
- 只输出 SpeedOfLight / Occupancy / MemoryWorkloadAnalysis 三组指标（默认会输出几十个 section，每个几百行，文件爆炸）

**全套指标用 `--set full`**，但首次跑一定要先 `--section` 几个关键 section，否则输出文件几 MB 起步，后面 grep 不到想要的数。

## 一次真实的 ncu 工作流

我们项目里用来抓 MiniMind-O decode loop 的入口是 `tools/bench_ncu_minimind.py`。它的核心代码是：

```python
import torch
from nanovllm_omni import Omni, SamplingParams

model = Omni(model_path="minimind-o")
prompt_ids = torch.tensor([[1, 2, 3, 4, 5, 6, 7, 8, 9]], device="cuda")

# warmup 20 步
for _ in range(20):
    model.generate(prompt_ids, SamplingParams(max_tokens=16))

# 抓 5 步
torch.cuda.cudart().cudaProfilerStart()
for _ in range(5):
    model.generate(prompt_ids, SamplingParams(max_tokens=16))
torch.cuda.cudart().cudaProfilerStop()
```

配合上面的 `ncu --launch-skip 100 --launch-count 5`，我们能稳抓到 5 次 decode step 的 top kernel。

跑完输出（简化版）：

```
  kernel                  | SOL% | Occupancy% | Memory% | Compute% | Stall
  ------------------------|------|------------|----------|----------|--------
  cutlass_h40xx_s1688gemm |  62  |    84      |   91     |   31     | ldst
  flash_fwd_kernel         |  58  |    77      |   74     |   45     | barrier
  vectorized_layer_norm   |  44  |    91      |   48     |    8     | short_scoreboard
  index_copy_kernel        |  31  |    53      |   29     |    2     | wait
```

注意：`SOL%` 是各项指标的**最大值**，不是平均值。一个 kernel SOL=62% 不代表它有 38% 的余量——它可能某一项是 91%，另一项是 31%，取的是瓶颈那一项。

## 怎么读这些数字

3 条经验：

**1. SOL<60% 多半是 launch-bound。** 如果一个 GEMM kernel SOL 只有 50%，但 Memory% 是 91%、Compute% 只有 31%——它被访存喂不饱。访存喂不饱通常是 launch 太频繁、新 kernel 来不及喂数据进来。**这种 kernel 单独优化没用，要的是把多个 launch 合并**——也就是 CUDA Graph 的典型用例。

**2. achieved_occupancy<50% 多半是寄存器压力。** SM 上的 warp 槽位没填满，原因是每个 thread 用了太多 register，编译器没法塞更多 warp 上来。**修法是 `--maxrregcount` 调小，或者重写 kernel 用更少寄存器**。但小模型上这种问题很少见——小 kernel 寄存器压力小。

**3. memory_throughput 占 roofline 比例对应访存瓶颈。** Roofline 模型里横轴是算术强度 (MACs/byte)，纵轴是 FLOPS。一个 memory-bound kernel 在 roofline 图上贴在斜线上（带宽限制），compute-bound 贴在水平线上（算力限制）。ncu 的 `--section MemoryWorkloadAnalysis` 里 `L1/TEX Hit Rate` 和 `L2 Hit Rate` 告诉你访存有没有命中。

我们 MiniMind-O decode loop 那次 ncu 跑出来的 top 4 kernel 形态是：**GEMM 是 compute-bound 但 SOL 60% 左右、attention 是 memory-bound SOL 60% 左右、norm 和 index_copy 都是小 kernel SOL 40-50%**。每一项单独看都没有大毛病，但**整体跑下来 320ms 里只有 28ms 是 kernel 在跑**——剩下 292ms 全在等 host 把下一个 launch 排上来。

这就是 ncu 给我们的硬证据：**不是"哪个 kernel 慢"，是"kernel 没在跑"。**

## 它帮我们回答了什么

ncu 在我们项目里只回答了一个核心问题，但这个问题的答案翻转了整个优化方向：

> **"320ms 里 kernel 实际工作时间占比多少？"**

回答是 6% 左右。剩下 94% 是 host 排队 + GPU 空等。**这不是"kernel 优化不够"，是"launch 优化不够"**。这个判断做完之后，CUDA Graph 才进入候选 list——否则你可能会继续在算子融合上耗时间，得到个 5% 的边际改善然后宣称"kernel 优化到顶了"。

回答不了的问题：

- "这次推理总共多少 ms" → 问 `nsys` 或 `torch.profiler`
- "这个 kernel 在哪个 GPU 上跑得快" → 自己 benchmark sweep
- "为什么我的 patch 没生效" → 不是 ncu 的题，先看 PyTorch eager 模式下 kernel 名变了没

## 踩坑清单

| # | 坑 | 症状 | 修法 |
|---|---|---|---|
| 1 | 没 `--launch-skip` | profile 抓到的是 cold start（第一次 launch，cache miss 严重） | 至少 skip 100 次 |
| 2 | 没 `--launch-count` | 整个推理里所有 launch 都 profile，输出 50MB+ | 默认就设个 count |
| 3 | `--set full` 一次拉完 | 输出文件几个 GB，grep 都 grep 不动 | 第一次跑只 `--section SpeedOfLight` + `Occupancy` |
| 4 | macOS 跑 | 报错 "CUDA not available" / 直接段错误 | ncu 不支持 macOS，必须 GPU 机器 |
| 5 | 多 stream profile | section 输出跨 stream 串了 | 加 `--cuda-graph-trace=node` 或在 Python 里显式用 side stream |
| 6 | profile 时有其他进程在用 GPU | 数字被隔壁 kernel 污染 | `nvidia-smi` 确认独享 |
| 7 | `--replay-mode kernel` 没开 | 默认 application replay 慢到哭 | 加 `--replay-mode kernel`（默认就是，但确认一下） |
| 8 | profile 时碰 RNG（dropout / random sampling） | 每次跑出来的数字漂移 | 关 dropout / 锁 seed |

第 1 和第 2 条最致命——没 skip + 没 count = 一次 profile 输出几 GB，编辑器卡死，还得从头跑。我们有过一次 23GB 输出文件的惨案。

## 它和 torchprofile / Kineto 的接力

把三层工具串起来用就是完整的优化三角：

```
torchprofile    账面算力有没有省    (MACs)
nsys / Kineto   端到端 ms 分布       (timeline)
ncu             单 kernel 微架构    (SOL / occupancy)
```

每一层都有自己的"放大镜倍数"，互相不替代：

- torchprofile 看"该不该省"
- ncu 看"GPU 里发生了什么"
- nsys 看"时间花在哪儿"

我们 nanovllm-omni 仓库的 `docs/perf/ncu-generate-kernels-2026-09-01.md`（74KB, 51 节）就是用这套三角搭起来的，里面 §6-§12 是 ncu 取证部分的源材料。

## 收尾

ncu 不是一个"优化工具"——它是一个**取证工具**。它的输出不是"哪里慢"或"怎么改"，而是"这个 kernel 在硬件上跑了什么"。这种证据在 launch-bound 这种"看起来都还行但端到端就是慢"的形态下是唯一能让你**不靠猜**地决定下一步动作的工具。

MiniMind-O 这边的故事是：torchprofile 说账面算力优化到位、Kineto 说端到端 wall-clock 还有 320ms，ncu 给出那句"320ms 里 kernel 工作只占 28ms"——**然后 CUDA Graph 才登场**。下一篇[《MiniMind-O CUDA Graph 优化实录》](MiniMind-O CUDA Graph优化实录.md)就是从这个 ncu 结论开始的。

下一篇计划写 SmolVLA 的接入——从文本 AR 跨到 VLA 视觉+动作，同样的 5 条逐一打勾 + 4 个障碍一一打掉，但场景完全换一套，到时候再说。