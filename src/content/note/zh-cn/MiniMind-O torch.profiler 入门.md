---
title: MiniMind-O torch.profiler 入门：从 320ms 里拆出 17000 次 launch
timestamp: 2026-09-04T22:00:00+08:00
tags:
  - AI
  - LLM
  - 性能优化
  - CUDA
  - PyTorch
  - Profiler
series: nanovllm-omni 开发手记
description: 这次真正用来分析 MiniMind-O 的不是第三方 torchprofile，而是 PyTorch 自带的 torch.profiler。它通过 Kineto 记录 CPU、CUDA kernel 和自定义阶段，最后导出一份可以在 Perfetto 里打开的 Chrome trace。
toc: true
---

## torch.profiler 和 Kineto 是什么关系

`torch.profiler` 是 PyTorch 提供的 Python API。它负责配置采集范围、活动类型、schedule 和输出方式；在 CUDA 场景下，底层采集引擎主要是 **Kineto**。

所以两者不是两个并列工具：

- `torch.profiler`：你在 Python 里调用的 API 入口
- Kineto：负责记录 CPU activity、CUDA runtime 和 GPU kernel 的后端
- Chrome trace：采集结果的一种 JSON 输出格式，可以交给 Perfetto 或 Chrome trace viewer 查看

它们和 ncu 的关系也不一样：`torch.profiler` / Kineto 看**一整次推理的时间线**，回答“时间花在哪里”；ncu 看**某一个 kernel 的硬件计数器**，回答“这个 kernel 在 GPU 里跑得怎么样”。

## 最小可用例子

项目里最小的采集方式是：

```python
import torch
from torch.profiler import ProfilerActivity, profile

with profile(
    activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA],
    record_shapes=False,
) as prof:
    with torch.no_grad():
        model.generate(prompt, sampling_params)

prof.export_chrome_trace("/tmp/minimind.trace.json")
```

这里有两个细节：

1. `ProfilerActivity.CPU` 记录 Python 到 CUDA runtime 的调用，`ProfilerActivity.CUDA` 记录 GPU kernel。
2. `torch.no_grad()` 是模型推理本身的设置，不能因为 profiler 会采集事件就省掉。否则会把 autograd 的计算也放进 trace，结果既慢又难读。

第一次跑时不要急着 profile。JIT 编译、CUDA kernel autotune 和 cuDNN benchmark 都会污染结果。先在 profiler 外 warmup，再采集稳定的一次：

```python
for _ in range(3):
    model.generate(prompt, sampling_params)

with profile(
    activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA],
) as prof:
    model.generate(prompt, sampling_params)

prof.export_chrome_trace("/tmp/minimind.trace.json")
```

## nanovllm-omni 里的真实用法

我们的 benchmark 包了一层：

```python
from torch.profiler import ProfilerActivity, profile, record_function
from torch.profiler import tensorboard_trace_handler

with (
    profile(
        activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA],
        record_shapes=False,
        on_trace_ready=tensorboard_trace_handler("/tmp/torch-trace"),
    ),
    record_function("bench_outer"),
):
    run_benchmark()
```

模型内部还用 `record_function` 标了几个阶段：`tokenize`、`generate`、`generate.step`、`decode` 和 `wav`。这一步很关键。没有自定义标记时，trace 里只有几千个 kernel 名字；有了阶段标记之后，才能问：

- `generate` 占了多少 wall-clock
- 16 个 autoregressive step 是否平均
- `mimi.decode` 是不是端到端瓶颈
- 某个阶段里面到底发射了多少 kernel

命令行入口是：

```bash
python -m nanovllm_omni.optim.bench trace-torch \
  --out /tmp/minimind.trace.json

python -m nanovllm_omni.optim.bench profile-detail \
  --out /tmp/minimind-profile
```

`trace-torch` 只导出 trace；`profile-detail` 还会调用 `parse_kineto_trace`，把 trace 解析成按阶段统计的 kernel 数、累计 kernel 时间和 top kernel 表。

## 怎么看 Chrome trace

生成 JSON 后可以打开：

```text
https://ui.perfetto.dev
```

把 `minimind.trace.json` 拖进去，先看三层：

1. **CPU track**：Python 调用、`cudaLaunchKernel`、`cudaStreamSynchronize` 排队情况。
2. **CUDA API track**：host 什么时候发起 kernel，什么时候在同步。
3. **GPU kernel track**：kernel 的开始时间、持续时间和相互之间的空洞。

我们在 MiniMind-O 上看到的不是一个“某个 kernel 慢”的故事，而是一条很长的细碎时间线：单次生成约 **17000 次 `cudaLaunchKernel`**、**344 次 `cudaStreamSynchronize`**，cutlass GEMM 累计约 **28ms**。端到端约 320ms，绝大多数时间不是 GEMM 在计算，而是 host 在排队、GPU 在等下一个 launch。

这就是 `torch.profiler` 最有价值的地方：它把“感觉有 launch overhead”变成了一张可以定位到阶段、runtime API 和 kernel 的时间线。

## 它能看什么，不能看什么

`torch.profiler` 主要能看：

- CPU / CUDA activity 的开始时间和持续时间
- 每个 operator 或 kernel 的调用次数
- 自定义阶段的 wall-clock
- kernel 名字、累计时间和调用次数
- 可选的输入 shape、Python stack 和 memory activity

`key_averages()` 可以先做一个粗汇总：

```python
for item in prof.key_averages().table(
    sort_by="self_cuda_time_total",
    row_limit=20,
).splitlines():
    print(item)
```

它不能直接告诉你：

- kernel 是否达到 SM / memory bandwidth 上限
- warp 为什么 stall
- occupancy、SOL 和 roofline 指标

这些要交给 ncu。也不要把 `self_cuda_time_total` 简单相加后当成端到端时间：GPU kernel 可能异步执行，多个 event 之间也可能重叠。端到端 wall-clock 要看自定义 stage 或完整 trace 的时间范围。

## 这次取证留下的 4 个坑

1. **warmup 放进 profiler**：JIT、autotune 和首次显存分配会污染 trace。先 warmup，再 profile。
2. **trace 一上来开 `record_shapes=True`**：动态生成场景很容易让 JSON 膨胀。只有排查 shape 问题时才打开。
3. **只看 operator 表，不看 timeline**：operator 汇总能告诉你谁累计耗时高，但看不到 host wait 和 kernel 之间的空洞。
4. **把 Kineto 当 ncu**：Kineto 记录执行事件；它不提供寄存器压力、occupancy 和 stall reason。

## 和 ncu 的接力

实际工作流是：

```text
torch.profiler / Kineto  →  整体时间线：时间花在哪儿
ncu                      →  单 kernel：硬件为什么没吃满
CUDA Graph               →  如果是 launch-bound：把大量 launch 打包
```

所以[《MiniMind-O ncu 入门》](/note/MiniMind-O%20ncu%20入门)接着处理的是 profiler 时间线暴露出来的问题，而不是重新做一遍同样的统计。

这次真正的结论很简单：先用 `torch.profiler` 确认问题在时间线上，再用 ncu 判断 kernel 本身，最后才决定要不要上 CUDA Graph。工具名字不能混，证据链也不能跳。