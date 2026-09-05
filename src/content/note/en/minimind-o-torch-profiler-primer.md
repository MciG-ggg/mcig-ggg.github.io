---
title: "MiniMind-O torch.profiler primer: decomposing 17,000 launches out of 320ms"
timestamp: 2026-09-04T22:00:00+08:00
tags:
  - AI
  - LLM
  - Performance Optimization
  - CUDA
  - PyTorch
  - Profiler
series: nanovllm-omni Dev Notes
description: What actually analyzed MiniMind-O this time wasn't the third-party torchprofile — it was PyTorch's built-in torch.profiler. It records CPU, CUDA kernel, and custom-stage activity through Kineto, and finally exports a Chrome trace you can open in Perfetto.
toc: true
---

## Get the name straight first

The previous post, [《Cutting MiniMind-O from 846ms to 320ms》](/en/note/minimind-o-from-846ms-to-320ms), ended noting that a lot of launch overhead remained after optimization and needed more evidence. One correction of names is in order here: **the project actually uses PyTorch's built-in `torch.profiler`, not the third-party `torchprofile`.**

I had mixed up the two similarly-named projects, which drifted the article's direction. `torchprofile` is a separate MACs-statistics library; the repo's code never imports it. What's actually used is:

```python
from torch.profiler import ProfilerActivity, profile, record_function
```

## How torch.profiler relates to Kineto

`torch.profiler` is the Python API PyTorch provides. It's responsible for configuring the capture scope, activity types, schedule, and output method; in the CUDA case, the underlying capture engine is mostly **Kineto**.

So they're not two parallel tools:

- `torch.profiler`: the API entry point you call from Python
- Kineto: the backend that records CPU activity, CUDA runtime, and GPU kernels
- Chrome trace: a JSON output format of the capture results, viewable in Perfetto or the Chrome trace viewer

Their relation to ncu is also different: `torch.profiler` / Kineto look at the timeline of **an entire inference run**, answering "where did the time go"; ncu looks at the hardware counters of **one kernel**, answering "how well did this kernel run on the GPU".

## Minimal usable example

The project's minimal capture approach:

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

Two details here:

1. `ProfilerActivity.CPU` records the Python-to-CUDA-runtime calls; `ProfilerActivity.CUDA` records GPU kernels.
2. `torch.no_grad()` is a setting for the inference itself; you can't skip it just because the profiler collects events. Otherwise autograd compute also lands in the trace, making it both slow and hard to read.

Don't profile on the first run. JIT compilation, CUDA kernel autotune, and cuDNN benchmark all pollute the results. Warm up outside the profiler first, then capture a stable pass:

```python
for _ in range(3):
    model.generate(prompt, sampling_params)

with profile(
    activities=[ProfilerActivity.CPU, ProfilerActivity.CUDA],
) as prof:
    model.generate(prompt, sampling_params)

prof.export_chrome_trace("/tmp/minimind.trace.json")
```

## The real usage inside nanovllm-omni

Our benchmark wraps it:

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

Inside the model, several stages are marked with `record_function`: `tokenize`, `generate`, `generate.step`, `decode`, and `wav`. This step is crucial. Without custom markers, the trace is just thousands of kernel names; with stage markers, you can now ask:

- how much wall-clock does `generate` take
- whether the 16 autoregressive steps are even
- whether `mimi.decode` is the end-to-end bottleneck
- how many kernels actually get launched inside a given stage

The CLI entry points are:

```bash
python -m nanovllm_omni.optim.bench trace-torch \
  --out /tmp/minimind.trace.json

python -m nanovllm_omni.optim.bench profile-detail \
  --out /tmp/minimind-profile
```

`trace-torch` only exports the trace; `profile-detail` additionally calls `parse_kineto_trace`, parsing the trace into per-stage kernel counts, cumulative kernel time, and a top-kernel table.

## How to read the Chrome trace

Once the JSON is generated, open:

```text
https://ui.perfetto.dev
```

Drag `minimind.trace.json` in, and look at three layers first:

1. **CPU track**: Python calls, `cudaLaunchKernel`, `cudaStreamSynchronize` queueing.
2. **CUDA API track**: when the host launches kernels, and when it's synchronizing.
3. **GPU kernel track**: kernel start times, durations, and the gaps between them.

What we saw on MiniMind-O wasn't a "one kernel is slow" story, but one long, fragmented timeline: a single generation produces roughly **17,000 `cudaLaunchKernel`** calls and **344 `cudaStreamSynchronize`** calls, with cutlass GEMMs totaling about **28ms**. End-to-end is around 320ms, and the vast majority of the time isn't GEMM computing — it's the host queueing and the GPU waiting for the next launch.

That's where `torch.profiler` is most valuable: it turns "it feels like launch overhead" into a timeline you can localize down to stages, runtime APIs, and kernels.

## What it can see, and what it can't

`torch.profiler` can mainly see:

- start times and durations of CPU / CUDA activity
- call counts per operator or kernel
- wall-clock of custom stages
- kernel names, cumulative time, and call counts
- optionally input shapes, Python stacks, and memory activity

`key_averages()` gives you a coarse summary first:

```python
for item in prof.key_averages().table(
    sort_by="self_cuda_time_total",
    row_limit=20,
).splitlines():
    print(item)
```

It can't directly tell you:

- whether a kernel is hitting SM / memory-bandwidth limits
- why warps stall
- occupancy, SOL, and roofline metrics

Those belong to ncu. And don't just sum up `self_cuda_time_total` and treat it as end-to-end time: GPU kernels can execute asynchronously, and multiple events can overlap. End-to-end wall-clock should come from the custom stages or the full trace's time range.

## 4 pitfalls this forensics pass left behind

1. **Putting warmup inside the profiler**: JIT, autotune, and first-time memory allocation pollute the trace. Warm up first, then profile.
2. **Turning on `record_shapes=True` right away**: in dynamic-generation scenarios the JSON inflates easily. Only enable it when debugging shape issues.
3. **Looking only at the operator table, not the timeline**: operator summaries tell you who has high cumulative time, but you can't see host waits or the gaps between kernels.
4. **Treating Kineto as ncu**: Kineto records execution events; it doesn't provide register pressure, occupancy, or stall reasons.

## The handoff to ncu

The actual workflow is:

```text
torch.profiler / Kineto  →  overall timeline: where the time goes
ncu                      →  single kernel: why the hardware isn't saturated
CUDA Graph               →  if launch-bound: pack the many launches
```

So [《MiniMind-O ncu primer》](/en/note/minimind-o-ncu-primer) picks up the problems the profiler timeline exposed, rather than re-doing the same statistics from scratch.

The real conclusion of this one is simple: first use `torch.profiler` to confirm the problem lives on the timeline, then use ncu to judge the kernel itself, and only then decide whether to reach for CUDA Graph. Don't confuse the tool names, and don't skip a link in the evidence chain.