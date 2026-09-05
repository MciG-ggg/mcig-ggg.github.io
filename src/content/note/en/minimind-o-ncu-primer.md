---
title: "MiniMind-O ncu primer: the forensics trail from 320ms to 'all 510ms is idle waiting'"
timestamp: 2026-09-05T22:00:00+08:00
tags:
  - AI
  - LLM
  - Performance Optimization
  - CUDA
  - Profiler
  - PyTorch
series: nanovllm-omni Dev Notes
description: torch.profiler / Kineto told us there's a lot of launch overhead in the wall-clock, but only ncu can give hard evidence for "who exactly is waiting on whom for those 510ms". This post records the whole process of catching MiniMind-O's top-4 kernels in the decode loop with Nsight Compute on the RTX 3050, and how to read "it's time for CUDA Graph" out of that pile of percentages.
toc: true
---

## Where the last post left off

The previous post, [《MiniMind-O torch.profiler primer》](/en/note/minimind-o-torch-profiler-primer), filled in the timeline layer — it records CPU, CUDA runtime, GPU kernels, and custom stages, but just looking at the timeline still can't explain kernel hardware utilization.

torch.profiler / Kineto can't answer "did this kernel actually saturate the GPU". That step belongs to **Nsight Compute (ncu)**.

## What ncu is / isn't

ncu is NVIDIA's official **per-kernel microarchitecture profiler**. It inserts hardware counter sampling at every kernel launch (without modifying the kernel itself), and after the run tells you, for that kernel:

- **SM occupancy (achieved occupancy)**: the fraction of warp slots the SM can hold that are actually running in parallel
- **memory throughput**: actual bandwidth vs. this GPU's theoretical bandwidth, as a percentage
- **compute throughput**: the actual share of FMA / ALU / tensor core
- **SOL (Speed Of Light)**: a combined score of the above, one 0–100% number per kernel
- **stall reason**: why warps aren't running (memory wait, barrier, instruction issue, etc.)

It is **not**:

- **not a timeline profiler** — if you want "how many ms total did this launch take", use `nsys` or `torch.profiler` / Kineto, not ncu
- **not an end-to-end profiler** — it can only watch one kernel at a time; profiling every kernel of a full inference explodes the data volume
- **not an optimization advisor** — it tells you numbers; how to change things is your own judgment

The division of labor between `nsys` and ncu:

| Tool | Time scale | Output |
|---|---|---|
| `nsys profile` | a whole inference | timeline + ms per launch + API calls |
| `ncu --set full` | a single kernel | that kernel's microarchitecture metrics, stall reasons, SOL |
| `torch.profiler` | a whole inference | operator-level ms, parameters, memory |

**Use `nsys` to find the slow kernel, then use `ncu` to see what's happening inside that kernel.** That's the default posture.

## Installing it

ncu isn't on pip — it's the NVIDIA Nsight Compute CLI, shipped with the CUDA toolkit (or installable standalone):

```bash
# Linux
sudo apt install nsight-compute-2025.3.0

# macOS doesn't support ncu (the CUDA toolkit doesn't exist on macOS) — this is ncu's biggest local limitation in our project
# verify
ncu --version
# expected: NVIDIA (R) Nsight Compute version 2025.3.0 ...
```

**There's also a driver floor.** The RTX 3050 (Ampere) needs ≥ R535. Check:

```bash
nvidia-smi --query-gpu=driver_version --format=csv
# expected: 535.x or higher
```

Our project develops on macOS but runs on a Linux/CUDA GPU, so ncu has to run on the GPU machine.

## How to target one kernel

The most common posture: `--kernel-name` with a regex filter, `--launch-skip` to skip the first N launches (warmup), `--launch-count` to only look at the following M.

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

What this command means:

- run the `bench profile` target (the entry point in your project)
- skip the first 100 launches (let the model warm up to steady state)
- capture microarchitecture metrics for the next 5 launches matching the regex
- only output the SpeedOfLight / Occupancy / MemoryWorkloadAnalysis blocks (the default emits dozens of sections, hundreds of lines each — the file explodes)

For the **full metric set use `--set full`**, but on the first run you must pass just a few `--section`s, otherwise the output file is several MB at minimum and you can't grep the number you want afterward.

## A real ncu workflow

The entry point we use in the project to catch MiniMind-O's decode loop is `tools/bench_ncu_minimind.py`. Its core code:

```python
import torch
from nanovllm_omni import Omni, SamplingParams

model = Omni(model_path="minimind-o")
prompt_ids = torch.tensor([[1, 2, 3, 4, 5, 6, 7, 8, 9]], device="cuda")

# warmup 20 steps
for _ in range(20):
    model.generate(prompt_ids, SamplingParams(max_tokens=16))

# capture 5 steps
torch.cuda.cudart().cudaProfilerStart()
for _ in range(5):
    model.generate(prompt_ids, SamplingParams(max_tokens=16))
torch.cuda.cudart().cudaProfilerStop()
```

Combined with the `ncu --launch-skip 100 --launch-count 5` above, we can reliably catch 5 decode steps' top kernels.

Example output (simplified):

```
  kernel                  | SOL% | Occupancy% | Memory% | Compute% | Stall
  ------------------------|------|------------|----------|----------|--------
  cutlass_h40xx_s1688gemm |  62  |    84      |   91     |   31     | ldst
  flash_fwd_kernel         |  58  |    77      |   74     |   45     | barrier
  vectorized_layer_norm   |  44  |    91      |   48     |    8     | short_scoreboard
  index_copy_kernel        |  31  |    53      |   29     |    2     | wait
```

Note: `SOL%` is the **max** of the individual metrics, not the average. A kernel with SOL=62% doesn't mean it has 38% headroom — it could be 91% on one metric and 31% on another; what's taken is the bottleneck one.

## How to read these numbers

3 rules of thumb:

**1. SOL<60% is usually launch-bound.** If a GEMM kernel has SOL around 50%, but Memory% is 91% and Compute% is only 31% — it isn't being fed fast enough by memory access. Unfed memory access is usually because launches are too frequent and new kernels can't get data in fast enough. **Optimizing that kernel alone is useless; what you need is to merge many launches** — i.e., the classic CUDA Graph use case.

**2. achieved_occupancy<50% is usually register pressure.** The SM's warp slots aren't filled because each thread uses too many registers and the compiler can't fit more warps on. **Fix: shrink `--maxrregcount`, or rewrite the kernel to use fewer registers.** But this problem is rare on small models — small kernels have little register pressure.

**3. memory_throughput's share of the roofline corresponds to the memory bottleneck.** In the roofline model, the x-axis is arithmetic intensity (MACs/byte) and the y-axis is FLOPS. A memory-bound kernel sits on the diagonal line in the roofline chart (bandwidth-limited); a compute-bound one sits on the horizontal line (compute-limited). The `L1/TEX Hit Rate` and `L2 Hit Rate` in ncu's `--section MemoryWorkloadAnalysis` tell you whether memory accesses are hitting.

The top-4 kernel shape from our MiniMind-O decode-loop ncu run: **GEMM compute-bound at about 60% SOL, attention memory-bound at about 60% SOL, and norm / index_copy both small kernels at 40–50% SOL**. No single one of them looks seriously wrong — but **across the whole 320ms, only 28ms is kernels actually running** — the other 292ms is all waiting for the host to queue up the next launch.

That's the hard evidence ncu handed us: **it's not "which kernel is slow", it's "the kernels aren't running".**

## What it helped us answer

In our project ncu answered only one core question, but that answer flipped the whole optimization direction:

> **"Out of 320ms, what fraction is kernels actually working?"**

The answer is about 6%. The other 94% is host queueing + GPU idling. **This isn't "kernels not optimized enough", it's "launches not optimized enough".** Only after making that call did CUDA Graph enter the candidate list — otherwise you might keep burning time on operator fusion, get a marginal 5% improvement, and declare "kernel optimization has hit its ceiling".

Questions it can't answer:

- "how many ms total did this inference take" → ask `nsys` or `torch.profiler`
- "which GPU runs this kernel fastest" → benchmark sweep yourself
- "why didn't my patch take effect" → not ncu's business; first check whether the kernel name changed in PyTorch eager mode

## Pitfall checklist

| # | Pitfall | Symptom | Fix |
|---|---|---|---|
| 1 | no `--launch-skip` | profile catches cold start (first launch, heavy cache miss) | skip at least 100 |
| 2 | no `--launch-count` | profiles every launch of the whole inference, 50MB+ output | set a count by default |
| 3 | pulling `--set full` all at once | output file of several GB; grep can't even touch it | first run only `--section SpeedOfLight` + `Occupancy` |
| 4 | running on macOS | errors "CUDA not available" / segfaults | ncu doesn't support macOS; it must run on the GPU machine |
| 5 | profiling across multiple streams | section output gets tangled across streams | add `--cuda-graph-trace=node` or use an explicit side stream in Python |
| 6 | other processes using the GPU while profiling | numbers polluted by neighboring kernels | confirm exclusivity with `nvidia-smi` |
| 7 | `--replay-mode kernel` not enabled | default application replay is painfully slow | add `--replay-mode kernel` (it's the default, but confirm it) |
| 8 | touching RNG while profiling (dropout / random sampling) | numbers drift between runs | turn off dropout / lock the seed |

Pitfalls 1 and 2 are the most fatal — no skip + no count = one profile outputs several GB, the editor freezes, and you have to start over. We had a 23GB output-file disaster once.

## The handoff with torch.profiler / Kineto

Chaining the three layers together gives the complete optimization triangle:

```
torch.profiler / Kineto  end-to-end timeline    (CPU/CUDA/kernel)
nsys                     system-level timeline (API/process/thread)
ncu                      single-kernel microarch (SOL / occupancy)
```

Each layer has its own "magnification" and they don't replace each other:

- torch.profiler / Kineto see "where the time goes"
- nsys sees "how processes and CUDA APIs queue up"
- ncu sees "what happens inside a single kernel on the GPU"

Our nanovllm-omni repo's `docs/perf/ncu-generate-kernels-2026-09-01.md` (74KB, 51 sections) was built on exactly this triangle; §6–§12 are the raw material for the ncu forensics section.

## Wrap-up

ncu isn't an "optimization tool" — it's a **forensics tool**. Its output isn't "where it's slow" or "how to fix it", but "what this kernel actually did on the hardware". That kind of evidence, in the launch-bound shape where "everything looks fine but end-to-end is still slow", is the only thing that lets you decide the next move **without guessing**.

MiniMind-O's story: torch.profiler / Kineto said end-to-end wall-clock still sat at 320ms with ~17,000 launches piled in the timeline; ncu then delivered that line "only 28ms of the 320ms is kernel work" — **and only then did CUDA Graph step on stage**. The next post, [《MiniMind-O CUDA Graph optimization record》](/en/note/minimind-o-cuda-graph), starts exactly from that ncu conclusion.

A later post plans to cover SmolVLA integration — stepping over from text AR to VLA vision+action, ticking off the same 5 checks and knocking out the same 4 obstacles, but with a completely different scenario. We'll see then.