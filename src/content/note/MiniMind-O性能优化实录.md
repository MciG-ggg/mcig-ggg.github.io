---
title: 把 MiniMind-O 从 846ms 压到 320ms：25 轮实验的性能优化实录
timestamp: 2026-08-31T19:34:00+08:00
tags:
  - AI
  - LLM
  - 性能优化
  - CUDA
  - PyTorch
series: nanovllm-omni 开发手记
description: 在只允许 monkey-patch、零新依赖、不改上游模型代码的约束下，一个人用 25 轮自动实验把 RTX 3050 上 MiniMind-O 的端到端延迟从 846ms 压到 320ms 的全过程——包括一个潜伏了 22 轮的最大真实 bug。
toc: true
---
## 缘起：为什么要在 4GB 显卡上抠这几百毫秒

在做 `nanovllm-omni` 的过程中，我在本地一台 **RTX 3050 4GB Laptop** 上跑 MiniMind-O（一个三阶段 omni 模型：thinker → talker → mimi 语音解码）做端到端音频生成。一次完整的 `Omni.generate` 流程是：

```
prompt → 16 token 逐 token 自回归 → mimi.decode → WAV 编码
```

最初这条链路要 **928ms** 上下，对于一个语音对话 demo 来说体感明显卡顿。我给项目定了一个目标：把端到端 wall-clock 压到 **≤ 500ms**。

但真正的约束不是目标，而是边界。三条硬规则：

- 不引入任何**新依赖**（只能用 torch 内置算子）
- 不复制、不修改**上游模型代码**（HF 远程模型，`AutoModelForCausalLM(trust_remote_code=True)` 加载）
- 保持**公开 API 不变**（`Omni / AsyncOmni / SamplingParams / OmniRequestOutput`）

也就是说：只能在外面**打补丁**（monkey-patch），不能动里面的螺丝。这个约束恰恰把这次优化的性质从"改代码"变成了"在外层缝合"——也最终铸成了一个很有意思的 bug。

## 先搭一把可信的尺子：测量优先

优化之前，先解决"我怎么知道改动有没有用"的问题。这一步是我在这次经历里收获最大的地方之一。

在一小块 GPU 上，裸测一次 `generate` 的偶然波动远大于优化本身的收益。我们最终采用的 benchmark 方案是：

- 6 个 prompt × 5 runs × `max_tokens=16`
- 取 **median**（不被个别偶发尖峰带偏）
- 每次改动前后跑 20+ 次连续测量建立置信区间

一个非常反直觉的发现是 **GPU 温度对测量结果的影响是毁灭性的**：

- 冷态 GPU（60°C，1725 MHz）：stdev 只有 7–11ms，测量很干净
- 热态 GPU（>80°C，1590 MHz）：absolute wall-clock 直接**翻倍到 1200ms**

同一次改动，冷态热态测出来能差好几倍。这让我养成了"跑 bench 前先确认温度/频率"的习惯——不然你优化了个寂寞，还以为是 regression。

教训写成一个原则：**任何 < 20ms 的改动在 7ms 的噪声地板里都不可信**。后面 E13、E22 这类"听起来有道理但收益在噪声内"的改动，全部果断 discard。

## 25 轮实验的全景

整个优化过程是在一轮轮"提出假设 → 做实验 → 测量 → 决定 keep/discard"的循环里推进的（autoresearch，25 轮）。完整轨迹如下：

| # | 描述 | 状态 | Median (ms) | Δ vs prev |
|---|---|---|---:|---:|
| baseline | KV buffer + SDPA | — | **846.4** | — |
| E1 | 移除 KV buffer，保留 SDPA | keep | 815.4 | −31 (−3.7%) |
| E3 | 删 `.clone()` 省拷贝 | discard | 1050 | +235（破坏正确性）|
| E4 | SDPA decode `is_causal=True` | keep | 803.1 | −12 (−5.1%) |
| E5 | **Fused RMSNorm** (`aten._fused_rms_norm`) | keep | **625.6** | **−178 (−26.1%)** |
| E6 | Fused QKV + gate-up 投影 | keep* | 467.8 | −158 (−44.7%) |
| E7 | Fused RoPE（消除 cat） | keep | ~478 | marginal |
| E8 | view/split 清理 | keep | ~478 | — |
| E10 | skip `rp==1.0` repetition penalty | keep | ~408 | −70 |
| E11 | model_input 单次 contiguous `copy_` | keep | ~397 | −11 |
| E12 | `torch.compile(dynamic=True)` on RoPE | keep | ~386 | −11 |
| E13 | RMSNorm shape 缓存 | keep（后 revert）| ~382 | −4 |
| E14 | `torch.compile(reduce-overhead)` RoPE | crash | — | CUDA Graph 错误 |
| E15 | 去掉 torch.compile RoPE | discard | ~395 | +13（确认 E12 有效）|
| E16 | `torch.compile` 整块 MiniMindBlock | keep (off) | ~473 | 默认关 |
| E17–E23 | 稳定性 / 确定性 / 跨 prompt 验证 | keep | 379–422 | — |
| **E24** | **修复 qkv_fusion dedupe bug** | **keep** | **320** | **−62 (−16%)** |
| E25 | E24 后重试 KV buffer | discard | 452 | +132（回归）|

> `*`：E6 标称 −44.7% 其实是被同一批合并进 baseline 的 RMSNorm / RoPE 改动贡献的。**QKV 融合的真实收益直到 E24 才显现**——因为在那之前它一直是坏的。

从 846ms 到 320ms，总计 **−62.2% wall-clock**。而这个数字里，最大的一笔"真实改进"，不是某个聪明的算子融合，而是一个 bug 的修复。

## 那 13 处 monkey-patch 是怎么缝上去的

所有优化都落在 `nanovllm_omni/models/minimind_omni/attention.py` + `bundle.py` + `generation.py`，全部通过**类 / 实例级别的 `forward = bound_method.__get__(self, cls)`** 完成，不碰远程模型一个字节。核心几处：

| Patch | 收益机理 |
|---|---|
| 预分配 `text_buffer` / `audio_buffer` | 消除生成循环内 `torch.cat` 的反复增长 |
| SDPA decode `is_causal=False` | decode 时 Q 长度 1 + 全量 KV past，`is_causal=True` 会让 mask 只 attend 到 K[0]，产出噪音音频 |
| Fused RMSNorm (`aten._fused_rms_norm`) | 把 `pow+mean+rsqrt+mul×2+float+type_as` 合并成单 kernel（E5，−178ms）|
| Fused QKV / gate-up 投影 | 3 个独立 matmul → 1 个 `qkv_proj`（E6/E24 的主角）|
| Fused RoPE（去 cat） | 利用 `cos/sin` 在 dim 上的重复性质，绕开 `rotate_half` 的 cat |
| skip `rp==1.0` repetition penalty | 默认 `rp=1.0` 时 entire unique+divide 是 no-op |
| model_input 单次 contiguous `copy_` | 9 个 strided write → 1 个 view+`copy_` |
| `torch.compile(dynamic=True)` on RoPE | Inductor 把 RoPE 的 ~6 个 elementwise kernel 融合成 ~2 |

这些缝合的微观收益单个都不大（ms 级），但叠起来就是质变——而且零成本：没有新依赖，没有新模型权重，只有几段绑定到类/实例的 `forward`。

## E24：那个潜伏了 22 轮的真凶

这是全过程的戏剧顶点，也是最大的工程教训。

`enable_fused_projections` 的本意是对每个 Attention/MLP 做一次 QKV / gate-up 融合 patch。去重逻辑是这样写的：

```python
seen_attn.add(cls)   # 按“类”去重
seen_mlp.add(cls)
```

但 `qkv_proj` / `gate_up_proj` 是**实例属性**，不是类属性。于是灾难性地：

> **12 个 Attention 里只有第 1 个（thinker.0）被 patch 了，其余 11 个 layer 继续跑 3 个独立 matmul。**

从 E6 开始，我一度以为"Fused QKV"已经生效了 22 轮。直到某次 profiling 发现 11/12 的层压根没融合，修掉这个去重 bug 之后——**median 直接从 382ms 掉到 320ms，−62ms，是 5.4× stdev 的显著改进**。

这一课我刻在脑子里：

> **class-level 的 `seen` 去重在 per-instance patch 场景下就是 bug。看到 `seen.add(...)`，先问一句：这个 patch 是打在类上还是实例上？**

## 撞墙记录：CUDA Graph 的边界

不是每条路都走得通，这块记录了几个"反直觉"的失败：

- `torch.compile(reduce-overhead)` 包 RoPE → **crash**：与 KV cache 后续的 `torch.cat` 冲突（"accessing tensor output of CUDAGraphs that has been overwritten"）。
- `torch.compile` 整块 MiniMindBlock.forward → **+91ms regression**：Inductor 的 launch overhead × 16 层 ≈ 700ms 远超收益。
- 全栈 CUDA Graph capture：理论收益 50–150ms，但需要把 KV cache 静态化 + monkey-patch `model.model.forward` 的 `start_pos` 计算，范围太大、revert 复杂、影响所有下游用户。**未实施**——收益不够 cover 风险。

这些"不做什么"和"做什么"一样重要：在小模型 + 小批量上，`torch.compile` 的固定开销往往吃光收益，而手写 operator fusion 反而稳赚。

## 最终结果与护拦

在 `11c3c13` 上锁定的最终状态：

```text
benchmark (6 prompt × 5 runs × max_tokens=16):
  median:    320 ms
  mean:      323 ms
  stdev:     11.6 ms
  min:       305 ms
  max:       343 ms
  <500ms:    20/20 (100%)
  
VRAM:        493 MB
```

并且验证充分：

- **Determinism**：5× 同一 (prompt, seed=42) → 同一 WAV MD5，**bit-exact**
- **Cross-prompt**：12/12 异质 prompt 完整生成 8 frames，无截断
- **Public API**：`Omni / AsyncOmni / SamplingParams / OmniRequestOutput` 全部可导入，49 个非 smoke 测试通过
- **Wide-prompt**：median 422ms（bench prompt 偏短，跨域 prompt 下仍远低于 500ms 目标）

整个项目侧护栏也立住了：不 vendor 模型副本、不引入新依赖、不改 HF 远程模型代码、不破坏公开 API、不改变生成内容（位级）。

## 复盘：这次经历教会我的三件事

1. **先造一把可信的尺子，再谈优化。** GPU 温度能把测量漂移放大一个数量级；在没搞清噪声地板之前，任何 <20ms 的"优化"都可能是自我安慰。中位数 + 多次测量 + 温度监控，是我用真金白银换来的习惯。

2. **最大的收益往往不是新技巧，而是把已有的技巧真正做对。** 一个 `seen.add(cls)` 还是 `seen.add(self)` 的差别，等于 22 轮实验、11 个 layer、−62ms。别急着学新招，先检查老招有没有真的在整个对象集合上生效。

3. **约束不是敌人，是老师。** "只能 monkey-patch + 只用 torch 内置算子"这条约束，逼着我按算子语义去融合，而不是往项目里塞一个 CUDA kernel 或者复制一份 vendor 模型。最终实现的优化栈**零依赖、零 vendor、位级无副作用**——这比"快"更难，也更值钱。

完整优化栈与复现命令在 nanovllm-omni 的 `docs/perf/minimind-omni-under-500ms.md`，本地 `python -m nanovllm_omni.optim.bench time` 即可复现。