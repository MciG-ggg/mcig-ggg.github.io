---
title: 把 SmolVLA 接入 nanovllm-omni：从 LeRobot 的 action chunk 到 Omni().generate(...)
timestamp: 2026-09-03T23:30:00+08:00
tags:
  - AI
  - VLA
  - Robotics
  - LeRobot
  - 工程
series: nanovllm-omni 开发手记
description: 前两篇把 MiniMind-O 端到端从 846ms 压到 42ms，这篇换轨道——把 4 个模型族里唯一没接入 engine 的 SmolVLA 接上 Omni()。记录薄壳封装策略在 VLA 上的验证、两条调用路径（synthetic demo + LIBERO eval）的设计选择、ActionArtifact 输出契约，以及为什么这次仍然不做 CUDA Graph。
toc: true
---

## 上一篇写到哪了

前两篇[《把 MiniMind-O 从 846ms 压到 320ms》](/note/MiniMind-O%E6%80%A7%E8%83%BD%E4%BC%98%E5%8C%96%E5%AE%9E%E5%BD%95)和[《把 MiniMind-O 再压 7.5 倍》](/note/MiniMind-O%20CUDA%20Graph%E4%BC%98%E5%8C%96%E5%AE%9E%E5%BD%95)讲的都是 MiniMind-O 这一族。但仓库里其实注册了 4 个模型族：`minimind_omni` / `smolvlm` / `sd_turbo` / `smolvla`。前三族都能通过 `Omni(...).generate(...)` 走完整栈，就剩 `smolvla`——前文[§51.2 跨族扫描](https://github.com/mcig-ggg/nanovllm-omni/blob/main/docs/perf/ncu-generate-kernels-2026-09-01.md)里把它标成"未来候选"。这一篇就是兑现那个 future candidate。

## 为什么要接 VLA

VLA（Vision-Language-Action）是过去两年 robotics 圈最热的范式。LLM 当大脑、视觉编码器当眼睛、action head 当手——一个模型直接从图像+指令吐动作序列。`Omni(...)` 的设计原本就奔着「统一多种模态」去的，缺 VLA 这一族就不完整。

SmolVLA 是 HuggingFace LeRobot 团队训的小尺寸 VLA：SigLIP（视觉）+ SmolVLM（语言）+ 一个 action expert，**一个 checkpoint 里同时塞了三个组件**。这跟 MiniMind-O 的「thinker + talker + code2wav 三 stage」不一样——SmolVLA 没法在推理阶段拆 stage，因为三组件之间有 cross-attention。所以 `nanovllm_omni/models/smolvla/pipeline.py:17-30` 是**单 stage**，而不是多 stage：

```python
stages=(
    StageConfig(
        stage_id=0, name="vla",
        kind=StageExecutionType.LLM_GENERATION,
        factory="nanovllm_omni.models.smolvla.stage:_vla_stage",
        process_input=None, input_sources=(),
        is_terminal=True, final_output_type="actions",
    ),
),
```

`final_output_type="actions"` 是关键——它跟 MiniMind-O 的 `"audio"`、SmolVLM 的 `"text"`、SD-Turbo 的 `"image"` 并列，是 OmniRequestOutput multimodal_output 的四种合法 key 之一（pipeline 拓扑见 `nanovllm_omni/models/smolvla/pipeline.py:26-35`）。`Omni` 公共 API 不动，新加一族 = 加一个新的 `final_output_type` + 配套的输出契约（`ActionArtifact`，见下文）。

## 薄壳策略在 VLA 上的验证

按 §51.3「深入 forward vs 薄壳封装」的策略——非自家训的模型走薄壳路线——SmolVLA 是 HuggingFaceVLA / LeRobot 训的，所以走薄壳。具体动作：

1. **不重写 forward**——`SmolVLAPolicy.select_action` / `predict_action_chunk` 是 LeRobot 训出来的，里面带 50 步 AR chunk 队列、cross-attention、proprioception 处理，全部自家实现。我们**只调用**，不碰。
2. **不 vendor**——`lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy` 走 PyPI 的 `lerobot`，导入路径靠 `nanovllm_omni/models/smolvla/stage.py:39-56` 的双 import 兼容（新旧 namespace）：

   ```python
   try:
       from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
   except ImportError:
       from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
   ```
3. **不 fork**——`models/smolvla/stage.py` 的 269 行全是**适配 LeRobot 接口到 Omni 调用契约**的薄代码，没有任何 LeRobot 自身的逻辑被复制。

这跟 §51.3 说的「sglang / vllm-omni 也走集中 patch，nanovllm-omni 的差异化是 vendor=0 + fork=0 + subclass=0」是同一个原则在 VLA 上的兑现。

## 两条调用路径

VLA 比文本生成复杂——它要的不是「下一个 token」，是「接下来 50 步要执行的动作」。这决定了 SmolVLA 有两条调用路径，stage 工厂在 `stage.py:239-264` 的 `vla_forward` 里分支：

### 路径 A：synthetic demo（默认）

`SamplingParams.extra["image"]`（HWC uint8 RGB）+ `extra["wrist_image"]`（可选，第二相机）+ `extra["state"]`（机器人本体感知）传进来。`_obs_batch`（`stage.py:148`）把图像转 NCHW、把 state 装进 batch、组装 lerobot 期望的 `observation.images.image` / `observation.state` / `task` 键。

走 `policy.predict_action_chunk(batch)`：一次性吐整块动作（`[chunk_size, action_dim]`；本仓库测试 mock 用的是 chunk_size=50、action_dim=7，对应 LIBERO 的 7-DoF 末端执行器维度）。

这条路径的好处是**不用装 LIBERO 环境**，单元测试 `tests/test_smolvla.py::test_omni_generate_returns_actions` 就是用 `monkeypatch` 替换 policy 跑通的（numpy + torch 即可，无需 lerobot）。

### 路径 B：LIBERO eval（精确复刻官方评估链）

`SamplingParams.extra["libero_obs"]` 存在时分支走 `_libero_forward`（`stage.py:213-237`），这条路径**严格复刻 lerobot/scripts/lerobot_eval.py ~L268-300 的处理顺序**。这一步至关重要——LIBERO 是 SmolVLA 的官方 benchmark，跑分要 100% 复现原版处理链才能对得上 leaderboard。

顺序是：
```
preprocess_observation(obs)  # 改名 unknown keys → observation.*
→ 注入 task（必须后注入，否则被改名成 observation.task 破坏 tokenizer）
→ env_preprocessor  # flip + quat->axis-angle
→ policy.preprocessor
→ policy.select_action  # 50 步 chunk 队列
→ policy.postprocessor
→ env_postprocessor
```

`stage.py:220-223` 的 in-function 注释专门强调「preprocess 必须先于 task 注入」——这是从 LeRobot 评测代码里学到的坑，跳过顺序会直接打乱 tokenizer 查表。

`_env_processors`（`stage.py:204`）用 `task_suite` 字符串做缓存键，避免每次 `forward` 都重建 lerobot 的 `LiberoEnvConfig.get_env_processors()`。

## 第 N 条接线：pipeline.yaml + registry 自动注册

跟 sd_turbo / smolvlm 完全一样的接线套路——这部分基本是从 MiniMind-O 那次抄过来的：

1. **`nanovllm_omni/models/smolvla/pipeline.py`**：声明 `PipelineConfig`，注册到 `OMNI_PIPELINES`。三个 `registration_handles`（`smolvla` / `HuggingFaceVLA/smolvla_libero` / `lerobot/smolvla_base`）+ 一个 `hf_architectures=("SmolVLAPolicy",)` 作为 Layer-6 disambiguator。
2. **`nanovllm_omni/deploy/smolvla.yaml`**：单 stage、`default_sampling_params: {}`（VLA 没"采样"概念——动作是确定性的，除非做 ensemble；deploy 留空给 caller override）。
3. **`nanovllm_omni/config/registry.py:265-276`** 的 `_load_builtin_pipelines()` 自动 import 四个 pipeline 模块做 side-effect 注册。`SmolVLA` 跟其他三族一起进 `OMNI_PIPELINES`，用户**不直接 import**。

路由兜底有 6 层 cascade（`entrypoints/base.py` 的 `try_infer_model_type`），smolvla 命中第 5 层（路径 basename substring）和第 6 层（`hf_architectures`）都能识别。`tests/test_smolvla.py::test_local_dir_basename_falls_back_to_registered_key` 验证空 config.json 也能从 `/path/to/smolvla_libero` 这种路径猜到——这是从 MiniMind-O 那次学到的兜底。

## 输出契约：ActionArtifact

VLA 输出不是字符串也不是图片，是**动作张量**——`[chunk_size, action_dim]` 的 numpy 数组（或 torch tensor）。`nanovllm_omni/outputs.py:365` 定义 `ActionArtifact`：

```python
@dataclass(frozen=True)
class ActionArtifact:
    array: Any            # 2-D [chunk_size, action_dim]
    action_dim: int
    chunk_size: int
    dtype: str            # str(getattr(array, "dtype", ""))

    @classmethod
    def from_array(cls, array: Any) -> "ActionArtifact":
        # 自动从 array.shape 抽 chunk_size/action_dim,验证 2-D
        ...
```

`__post_init__` 跑 shape 验证——`chunk_size` 必须等于 `array.shape[0]`、`action_dim` 必须等于 `array.shape[1]`。这把「VLA 输出必须是 2-D」的契约钉死在类型层。

stage 工厂在 `_to_action_artifact`（`stage.py:181-195`）里把 LeRobot 返回的 torch tensor / numpy array 统一包成 `ActionArtifact`，然后塞进 `OmniRequestOutput.multimodal_output["actions"]`。`tests/test_smolvla.py::test_from_pipeline_actions_key` 验证 `OmniRequestOutput.from_pipeline(artifact, final_output_type="actions")` 正确路由到 `"actions"` key。

这条契约的另一个作用：**让 Omni 公共 API 不动**。`Omni.generate()` 的 `multimodal_output` 已经是 `dict[str, Any]`，加一个 `"actions"` key 是 additive change，不需要改任何已存在的 caller。这跟前面 4 族接线的原则一致。

## 为什么这次没顺手做 CUDA Graph

`§51.2` 把 SmolVLA 标成"未来候选"。这次接入也确认了——按 §51.2 跨族扫描表的判定（5 个条件里只有条件 5 接近过、且未实测，其余 4 个全是"取决于 LeRobot 内部 forward"），SmolVLA 的形态跟 CUDA Graph 的判定框不匹配：

- 条件 1（launch-bound）：没测，但 LeRobot 的 SmolVLAPolicy 内部是 50 步 AR chunk 队列，每步跑 SigLIP + SmolVLM + action expert——单步工作量远大于 MiniMind-O 的单层 transformer。launch overhead 占比未知。
- 条件 2（shape 固定）：select_action 内部 chunk 队列的 shape 是相对稳定的（每 chunk 50 步 AR），但 50 步内部是动态 AR，shape 逐 token 漂移。
- 条件 3-4（地址稳定 / 分支稳定）：LeRobot 内部 forward 不在我们控制范围，没法保证预分配 buffer 兼容。
- 条件 5（re-capture 摊平）：50 步 chunk 队列内部如果每步都 re-capture，成本摊不平。

要做的话，第一件事**不是 capture**——是把 `enable_fixed_kv_buffer` 那套预分配 buffer + `_kv_pos` 光标的基础设施**复刻进 LeRobot 的 SmolVLAPolicy 内部**。工程量跟当初给 MiniMind-O 做 CUDA Graph 那次相当（参考 [《把 MiniMind-O 再压 7.5 倍》](/note/MiniMind-O%20CUDA%20Graph%E4%BC%98%E5%8C%96%E5%AE%9E%E5%BD%95)），是个独立 session 的事，不是这次「接入」的范围。

## 验证覆盖

`tests/test_smolvla.py` 12 个测试，覆盖：

- 注册表层：`resolve_pipeline_config("smolvla")` / `resolve_pipeline_config("HuggingFaceVLA/smolvla_libero")` 都返回正确 `PipelineConfig`
- 输出契约：`ActionArtifact.from_array` + `OmniRequestOutput.from_pipeline(artifact, final_output_type="actions")` 走通
- 缺依赖兜底：把 `_import_smouvla_policy` 换成抛 ImportError → 抛清晰的"lerobot is required for SmolVLA"
- 缺输入兜底：缺 `extra["image"]` → 抛 `ValueError` 提示正确路径
- `Omni().generate()` 端到端：mock 一个 `_Policy.predict_action_chunk` → 完整栈 `PipelineRunner → stage bundle → vla_forward → _to_action_artifact → OmniRequestOutput` 走通，输出 `ActionArtifact(chunk_size=50, action_dim=7)`
- 6 层路由 cascade：L1 / L3 / L5 / L6 都覆盖
- image bytes 兜底（TK-017）：base64 → PIL → NCHW tensor

没做 GPU e2e——需要真实 LeRobot checkpoint + CUDA，跑通需要 WSL 3050（受限于 RTX 3050 4GB 的 VRAM）。这条等下次 session。

## 收尾

这次接入没有性能数字，没有 CUDA Graph，没有 kernel 调优——只是把 4 族里最后一族接进 `Omni()` 公共 API。接入本身不是优化，但没接入就谈不上优化。

跟 MiniMind-O 那两次的关系：
- 第一篇（25 轮 monkey-patch）讲的是「**深入 forward**」的工程价值——对自己训的模型做微观优化；
- 第二篇（CUDA Graph 7.51×）讲的是「**launch-bound 判题**」的判定方法——5 条件 + 跨族扫描；
- 这篇讲的是「**薄壳封装**」在 VLA 上的兑现——非自家训的模型如何保持 thin delta。

四条策略分别适配四类模型 ownership，每一族接线的成本都是「写 pipeline.py + stage.py + deploy yaml + 测试」这四步，没有例外。这跟 §51.3 的两条正交策略是同一个框架在不同族上的实例化。

下一篇候选：
1. **SmolVLA CUDA Graph 接入**：把 `enable_fixed_kv_buffer` 复刻进 LeRobot SmolVLAPolicy。如果你想跑这条线，先说一声。
2. **跨族 serve-path 端到端 e2e**：把 4 族用同一份 bench harness 跑起来，对比 latency / VRAM。
3. **Omni() OpenAI API 适配**：当前 `OmniRequestOutput` 已经有结构化形状，加个 `POST /v1/chat/completions` 适配 OpenAI 接口，让 smolvlm 和 smolvla 能用同一套 client。

哪条想跟？
