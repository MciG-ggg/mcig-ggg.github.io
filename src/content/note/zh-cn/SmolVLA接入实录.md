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
description: 这篇只讲怎么接入：四步接线（pipeline.py + stage.py + deploy/smolvla.yaml + registry 自动注册）、两条调用路径（synthetic demo 走 predict_action_chunk、LIBERO eval 严格复刻官方处理链）、以及新增的 ActionArtifact 输出契约。
toc: true
---

## 这篇只讲一件事：怎么把一个新的模型族接入 Omni()

仓库里注册了 4 个模型族：`minimind_omni` / `smolvlm` / `sd_turbo` / `smolvla`。前三族都能走 `Omni(...).generate(...)`，只剩 `smolvla`。这篇补上最后这一族——只讲接入动作本身：写哪四个文件、输入怎么进、输出长什么样。

加一个新的模型族 = 加一个新的 `final_output_type` + 配套的输出契约。SmolVLA 的三组件（SigLIP 视觉 + SmolVLM 语言 + action expert）之间有 cross-attention，推理阶段没法拆成多 stage，所以它注册成**单 stage**：

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

`final_output_type="actions"` 跟 MiniMind-O 的 `"audio"`、SmolVLM 的 `"text"`、SD-Turbo 的 `"image"` 并列，是 `OmniRequestOutput.multimodal_output` 的四种合法 key 之一（拓扑见 `nanovllm_omni/models/smolvla/pipeline.py:26-35`）。

## 四步接线

### 1. pipeline.py：声明 + 注册

`nanovllm_omni/models/smolvla/pipeline.py` 声明 `PipelineConfig`，注册进 `OMNI_PIPELINES`。三个 `registration_handles`（`smolvla` / `HuggingFaceVLA/smolvla_libero` / `lerobot/smolvla_base`）+ 一个 `hf_architectures=("SmolVLAPolicy",)` 作为模型类型路由的 disambiguator。

### 2. stage.py：把 LeRobot 接口适配成 Omni 调用契约

SmolVLA 是 LeRobot 训的，`select_action` / `predict_action_chunk` 内部是 50 步 AR chunk 队列 + cross-attention，全在 LeRobot 手里。我们**不重写 forward、不 vendor**，只写一层薄适配——`models/smolvla/stage.py` 的 269 行全是适配代码，LeRobot 自身逻辑一行没复制。

关键适配点有两个：

**版本兼容的 import**（新旧 namespace 都挂得上），`stage.py:39-56`：

```python
try:
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
except ImportError:
    from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
```

**输入装配 + 两条调用路径 + 输出包装**，见下两节。

### 3. deploy/smolvla.yaml：单 stage 配置

`default_sampling_params: {}`——VLA 没"采样"概念，动作是确定性的（除非做 ensemble），deploy 留空给 caller override。

### 4. registry 自动注册 + 路由兜底

`config/registry.py:265-276` 的 `_load_builtin_pipelines()` 自动 import 四个 pipeline 模块做 side-effect 注册，SmolVLA 跟其他三族一起进 `OMNI_PIPELINES`，**用户不直接 import**。

路由兜底有 6 层 cascade（`entrypoints/base.py` 的 `try_infer_model_type`），smolvla 命中第 5 层（路径 basename substring）和第 6 层（`hf_architectures`）都能识别——空 config.json 时 `/path/to/smolvla_libero` 这种路径也能猜对模型族。

## 两条调用路径

VLA 要的不是「下一个 token」，是「接下来 50 步的动作」。stage 工厂在 `stage.py:239-264` 的 `vla_forward` 里按输入分支：

### 路径 A：synthetic demo（默认）

`SamplingParams.extra["image"]`（HWC uint8 RGB）+ `extra["wrist_image"]`（可选，第二相机）+ `extra["state"]`（机器人本体感知）传进来。`_obs_batch`（`stage.py:148`）把图像转 NCHW、state 装进 batch、组装 lerobot 期望的 `observation.images.image` / `observation.state` / `task` 键。

走 `policy.predict_action_chunk(batch)`：一次性吐整块动作 `[chunk_size, action_dim]`（本仓库测试 mock 用 chunk_size=50、action_dim=7，对应 LIBERO 的 7-DoF 末端执行器）。

这条路径的好处：**不用装 LIBERO 环境**就能跑通单元测试——`tests/test_smolvla.py::test_omni_generate_returns_actions` 用 `monkeypatch` 替换 policy，numpy + torch 即可，无需 lerobot。

### 路径 B：LIBERO eval（精确复刻官方评估链）

`SamplingParams.extra["libero_obs"]` 存在时走 `_libero_forward`（`stage.py:213-237`），**严格复刻 lerobot/scripts/lerobot_eval.py ~L268-300 的处理顺序**。LIBERO 是 SmolVLA 的官方 benchmark，跑分要对得上 leaderboard，必须 100% 复现原版处理链：

```
preprocess_observation(obs)  # 改名 unknown keys → observation.*
→ 注入 task（必须后注入，否则被改名成 observation.task 破坏 tokenizer）
→ env_preprocessor  # flip + quat->axis-angle
→ policy.preprocessor
→ policy.select_action  # 50 步 chunk 队列
→ policy.postprocessor
→ env_postprocessor
```

`stage.py:220-223` 的注释专门强调「preprocess 必须先于 task 注入」——从 LeRobot 评测代码里学到的坑，跳过顺序会直接打乱 tokenizer 查表。`_env_processors`（`stage.py:204`）用 `task_suite` 字符串做缓存键，避免每次 `forward` 都重建 lerobot 的 `LiberoEnvConfig.get_env_processors()`。

## 输出契约：ActionArtifact

VLA 输出是动作张量 `[chunk_size, action_dim]`。`nanovllm_omni/outputs.py:365` 定义 `ActionArtifact`：

```python
@dataclass(frozen=True)
class ActionArtifact:
    array: Any            # 2-D [chunk_size, action_dim]
    action_dim: int
    chunk_size: int
    dtype: str            # str(getattr(array, "dtype", ""))

    @classmethod
    def from_array(cls, array: Any) -> "ActionArtifact":
        # 自动从 array.shape 抽 chunk_size/action_dim，验证 2-D
        ...
```

`__post_init__` 做 shape 验证——`chunk_size` 必须等于 `array.shape[0]`、`action_dim` 必须等于 `array.shape[1]`，把「VLA 输出必须是 2-D」钉死在类型层。

stage 工厂在 `_to_action_artifact`（`stage.py:181-195`）把 LeRobot 返回的 torch tensor / numpy array 统一包成 `ActionArtifact`，塞进 `OmniRequestOutput.multimodal_output["actions"]`。

这一步让 `Omni.generate()` **公共 API 不动**——`multimodal_output` 本来就是 `dict[str, Any]`，加一个 `"actions"` key 是 additive change，不需要改任何已存在的 caller。

## 结束

接入成本就是上面四步：pipeline.py + stage.py + deploy yaml + 测试，没有例外。没做 GPU e2e——需要真实 LeRobot checkpoint + CUDA，受限于 RTX 3050 4GB VRAM，等下次 session。