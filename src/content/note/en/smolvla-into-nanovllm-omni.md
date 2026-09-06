---
title: "Porting SmolVLA into nanovllm-omni: from LeRobot action chunks to Omni().generate(...)"
timestamp: 2026-09-03T23:30:00+08:00
tags:
  - AI
  - VLA
  - Robotics
  - LeRobot
  - Engineering
series: nanovllm-omni Dev Notes
description: "This post is only about how to wire the model in: the four wiring steps (pipeline.py + stage.py + deploy/smolvla.yaml + registry auto-registration), the two call paths (synthetic demo via predict_action_chunk, LIBERO eval replicating the official chain byte-for-byte), and the new ActionArtifact output contract."
toc: true
---

# Porting SmolVLA into nanovllm-omni: from LeRobot action chunks to Omni().generate(...)

## This post is one thing: how to wire a new model family into Omni()

The repo registers 4 model families: `minimind_omni` / `smolvlm` / `sd_turbo` / `smolvla`. The first three can run the full stack through `Omni(...).generate(...)`; only `smolvla` is left. This post closes that last family — and only covers the integration itself: which four files to write, how inputs flow in, what the output looks like.

Adding a family = adding a new `final_output_type` + its matching output contract. SmolVLA's three components (SigLIP vision + SmolVLM language + action expert) have cross-attention between them and can't be split into stages at inference time, so it registers as a **single stage**:

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

`final_output_type="actions"` sits alongside MiniMind-O's `"audio"`, SmolVLM's `"text"`, and SD-Turbo's `"image"` — one of the four legal keys of `OmniRequestOutput.multimodal_output` (topology at `nanovllm_omni/models/smolvla/pipeline.py:26-35`).

## The four wiring steps

### 1. pipeline.py: declare + register

`nanovllm_omni/models/smolvla/pipeline.py` declares a `PipelineConfig` and registers it into `OMNI_PIPELINES`. Three `registration_handles` (`smolvla` / `HuggingFaceVLA/smolvla_libero` / `lerobot/smolvla_base`) + one `hf_architectures=("SmolVLAPolicy",)` as the model-type routing disambiguator.

### 2. stage.py: adapt LeRobot's interface to Omni's call contract

SmolVLA was trained by LeRobot; `select_action` / `predict_action_chunk` internally hold a 50-step AR chunk queue + cross-attention, all under LeRobot's control. We **don't rewrite forward, don't vendor** — just one thin adapter: `models/smolvla/stage.py`'s 269 lines are all adaptation code, with zero LeRobot logic copied.

Two key adaptation points:

**Version-compatible import** (works across old and new namespaces), `stage.py:39-56`:

```python
try:
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
except ImportError:
    from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
```

**Input assembly + two call paths + output wrapping**, covered below.

### 3. deploy/smolvla.yaml: single-stage config

`default_sampling_params: {}` — a VLA has no "sampling" concept; actions are deterministic unless you do ensembles. Deploy leaves it empty for the caller to override.

### 4. Registry auto-registration + routing fallback

`_load_builtin_pipelines()` at `config/registry.py:265-276` auto-imports the four pipeline modules as a side-effect registration, so SmolVLA joins `OMNI_PIPELINES` with the other three families; users **never import it directly**.

The routing fallback is a 6-layer cascade (`try_infer_model_type` in `entrypoints/base.py`); smolvla is recognized at both layer 5 (path basename substring) and layer 6 (`hf_architectures`) — even an empty config.json gets the family right from a path like `/path/to/smolvla_libero`.

## Two call paths

A VLA wants not "the next token" but "the actions for the next 50 steps". The stage factory branches inside `vla_forward` at `stage.py:239-264` by input:

### Path A: synthetic demo (default)

`SamplingParams.extra["image"]` (HWC uint8 RGB) + `extra["wrist_image"]` (optional, second camera) + `extra["state"]` (robot proprioception) come in. `_obs_batch` (`stage.py:148`) converts the image to NCHW, packs the state into the batch, and assembles lerobot's expected `observation.images.image` / `observation.state` / `task` keys.

It goes through `policy.predict_action_chunk(batch)`: emits the whole chunk at once (`[chunk_size, action_dim]`; the repo's test mock uses chunk_size=50, action_dim=7, matching LIBERO's 7-DoF end-effector).

The benefit: **you don't need to install the LIBERO environment** to run the unit test — `tests/test_smolvla.py::test_omni_generate_returns_actions` replaces the policy with `monkeypatch` (numpy + torch only, no lerobot).

### Path B: LIBERO eval (byte-for-byte replication of the official chain)

When `SamplingParams.extra["libero_obs"]` is present, it branches to `_libero_forward` (`stage.py:213-237`), **strictly replicating the processing order of lerobot/scripts/lerobot_eval.py ~L268-300**. LIBERO is SmolVLA's official benchmark, so to line up with the leaderboard the scoring must reproduce the original chain 100%:

```
preprocess_observation(obs)  # rename unknown keys → observation.*
→ inject task (must be injected last, else it gets renamed to observation.task and corrupts the tokenizer)
→ env_preprocessor  # flip + quat->axis-angle
→ policy.preprocessor
→ policy.select_action  # 50-step chunk queue
→ policy.postprocessor
→ env_postprocessor
```

The in-function comment at `stage.py:220-223` stresses that "preprocess must precede task injection" — a pitfall learned from LeRobot's eval code; skipping the order directly scrambles the tokenizer's lookup table. `_env_processors` (`stage.py:204`) uses the `task_suite` string as a cache key so each `forward` doesn't rebuild lerobot's `LiberoEnvConfig.get_env_processors()`.

## The output contract: ActionArtifact

A VLA's output is an action tensor `[chunk_size, action_dim]`. `ActionArtifact` is defined at `nanovllm_omni/outputs.py:365`:

```python
@dataclass(frozen=True)
class ActionArtifact:
    array: Any            # 2-D [chunk_size, action_dim]
    action_dim: int
    chunk_size: int
    dtype: str            # str(getattr(array, "dtype", ""))

    @classmethod
    def from_array(cls, array: Any) -> "ActionArtifact":
        # auto-extract chunk_size/action_dim from array.shape, validate 2-D
        ...
```

`__post_init__` runs shape validation — `chunk_size` must equal `array.shape[0]` and `action_dim` must equal `array.shape[1]`. That nails the "VLA output must be 2-D" contract down at the type layer.

The stage factory wraps whatever torch tensor / numpy array LeRobot returns into an `ActionArtifact` in `_to_action_artifact` (`stage.py:181-195`), then stuffs it into `OmniRequestOutput.multimodal_output["actions"]`.

That keeps `Omni.generate()`'s **public API untouched** — `multimodal_output` is already `dict[str, Any]`, so adding an `"actions"` key is an additive change requiring no changes to any existing caller.

## Wrap

The integration cost is the same four steps, no exceptions: pipeline.py + stage.py + deploy yaml + tests. No GPU e2e was done — that needs a real LeRobot checkpoint + CUDA, limited by the RTX 3050 4GB VRAM. Next session.