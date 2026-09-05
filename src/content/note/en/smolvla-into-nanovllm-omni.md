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
description: The first two posts cut MiniMind-O end-to-end from 846ms to 42ms. This one switches track — wiring up SmolVLA, the only one of the 4 model families not yet connected to Omni(). It documents the thin-shell strategy validated on a VLA, the design choices behind two call paths (synthetic demo + LIBERO eval), the ActionArtifact output contract, and why we still didn't do CUDA Graph this time.
toc: true
---

# Porting SmolVLA into nanovllm-omni: from LeRobot action chunks to Omni().generate(...)

## Where the last posts left off

The previous two posts, [《Cutting MiniMind-O from 846ms to 320ms》](/en/note/minimind-o-from-846ms-to-320ms) and [《Pushing MiniMind-O another 7.5×》](/en/note/minimind-o-cuda-graph), were both about the MiniMind-O family. But the repo actually registers 4 model families: `minimind_omni` / `smolvlm` / `sd_turbo` / `smolvla`. The first three can run the full stack through `Omni(...).generate(...)`; only `smolvla` was left — the earlier [§51.2 cross-family scan](https://github.com/mcig-ggg/nanovllm-omni/blob/main/docs/perf/ncu-generate-kernels-2026-09-01.md) had flagged it as a "future candidate". This post is cashing that future candidate in.

## Why wire in a VLA

VLA (Vision-Language-Action) has been the hottest paradigm in robotics over the last two years. LLM as the brain, vision encoder as the eyes, action head as the hands — one model spits out an action sequence straight from image + instruction. `Omni(...)` was designed to "unify multiple modalities" from the start; without the VLA family it isn't complete.

SmolVLA is a small VLA trained by the HuggingFace LeRobot team: SigLIP (vision) + SmolVLM (language) + an action expert, **with all three components packed into a single checkpoint**. That's different from MiniMind-O's "thinker + talker + code2wav three-stage" — SmolVLA can't be split into stages at inference time because the three components have cross-attention between them. So `nanovllm_omni/models/smolvla/pipeline.py:17-30` is a **single stage**, not multiple:

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

`final_output_type="actions"` is the key — it sits alongside MiniMind-O's `"audio"`, SmolVLM's `"text"`, and SD-Turbo's `"image"`, one of the four legal keys of OmniRequestOutput's multimodal_output (pipeline topology at `nanovllm_omni/models/smolvla/pipeline.py:26-35`). The `Omni` public API doesn't move; adding a family = adding a new `final_output_type` + its matching output contract (`ActionArtifact`, below).

## Validating the thin-shell strategy on a VLA

Per §51.3's "go deep into forward vs thin shell" strategy — models not trained in-house take the thin-shell route — SmolVLA is trained by HuggingFace VLA / LeRobot, so it takes the thin shell. Specifically:

1. **No forward rewrite** — `SmolVLAPolicy.select_action` / `predict_action_chunk` were trained by LeRobot; inside there's a 50-step AR chunk queue, cross-attention, proprioception handling, all its own implementation. We **only call it**, never touch it.
2. **No vendoring** — `lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy` comes from PyPI's `lerobot`; the import path relies on the dual-import compat in `nanovllm_omni/models/smolvla/stage.py:39-56` (old vs new namespace):

   ```python
   try:
       from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
   except ImportError:
       from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
   ```
3. **No fork** — `models/smolvla/stage.py`'s 269 lines are all **thin code adapting LeRobot's interface to Omni's call contract**, with zero LeRobot logic copied.

This is the same principle from §51.3 — "sglang / vllm-omni also centralize patches; nanovllm-omni's differentiation is vendor=0 + fork=0 + subclass=0" — cashed out on a VLA.

## Two call paths

A VLA is more complex than text generation — what it wants isn't "the next token", it's "the actions to take over the next 50 steps". That's why SmolVLA has two call paths, and the stage factory branches inside `vla_forward` at `stage.py:239-264`:

### Path A: synthetic demo (default)

`SamplingParams.extra["image"]` (HWC uint8 RGB) + `extra["wrist_image"]` (optional, second camera) + `extra["state"]` (robot proprioception) come in. `_obs_batch` (`stage.py:148`) converts the image to NCHW, packs the state into the batch, and assembles the `observation.images.image` / `observation.state` / `task` keys lerobot expects.

It goes through `policy.predict_action_chunk(batch)`: emits the whole chunk of actions at once (`[chunk_size, action_dim]`; the repo's test mock uses chunk_size=50, action_dim=7, matching LIBERO's 7-DoF end-effector dimensions).

The benefit of this path is you **don't need to install the LIBERO environment** — the unit test `tests/test_smolvla.py::test_omni_generate_returns_actions` runs it by replacing the policy with `monkeypatch` (numpy + torch only, no lerobot needed).

### Path B: LIBERO eval (byte-for-byte replication of the official eval chain)

When `SamplingParams.extra["libero_obs"]` is present, it branches to `_libero_forward` (`stage.py:213-237`), and this path **strictly replicates the processing order of lerobot/scripts/lerobot_eval.py ~L268-300**. This step is critical — LIBERO is SmolVLA's official benchmark, and you can only line up with the leaderboard if your scoring reproduces the original processing chain 100%.

The order is:
```
preprocess_observation(obs)  # rename unknown keys → observation.*
→ inject task (must be injected last, else it gets renamed to observation.task and corrupts the tokenizer)
→ env_preprocessor  # flip + quat->axis-angle
→ policy.preprocessor
→ policy.select_action  # 50-step chunk queue
→ policy.postprocessor
→ env_postprocessor
```

The in-function comment at `stage.py:220-223` specifically stresses that "preprocess must precede task injection" — a pitfall learned from LeRobot's eval code; skipping the order directly scrambles the tokenizer's lookup table.

`_env_processors` (`stage.py:204`) uses the `task_suite` string as a cache key, so each `forward` doesn't rebuild lerobot's `LiberoEnvConfig.get_env_processors()`.

## The Nth piece of wiring: pipeline.yaml + registry auto-registration

Exactly the same wiring pattern as sd_turbo / smolvlm — this part was mostly copied from the MiniMind-O session:

1. **`nanovllm_omni/models/smolvla/pipeline.py`**: declares `PipelineConfig`, registers into `OMNI_PIPELINES`. Three `registration_handles` (`smolvla` / `HuggingFaceVLA/smolvla_libero` / `lerobot/smolvla_base`) + an `hf_architectures=("SmolVLAPolicy",)` as the Layer-6 disambiguator.
2. **`nanovllm_omni/deploy/smolvla.yaml`**: single stage, `default_sampling_params: {}` (a VLA has no "sampling" concept — actions are deterministic unless you do ensembles; deploy leaves it empty for caller override).
3. **`_load_builtin_pipelines()` at `nanovllm_omni/config/registry.py:265-276`** auto-imports the four pipeline modules as a side-effect registration. `SmolVLA` joins `OMNI_PIPELINES` with the other three families; users **don't import it directly**.

The routing fallback has a 6-layer cascade (`try_infer_model_type` in `entrypoints/base.py`); smolvla is recognized at both layer 5 (path basename substring) and layer 6 (`hf_architectures`). `tests/test_smolvla.py::test_local_dir_basename_falls_back_to_registered_key` verifies that even with an empty config.json, a path like `/path/to/smolvla_libero` is guessed correctly — a fallback learned from the MiniMind-O session.

## The output contract: ActionArtifact

A VLA's output is neither a string nor an image — it's an **action tensor**: a `[chunk_size, action_dim]` numpy array (or torch tensor). `ActionArtifact` is defined at `nanovllm_omni/outputs.py:365`:

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

The stage factory wraps whatever torch tensor / numpy array LeRobot returns into an `ActionArtifact` in `_to_action_artifact` (`stage.py:181-195`), then stuffs it into `OmniRequestOutput.multimodal_output["actions"]`. `tests/test_smolvla.py::test_from_pipeline_actions_key` verifies `OmniRequestOutput.from_pipeline(artifact, final_output_type="actions")` routes correctly to the `"actions"` key.

This contract has a second role: **keeping the Omni public API untouched**. `Omni.generate()`'s `multimodal_output` is already `dict[str, Any]`; adding an `"actions"` key is an additive change that requires no changes to any existing caller. That's consistent with the wiring principle of the previous 4 families.

## Why we didn't do CUDA Graph on the side this time

`§51.2` flagged SmolVLA as a "future candidate". This integration also confirmed it — per the §51.2 cross-family scan's judgement (of the 5 conditions, only condition 5 even came close, and wasn't actually measured; the other 4 are all "depends on LeRobot's internal forward"), SmolVLA's shape doesn't fit CUDA Graph's decision box:

- Condition 1 (launch-bound): not measured, but LeRobot's SmolVLAPolicy internally is a 50-step AR chunk queue, each step running SigLIP + SmolVLM + action expert — per-step work is far larger than MiniMind-O's single transformer layer. The launch-overhead share is unknown.
- Condition 2 (fixed shape): the chunk queue's shape inside `select_action` is relatively stable (50 steps of AR per chunk), but inside the 50 steps it's dynamic AR — shape drifts token by token.
- Conditions 3-4 (address stable / branch stable): LeRobot's internal forward is outside our control; we can't guarantee preallocated-buffer compatibility.
- Condition 5 (re-capture amortization): if the 50-step chunk queue re-captures every step internally, the cost doesn't amortize.

If you ever want to do it, the first thing is **not capture** — it's replicating the `enable_fixed_kv_buffer` preallocated-buffer + `_kv_pos` cursor infrastructure **into the internals of LeRobot's SmolVLAPolicy**. The engineering effort is comparable to the MiniMind-O CUDA Graph session (see [《Pushing MiniMind-O another 7.5×》](/en/note/minimind-o-cuda-graph)) — an independent session's worth of work, out of scope for this "integration".

## Verification coverage

`tests/test_smolvla.py` has 12 tests covering:

- registration layer: `resolve_pipeline_config("smolvla")` / `resolve_pipeline_config("HuggingFaceVLA/smolvla_libero")` both return the correct `PipelineConfig`
- output contract: `ActionArtifact.from_array` + `OmniRequestOutput.from_pipeline(artifact, final_output_type="actions")` work through
- missing-dependency fallback: swapping `_import_smouvla_policy` to raise ImportError → raises a clear "lerobot is required for SmolVLA"
- missing-input fallback: missing `extra["image"]` → raises a `ValueError` pointing to the right path
- `Omni().generate()` end-to-end: mocking a `_Policy.predict_action_chunk` → the full stack `PipelineRunner → stage bundle → vla_forward → _to_action_artifact → OmniRequestOutput` runs through, outputting `ActionArtifact(chunk_size=50, action_dim=7)`
- 6-layer routing cascade: L1 / L3 / L5 / L6 all covered
- image bytes fallback (TK-017): base64 → PIL → NCHW tensor

No GPU e2e was done — it needs a real LeRobot checkpoint + CUDA, and running it through requires the WSL 3050 (limited by the RTX 3050 4GB VRAM). That's for the next session.

## Wrap-up

This integration has no performance numbers, no CUDA Graph, no kernel tuning — it just wired the last of the 4 families into the `Omni()` public API. Integration itself isn't optimization, but you can't optimize what isn't integrated.

How this relates to the two MiniMind-O posts:
- The first post (25 rounds of monkey-patch) was about the engineering value of "**going deep into forward**" — micro-optimizing a model you trained yourself;
- The second post (CUDA Graph 7.51×) was about the "**launch-bound adjudication**" decision method — the 5 conditions + cross-family scan;
- This post is about the "**thin-shell wrapper**" cashed out on a VLA — how to keep a thin delta for models not trained in-house.

Four strategies each fit four types of model ownership, and every family's wiring cost is the same four steps: "write pipeline.py + stage.py + deploy yaml + tests", no exceptions. That's the same framework as §51.3's two orthogonal strategies, instantiated on different families.

Next-post candidates:
1. **SmolVLA CUDA Graph integration**: replicating `enable_fixed_kv_buffer` into LeRobot's SmolVLAPolicy. If you want to run this line, say the word first.
2. **Cross-family serve-path end-to-end e2e**: running all 4 families through the same bench harness, comparing latency / VRAM.
3. **Omni() OpenAI API adapter**: `OmniRequestOutput` already has a structured shape; add a `POST /v1/chat/completions` adapter for the OpenAI interface so smolvlm and smolvla can share one client.

Which one do you want to follow?