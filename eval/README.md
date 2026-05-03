# CarlaAir-Eval

Evaluation suite for the cooperative air-ground VLA tasks in the CarlaAir
paper. Runs the two diagnostic scenarios (moving-platform landing and
occlusion-recovery escort), the three cooperation modes (C0 / C1 / C2),
and the prompt-format ablation, and reports all the metrics defined in
the paper.

The suite is policy-agnostic — there are no baselines included. You plug
in your own UAV policy through a small `UAVPolicy` interface, and the
runner takes care of scenarios, cooperation protocol, traces, and
metric aggregation.

## Layout

```
carlaair_eval/
  api/           UAVPolicy ABC, action types, prompt builders
  scenarios/     Landing and Escort scenarios (CARLA + AirSim glue)
  runtime/       Episode runner, C0/C1/C2 coordinators, JSONL trace writer
  metrics/       TSR / LSR / CCR / CG, RSR / RAT, DF / ECL, bootstrap + sign test
  reference/     Rule-Coop-State state-feedback reference
  configs/       YAML files for the four condition grids
  scripts/       run_eval, compute_report, smoke tests
  examples/      Minimal dummy policy
  tests/         Unit tests for prompts and metric definitions
  constants.py   Paper constants (episode lengths, thresholds, C2 controller, etc.)
```

## Install and run

```bash
cd eval/
pip install -e .
```

Start CarlaAir in a separate terminal (from the repo root):

```bash
./platform/CarlaAir.sh Town10HD --quality Epic
```

Run an evaluation:

```bash
python -m carlaair_eval.scripts.run_eval \
    --config carlaair_eval/configs/landing_main.yaml \
    --policy mypkg.my_module:MyPolicy \
    --out results/my_policy
```

This writes a per-tick JSONL trace per episode plus a CSV / JSON summary
in the schema of the paper's Table 2 / Table 4 (and `timing.csv` for
DF / ECL).

To check that the suite works against your CarlaAir install without
writing any policy code:

```bash
python scripts/smoke_test.py        # ~3 s landing scenario
python scripts/smoke_test_escort.py # ~3 s escort scenario
```

## Unit tests

A 21-test conformance suite under `tests/` runs in a few seconds with
no simulator required:

```bash
pytest tests/
```

What it covers:

| Group | Count | What it asserts |
|---|---|---|
| Prompt templates  | 10 | Cue strings produced by `api/cue.py` match the canonical examples in Table 6 / Table C.5 of the paper character-for-character. |
| Landing metrics   | 5 | Boundary cases of TSR (`K = 3 s`), LSR (drift `≤ 0.3 m` within `2 s`, no collision), and CCR / CG aggregation. |
| Escort metrics    | 3 | RSR aggregation across multiple occlusion events; RAT cap at `15 s`. |
| Statistics        | 3 | Hierarchical bootstrap CI degenerate cases; sign-test edge cases. |

## Plugging in a UAV policy

```python
from carlaair_eval.api import (
    UAVPolicy, VelocityCommand, Observation, PartnerCue,
)

class MyPolicy(UAVPolicy):
    def reset(self, task_instruction, task_name, episode_id):
        ...

    def act(self, observation: Observation, partner_cue: PartnerCue | None):
        # partner_cue is None under C0; under C1 / C2 it carries the
        # paper-formatted prompt fragment in cue.text plus structured
        # fields in cue.structured.
        return VelocityCommand(vx=..., vy=..., vz=..., yaw_deg=...)
```

`act()` can return any of four action types, one per baseline regime in
the paper:

| Regime                       | Class                |
|------------------------------|----------------------|
| Continuous velocity command  | `VelocityCommand`    |
| Single waypoint              | `WaypointCommand`    |
| Dense trajectory             | `TrajectoryCommand`  |
| Discrete vocabulary          | `DiscreteCommand`    |

Discrete actions map to 0.5 s velocity bursts at 1.5 m/s, matching the
AerialVLN adaptation in the paper.

## Third-party baselines

We do not redistribute third-party baseline implementations or model
weights. Each baseline should be obtained from its official release.

This repository provides:

- the CarlaAir evaluation runtime;
- diagnostic task configurations;
- prompt templates;
- metric computation scripts;
- baseline adapter interfaces (input/output mapping for each policy regime).

The evaluated baselines are:

- [AerialVLA](https://arxiv.org/abs/2603.14363)
- [OpenFly](https://arxiv.org/abs/2502.18041)
- [OpenUAV](https://arxiv.org/abs/2410.07087)
- [SPF](https://arxiv.org/abs/2509.22653)
- [AerialVLN](https://openaccess.thecvf.com/content/ICCV2023/papers/Liu_AerialVLN_Vision-and-Language_Navigation_for_UAVs_ICCV_2023_paper.pdf)

All baselines are used with their official checkpoints and default
inference settings. To reproduce the paper's numbers, plug each
baseline into the `UAVPolicy` interface described above and run the
condition grid in `carlaair_eval/configs/`.

## Cooperation modes

| Mode                 | What it does                                                          |
|----------------------|------------------------------------------------------------------------|
| `C0`                 | No communication                                                       |
| `C1` / `C1-Sem`      | UGV→UAV semantic cue                                                   |
| `C1-Num`             | Same path, structured numeric cue                                      |
| `C1-Noisy`           | Same path, semantic cue with 30 % per-field corruption                 |
| `C1-Oracle-Bearing`  | Oracle bearing / range / elevation cue                                 |
| `C2`                 | Bidirectional, landing only — UAV forward-velocity magnitude drives UGV longitudinal speed |

The C2 controller follows Eq. (1) in the paper:
`v_UGV = v0 · clip(‖v_UAV^fwd‖ / v_ref, 0.5, 1.5)` with
`v0 = 4.0 m/s`, `v_ref = 2.0 m/s`. These live in `constants.py`. The
forward-velocity magnitude is read from the baseline's native action
output (continuous velocity → ‖(vx, vy)‖; waypoint → displacement /
inference period; trajectory → first-segment velocity; discrete →
realized UAV speed at the same tick).

## Metrics

| Metric | Definition                                                                                       | Code              |
|--------|--------------------------------------------------------------------------------------------------|-------------------|
| TSR    | Target visible to UAV camera ≥ 3 s cumulative                                                    | `metrics/landing` |
| LSR    | Lands on cargo bed ≤ 60 s, drift ≤ 0.3 m within 2 s of first contact, no collision               | `metrics/landing` |
| CCR    | LSR / max(TSR, 0.05)                                                                             | `metrics/landing` |
| CG     | LSR(C_k) − LSR(C_0)                                                                              | `metrics/landing` |
| RSR    | IoU ≥ 0.15 sustained ≥ 0.5 s within 15 s of occlusion onset                                      | `metrics/escort`  |
| RAT    | Onset → first frame above the IoU threshold; capped at 15 s                                      | `metrics/escort`  |
| DF     | Realised control update rate (per-episode median Δt)                                             | `metrics/timing`  |
| ECL    | Inter-tick delay + cue + policy inference time, partner-side consumption                         | `metrics/timing`  |

Aggregates use the paper's protocol: 3 seeds × 50 episodes per
condition, 95 % CIs from a 1000-resample hierarchical bootstrap
clustered by seed, across-baseline trends with a sign test on per-seed
mean CGs, and timing reported as episode-level median + IQR.

## Prompts

Prompt strings are kept in `api/cue.py`. The semantic / numeric / noisy
/ oracle-bearing variants for both tasks reproduce the templates in
Table 6 and Table C.5 of the paper. `tests/test_prompts.py` checks the
canonical examples character-for-character so the templates do not
silently drift.

## Output

```
results/my_policy/
  C0/seed_0/landing_C0_seed0_ep000.jsonl    per-tick trace
  C0/seed_0/landing_C0_seed0_ep001.jsonl
  ...
  landing_main.csv                          mode × {TSR, LSR, CCR, CG}
  landing_main.json                         + 95 % CI per cell
  timing.csv                                DF / ECL median + IQR
```

Each trace's header records the constants the runner used, so an
episode is reproducible and auditable from the file alone.

## Reference policy

`reference/rule_coop_state.py` is the state-based cooperative reference
from §4.2 of the paper. It uses ground-truth metric state and
deterministic feed-forward + P-control rules; useful as a sanity check
that the suite plumbing is wired correctly, but it is not a fair
baseline (it sees information the VLA models don't).

## Notes

- The bridge-occlusion event is the one geometry implemented in this
  release; the paper additionally lists building and large-artifact
  occluders in the geometry mix.
- Single-process execution means UAV and UGV observations come from the
  same physics tick — no inter-process timestamp alignment. This is a
  property of the underlying CarlaAir runtime, not of the eval suite.
- `--paper-strict` (on by default) keeps every constant in
  `constants.py` immutable; pass explicit overrides on the CLI if you
  want to deviate.
