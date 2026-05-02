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
  api/           UAVPolicy ABC, action types, prompt builders, phase decoder
  scenarios/     Landing and Escort scenarios (CARLA + AirSim glue)
  runtime/       Episode runner, C0/C1/C2 coordinators, JSONL trace writer
  metrics/       TSR / LSR / CCR / CG, RSR / RAT, DF / ECL, bootstrap + sign test
  reference/     Rule-Coop-State state-feedback reference
  configs/       YAML files for the four condition grids
  scripts/       run_eval, compute_report, smoke tests
  examples/      Minimal dummy policy
  tests/         Unit tests for prompts, oracle phase, and metrics
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

To run the unit tests (no simulator needed):

```bash
pytest tests/
```

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

## Cooperation modes

| Mode                 | What it does                                                          |
|----------------------|------------------------------------------------------------------------|
| `C0`                 | No communication                                                       |
| `C1` / `C1-Sem`      | UGV→UAV semantic cue                                                   |
| `C1-Num`             | Same path, structured numeric cue                                      |
| `C1-Noisy`           | Same path, semantic cue with 30 % per-field corruption                 |
| `C1-Oracle-Bearing`  | Oracle bearing / range / elevation cue                                 |
| `C2`                 | Bidirectional, landing only — UAV action drives UGV longitudinal speed |
| `C2-Oracle`          | C2 with VLA-decoded phase replaced by the oracle phase from Eq. (2)    |
| `C2-NoisyOracle`     | C2-Oracle with 30 % phase-label corruption                             |

The C2 UGV speed law follows Eq. (1) in the paper:
`v_UGV = v0 (1 + α · 1[approach] − β · 1[descend])` with
`v0 = 4.0 m/s`, `α = 0.25`, `β = 0.40`. These live in `constants.py`.

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
