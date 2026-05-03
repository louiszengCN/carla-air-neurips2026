# Anonymous Code Supplement

<p align="center">
  <img src="docs/images/teaser_video.gif" alt="CarlaAir teaser" width="100%"/>
</p>

This repository contains the code accompanying our NeurIPS 2026 submission on
closed-loop air-ground cooperative VLA evaluation. It has two parts:

- **`platform/`** — the CarlaAir runtime: the single-process integration of
  CARLA and AirSim described in §3 / Appendix B of the paper. Contains the
  three modified upstream files that compose the AirSim flight subsystem
  into CARLA's `GameMode`, a from-source build guide, the unified launcher,
  the AirSim plugin configuration, and the conda-based environment
  bootstrap.
- **`eval/`** — the diagnostic evaluation suite from §4 / Appendix C: the
  two cooperative tasks (moving-platform landing and occlusion-recovery
  escort), the C0 / C1 / C2 cooperation modes, the prompt-format ablation,
  the Rule-Coop-State reference, and the metric pipeline (TSR, LSR, CCR,
  CG, RSR, RAT, DF, ECL).

The two parts are independent: `platform/` is what an end user runs to host
the simulation; `eval/` is the Python package that drives evaluation
episodes on top of it.

## Quick start

CarlaAir is built from source. The eval suite assumes a running CarlaAir
process; build the platform first, then drive it with the eval package.

```bash
# 1. Build CarlaAir from source. End to end this builds CARLA 0.9.16
#    + AirSim 1.7.0 + Unreal Engine 4.26 with the three modified files
#    in platform/source_modifications/ applied. See platform/BUILD.md
#    for the full step-by-step guide.
#    The result is platform/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping.

# 2. Set up the Python environment used by both parts.
bash platform/env_setup/setup_env.sh
conda activate carlaAir

# 3. Launch the simulator.
./platform/CarlaAir.sh Town10HD --quality Epic

# 4. In a second terminal, install the eval package and run an evaluation.
pip install -e eval/
python -m carlaair_eval.scripts.run_eval \
    --config eval/carlaair_eval/configs/landing_main.yaml \
    --policy mypkg.my_module:MyPolicy \
    --out results/my_policy
```

See [`platform/BUILD.md`](platform/BUILD.md) for the build steps and
[`eval/README.md`](eval/README.md) for the `UAVPolicy` interface and the
condition grids.

## Directory layout

```
.
├── platform/
│   ├── BUILD.md                     From-source build guide
│   │                                (CARLA 0.9.16 + AirSim 1.7.0 + UE4.26).
│   ├── CarlaAir.sh                  Unified launcher (CARLA + AirSim ports).
│   ├── auto_traffic.py              Background traffic + pedestrian spawner.
│   ├── AirSimConfig/settings.json   AirSim plugin configuration.
│   ├── env_setup/                   conda-based environment bootstrap.
│   └── source_modifications/        The 3 upstream files modified to compose
│                                    AirSim into CARLA's GameMode (paper App. B).
└── eval/
    ├── carlaair_eval/               Eval package (api, scenarios, runtime,
    │                                metrics, reference, configs).
    │   └── scenarios/data/          Self-contained UGV trajectory used by
    │                                the escort scenario.
    ├── scripts/                     Smoke tests.
    ├── tests/                       Unit tests.
    └── README.md                    Eval-package documentation.
```

## Visual examples

Two grid montages, recorded directly inside CarlaAir, showing every
baseline / cooperation-mode combination side by side. Each cell is a
single rollout with the corresponding (baseline, mode) configuration;
empty cells in the rightmost column indicate that the Rule-Coop-State
reference is not run under that cooperation mode.

<p align="center">
  <img src="docs/gifs/landing_grid.gif" alt="Landing-task baseline grid" width="100%"/>
</p>

<p align="center"><em>
<b>Cooperative Moving-Platform Landing.</b> Rows: cooperation mode
(C0 / C1 / C2). Columns: aerial baseline (AerialVLA, OpenFly, OpenUAV,
SPF, AerialVLN) and the Rule-Coop-State reference.
</em></p>

<p align="center">
  <img src="docs/gifs/escort_grid.gif" alt="Escort-task baseline grid" width="100%"/>
</p>

<p align="center"><em>
<b>Cooperative Occlusion-Recovery Escort.</b> Rows: cooperation mode
(C0 / C1). Columns: same baselines plus the Rule-Coop-State reference.
</em></p>

> **Camera note.** All clips use the AirSim default 3rd-person chase
> camera, which is rigidly attached to the UAV body. Any visible camera
> shake is the UAV's own motion (the AirSim multirotor reacts to every
> commanded velocity change), not a recording artefact.

## License

Released under the MIT License (see `LICENSE`).
