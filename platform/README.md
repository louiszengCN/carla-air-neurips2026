# CarlaAir Platform

Single-process Unreal Engine runtime that hosts CARLA's ground-vehicle
simulation and AirSim's multirotor flight stack in the same world. Both
client APIs (`carla.Client`, `airsim.MultirotorClient`) connect to the
same UE4 process, and all sensor frames come off the same physics tick.

## Layout

```
platform/
├── README.md               This file.
├── BUILD.md                From-source build guide (CARLA + AirSim + UE4.26).
├── CarlaAir.sh             Launcher; opens CARLA on port 2000 and AirSim on 41451.
├── auto_traffic.py         Background traffic and pedestrian spawner used
│                           by CarlaAir.sh after the world is up.
├── AirSimConfig/
│   └── settings.json       AirSim plugin settings; copied to ~/Documents/AirSim/
│                           on first launch.
├── env_setup/
│   ├── setup_env.sh        Creates the carlaAir conda env and installs deps.
│   └── test_env.sh         Verifies that carla, airsim, and ports work.
└── source_modifications/   The three upstream files that, when applied on
                            top of CARLA 0.9.16 + AirSim 1.7.0 + UE4.26,
                            produce the unified runtime described in §3 /
                            App. B of the paper. See README inside.
```

## Two ways to use this

**(a) Build from source.** Follow [`BUILD.md`](BUILD.md) to build a
`CarlaUE4-Linux-Shipping` binary on top of CARLA 0.9.16 + AirSim 1.7.0
+ Unreal Engine 4.26 with the three modified files in
`source_modifications/` applied. Once built, place the binary tree
under `platform/CarlaUE4/` and run `./CarlaAir.sh`.

**(b) Use a pre-built CarlaAir distribution.** If you already have a
CarlaAir release tree, copy `CarlaAir.sh`, `auto_traffic.py`,
`AirSimConfig/`, and `env_setup/` from this repo into the release root
(or simply launch the release's own `CarlaAir.sh`). The eval suite in
[`../eval/`](../eval/) talks to the simulator over the standard
`carla` / `airsim` Python clients and does not care which build path
you took.

## Run

```bash
bash env_setup/setup_env.sh   # one-time, creates conda env "carlaAir"
conda activate carlaAir
./CarlaAir.sh Town10HD        # default Epic quality, 1280x720, both ports
```

Once the launcher prints `CarlaAir is ready!`, both ports are listening
and any `carla` / `airsim` Python client (or the eval suite in
`../eval/`) can connect. The launcher auto-spawns 10 vehicles + 10
pedestrians for traffic flow; pass `--kill` to shut down.

## Drone controls (in-window)

| Key | Action |
|---|---|
| WASD | Translate forward / left / back / right |
| Space / Shift | Ascend / descend |
| Mouse | Yaw |
| Scroll wheel | Adjust translation speed |
| N | Cycle weather presets |
| P | Toggle physics / noclip |
| H | Show / hide help overlay |
| Tab | Release / capture mouse |
