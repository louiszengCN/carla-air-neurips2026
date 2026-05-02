# CarlaAir Platform

Single-process Unreal Engine runtime that hosts CARLA's ground-vehicle
simulation and AirSim's multirotor flight stack in the same world. Both
client APIs (`carla.Client`, `airsim.MultirotorClient`) connect to the
same UE4 process, and all sensor frames come off the same physics tick.

## Layout

```
platform/
├── CarlaAir.sh             Launcher; opens CARLA on port 2000 and AirSim on 41451.
├── auto_traffic.py         Background traffic and pedestrian spawner used by
│                           CarlaAir.sh after the world is up.
├── AirSimConfig/
│   └── settings.json       AirSim plugin settings; copied to ~/Documents/AirSim/
│                           on first launch.
├── env_setup/
│   ├── setup_env.sh        Creates the carlaAir conda env and installs deps.
│   └── test_env.sh         Verifies that carla, airsim, and ports work.
├── examples/               Demo scripts (see below).
└── source_modifications/   The three upstream files that, when applied on top
                            of CARLA 0.9.16 + AirSim 1.7.0 + UE4.26, produce
                            the unified runtime described in §3 / App. B.
```

## Examples

| Script | What it does |
|---|---|
| `examples/quick_start_showcase.py` | 4-panel demo: drone chase + RGB / depth / semantic / LiDAR BEV. |
| `examples/air_ground_sync.py`      | Side-by-side ground-vehicle and aerial views in the same weather. |
| `examples/sensor_gallery.py`       | 6-panel sensor showcase on a single vehicle. |
| `examples/drive_vehicle.py`        | WASD vehicle control. |
| `examples/walk_pedestrian.py`      | First-person pedestrian control. |
| `examples/switch_maps.py`          | Cycle through maps automatically. |
| `examples/AGC/`                    | Air-Ground Convoy reference scenario: drone rides a truck, takes off, escorts, lands back. The cooperative landing scenario in `eval/` builds on this. |
| `examples/recording/`              | Trajectory record + replay toolkit (vehicle, drone, pedestrian). |
| `examples/trajectories/`           | A pre-recorded UGV trajectory (Town10HD bridge passage) reused by the eval suite to seed the escort scenario. |

## Build / install

This repository ships only the source-side artefacts:

- the launcher script,
- the conda env setup,
- the three modified UE4 files (see `source_modifications/`).

Compiling these against CARLA 0.9.16 + AirSim 1.7.0 + Unreal Engine 4.26 is
the standard CARLA build flow with AirSim added as a UE4 plugin and the
three substitutions applied. The resulting binary is what `CarlaAir.sh`
expects at `CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping`.

For development without rebuilding, drop the source-modified files into an
existing CARLA + AirSim source tree at the paths documented in
`source_modifications/README.md`.

## Run

```bash
bash env_setup/setup_env.sh   # one-time, creates conda env "carlaAir"
conda activate carlaAir
./CarlaAir.sh Town10HD        # default Epic quality, 1280x720, both ports
```

Once the launcher prints `CarlaAir is ready!`, both ports are listening
and any `carla`/`airsim` Python client (or the demo scripts above) can
connect. The launcher auto-spawns 10 vehicles + 10 pedestrians for traffic
flow; pass `--kill` to shut down.

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
