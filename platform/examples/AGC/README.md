# AGC — Air-Ground Convoy Scenario

A self-contained demo: a drone rides on a cargo truck's roof, takes off and
escorts the truck in flight, then returns and lands back on the truck — all
while the truck is in motion.

## Files

| File | What it is |
|---|---|
| `play.sh`                | One-shot launcher (starts CarlaAir if needed, then runs the scenario) |
| `air_ground_convoy.py`   | The scenario — state machine, per-frame control, 4-panel pygame view |
| `README.md`              | This file |

## Requirements

- **CarlaAir v0.1.7** installed; `AGC/` must live inside the install directory
  (so the launcher can find `../CarlaAir.sh`).
- `carlaAir` conda env with `carla`, `airsim`, `pygame`, `numpy`
  (created once via `../env_setup/setup_env.sh`).

## Run

```bash
cd AGC
./play.sh                      # 43-second demo
./play.sh --duration 60        # longer run
./play.sh --truck-speed 6.0    # slower truck (m/s)
```

The launcher detects whether CarlaAir is already running — if it is, the
scenario attaches to the existing session; otherwise it launches CarlaAir at
Epic quality on Town10HD and waits for both ports (`2000` CARLA, `41451`
AirSim) before spawning the scene.

## Scenario timeline

| Phase | Time (s) | What happens |
|---|---|---|
| MOUNTED    | 0.0 – 3.0  | Drone pose-locked to truck roof, both stationary |
| FOLLOWING  | 3.0 – 8.0  | Truck accelerates on direct per-frame P-control (target 29 km/h); drone still pose-locked |
| TAKING_OFF | 8.0 – 13.0 | Drone arms, vertical climb to 8 m above roof |
| ESCORTING  | 13.0 – 25.0 | Drone flies in formation directly above truck (feed-forward truck velocity + P-correction) |
| LANDING    | 25.0 – 35.0 | Drone descends smoothly along an ease-out curve; snap-locks to the roof the moment it enters the mount zone |
| LANDED     | 35.0 – 43.0 | Drone rigidly pose-locked on the roof; truck keeps driving 8 s |

## 4-panel pygame layout

```
┌─────────────────┬─────────────────┐
│  Truck Chase    │ Truck Rear-view │
│ (behind truck)  │ (watching drone)│
├─────────────────┼─────────────────┤
│  Drone FPV      │ Drone 3rd-Person│
│ (nose, -30°)    │ (behind + above)│
└─────────────────┴─────────────────┘
```

## Design notes

- **Truck motion**: direct `apply_control()` per frame with a P-controller on
  speed and a waypoint-lookahead P-controller on heading. Replaces Traffic
  Manager (its 20 Hz discrete updates + bang-bang throttle caused visible
  stutter on the heavy HGV).
- **Drone mount**: pose-lock via `simSetVehiclePose(ignore_collision=True)` at
  truck-local `(0, 0, 3.75) m`. The CARLA drone actor has `simulate_physics=
  False`, so it can never disturb the truck's physics.
- **Landing**: feed-forward velocity control (cmd = truck_velocity + Kp·error)
  tracks the moving target. Once the drone is within the mount zone
  (`hor < 0.5 m, vert < 0.35 m`) or phase progress hits 0.99, it snap-locks.
- **Cameras**: 4 CARLA RGB cameras (640×360). Truck rear-view has FOV 130 and
  tilt 22° so the drone stays framed during its full takeoff / escort / land
  arc. Drone FPV has FOV 120 and tilt –30° so the truck stays visible below.

## Exit / cleanup

- `ESC` in the pygame window — quits early, destroys actors, disarms drone.
- The scenario auto-ends at `--duration` seconds.
- CarlaAir itself stays running (you can run `./play.sh` again to replay).
- To stop CarlaAir: `../CarlaAir.sh --kill`.
