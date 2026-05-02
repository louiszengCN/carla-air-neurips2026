#!/usr/bin/env python3
"""Smoke test: connect to a running CarlaAir, set up the landing scenario,
run ~3 s of ticks with the Rule-Coop-State reference, then tear down.

Run from the `eval/` directory (so `carlaair_eval` resolves):

    python scripts/smoke_test.py
"""
from __future__ import annotations

import math
import sys
import time
import traceback
from pathlib import Path

# Make the package importable when running from the eval/ directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carlaair_eval.scenarios.landing import LandingScenario
from carlaair_eval.runtime.coordinator import build_coordinator
from carlaair_eval.reference.rule_coop_state import RuleCoopStateLanding
from carlaair_eval.api.policy import VelocityCommand


def main() -> int:
    print("[smoke] connecting + setting up LandingScenario ...")
    scen = LandingScenario(seed=0).setup()
    print(f"[smoke] truck = {scen._truck.type_id} at {scen._truck.get_location()}")
    print(f"[smoke] CARLA→NED offset = ({scen._ox:.2f}, {scen._oy:.2f}, {scen._oz:.2f})")

    policy = RuleCoopStateLanding()
    policy.reset(task_instruction="smoke", task_name="landing", episode_id=0)
    coord = build_coordinator("C1", task="landing")

    try:
        DURATION = 3.0   # seconds
        period = 1.0 / 10.0
        next_t = time.time()
        last_action = None
        t0 = time.time()
        i = 0

        while time.time() - t0 < DURATION:
            now = time.time()
            if now < next_t:
                time.sleep(min(0.005, next_t - now))
                continue
            next_t += period

            tgt_speed = coord.ugv_target_speed(None, last_action)
            scen.step_truck(tgt_speed)

            state = scen.get_state()
            cue = coord.cue(state)
            from carlaair_eval.api.policy import Observation
            obs = Observation(
                sim_time=state.sim_time,
                rgb_forward=state.rgb_forward,
                rgb_downward=state.rgb_downward,
                uav_state={"x": state.uav_world[0], "y": state.uav_world[1],
                           "z": state.uav_world[2], "yaw_rad": state.uav_yaw_rad,
                           "vx": state.uav_velocity_ned[0],
                           "vy": state.uav_velocity_ned[1],
                           "vz": state.uav_velocity_ned[2]},
                extras={"truck_world": state.truck_world,
                        "bed_world":   state.bed_world},
            )
            action = policy.act(obs, cue)
            scen.apply_uav_action(action)
            last_action = action

            if i % 5 == 0:
                bx, by, bz = state.bed_world
                ux, uy, uz = state.uav_world
                horiz = math.hypot(bx - ux, by - uy)
                print(f"[tick {i:02d}] t={state.sim_time:5.2f}s   "
                      f"truck_v={state.truck_speed_ms:.2f} m/s   "
                      f"horiz_to_bed={horiz:.2f}m   "
                      f"alt_diff={uz - bz:+.2f}m   "
                      f"cue.fmt={cue.format if cue else None}   "
                      f"act.vz={action.vz:+.2f}")
            i += 1

        print(f"[smoke] OK — ran {i} ticks of landing scenario.")
        return 0

    except Exception:
        traceback.print_exc()
        return 1
    finally:
        scen.teardown()
        try:
            policy.close()
        except Exception:
            pass
        print("[smoke] teardown complete.")


if __name__ == "__main__":
    sys.exit(main())
