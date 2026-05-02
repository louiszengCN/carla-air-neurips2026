#!/usr/bin/env python3
"""Smoke test for the escort scenario.

Verifies: spawn from trajectory JSON, occlusion sampler, IoU computation,
and per-tick coordinator cue under both occluded/unoccluded states.
"""
from __future__ import annotations

import math
import sys
import time
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from carlaair_eval.scenarios.escort import EscortScenario
from carlaair_eval.runtime.coordinator import build_coordinator
from carlaair_eval.reference.rule_coop_state import RuleCoopStateEscort
from carlaair_eval.api.policy import Observation
from carlaair_eval.utils.iou import yaw_pitch_roll_to_R, project_box_aabb, iou_with_image


def main() -> int:
    print("[smoke] connecting + setting up EscortScenario ...")
    scen = EscortScenario(seed=42).setup()
    print(f"[smoke] ugv = {scen._ugv.type_id} at {scen._ugv.get_location()}")
    print(f"[smoke] sampled occlusions ({len(scen.occlusions)}):")
    for ev in scen.occlusions:
        print(f"          onset={ev.onset_s:5.2f}s  dur={ev.duration_s:5.2f}s  kind={ev.kind}")

    policy = RuleCoopStateEscort()
    policy.reset(task_instruction="smoke", task_name="escort", episode_id=0)
    coord = build_coordinator("C1", task="escort")

    try:
        DURATION = 3.0
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
            scen.step_ugv(tgt_speed)

            state = scen.get_state()
            cue = coord.cue(state)

            obs = Observation(
                sim_time=state.sim_time,
                rgb_forward=state.rgb_forward,
                uav_state={"x": state.uav_world[0], "y": state.uav_world[1],
                           "z": state.uav_world[2], "yaw_rad": state.uav_yaw_rad},
                extras={"ugv_world": state.ugv_world,
                        "ugv_speed_ms": state.ugv_speed_ms},
            )
            action = policy.act(obs, cue)
            scen.apply_uav_action(action)
            last_action = action

            # Compute live IoU using utils
            cam_yaw_deg, cam_pitch_deg, _ = state.cam_yaw_pitch_roll_deg
            R = yaw_pitch_roll_to_R(cam_yaw_deg, cam_pitch_deg, 0.0)
            box = project_box_aabb(state.ugv_corners, state.cam_pose_world,
                                   R, scen.fov_forward, scen.image_w, scen.image_h)
            iou = iou_with_image(box, scen.image_w, scen.image_h)

            if i % 5 == 0:
                ux, uy, uz = state.uav_world
                gx, gy, gz = state.ugv_world
                horiz = math.hypot(gx - ux, gy - uy)
                occ = "OCC" if state.occluded else "vis"
                print(f"[tick {i:02d}] t={state.sim_time:5.2f}s  "
                      f"ugv_v={state.ugv_speed_ms:.2f}  "
                      f"horiz={horiz:.2f}m  alt={uz - gz:+.2f}m  "
                      f"iou={iou:.3f}  state={occ}  "
                      f"cue.fmt={cue.format if cue else None}")
            i += 1

        print(f"[smoke] OK — ran {i} escort ticks.")
        return 0

    except Exception:
        traceback.print_exc()
        return 1
    finally:
        scen.teardown()
        try: policy.close()
        except Exception: pass
        print("[smoke] teardown complete.")


if __name__ == "__main__":
    sys.exit(main())
