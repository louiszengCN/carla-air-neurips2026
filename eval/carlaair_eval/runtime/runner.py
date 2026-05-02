"""Episode runner — single-process, single-tick execution.

Inherits the time alignment guarantee from the carlaAir runtime (paper §3):
UAV and UGV observations are sampled from the same simulation tick, so the
runner does not need to align timestamps explicitly.

Top-level entry points:
  - `run_episode(...)` runs a single episode and returns an `EpisodeTrace`.
  - `run_eval(...)`   loops over (mode × seed × episode), writes JSONL traces,
                       computes metrics, and emits CSV + JSON summaries.
"""
from __future__ import annotations

import importlib
import math
import time
import random
from dataclasses import asdict
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Type

import numpy as np

from ..api.policy import (
    UAVPolicy, UAVAction, VelocityCommand, WaypointCommand,
    TrajectoryCommand, DiscreteCommand, Observation,
)
from ..api.cue import LANDING_BASE_INSTRUCTION, ESCORT_BASE_INSTRUCTION
from ..api.phase_decoder import LandingPhase
from ..constants import (
    LANDING_EPISODE_SECONDS, ESCORT_EPISODE_SECONDS,
    LANDING_TOUCHDOWN_DRIFT_M, LANDING_TOUCHDOWN_HOLD_S,
    DEFAULT_SEEDS, DEFAULT_EPISODES_PER_SEED,
)
from ..scenarios.landing import LandingScenario
from ..scenarios.escort import EscortScenario
from ..utils.iou import (
    iou_with_image, project_box_aabb, yaw_pitch_roll_to_R,
)
from .coordinator import build_coordinator
from .trace import EpisodeTrace, TraceWriter, TraceRecord


# ── helpers ──────────────────────────────────────────────────────────────
def _action_to_payload(action: UAVAction) -> Tuple[str, Dict[str, Any]]:
    if isinstance(action, VelocityCommand):
        return "velocity", asdict(action)
    if isinstance(action, WaypointCommand):
        return "waypoint", asdict(action)
    if isinstance(action, TrajectoryCommand):
        d = asdict(action)
        d["points"] = list(action.points)
        return "trajectory", d
    if isinstance(action, DiscreteCommand):
        return "discrete", asdict(action)
    return "unknown", {}


def _is_target_in_landing_view(scenario_state, fov_deg: float, img_w: int,
                                img_h: int) -> bool:
    """Cargo-bed centre within the camera view AND z>0 (in front)."""
    cam_yaw_deg = math.degrees(scenario_state.uav_yaw_rad)
    R = yaw_pitch_roll_to_R(cam_yaw_deg, -30.0, 0.0)
    box = project_box_aabb(
        scenario_state.bed_corners,
        scenario_state.uav_world,
        R, fov_deg, img_w, img_h,
    )
    return box is not None


def _ugv_in_view_iou(scenario_state, fov_deg: float, img_w: int,
                     img_h: int) -> float:
    cam_yaw_deg, cam_pitch_deg, _ = scenario_state.cam_yaw_pitch_roll_deg
    R = yaw_pitch_roll_to_R(cam_yaw_deg, cam_pitch_deg, 0.0)
    box = project_box_aabb(
        scenario_state.ugv_corners, scenario_state.cam_pose_world,
        R, fov_deg, img_w, img_h,
    )
    return iou_with_image(box, img_w, img_h)


def _build_landing_obs(state, sim_time: float) -> Observation:
    return Observation(
        sim_time=sim_time,
        rgb_forward=state.rgb_forward,
        rgb_downward=state.rgb_downward,
        depth=None,
        imu=None,
        gnss=None,
        uav_state={
            "x": state.uav_world[0], "y": state.uav_world[1], "z": state.uav_world[2],
            "yaw_rad": state.uav_yaw_rad,
            "vx": state.uav_velocity_ned[0],
            "vy": state.uav_velocity_ned[1],
            "vz": state.uav_velocity_ned[2],
        },
        extras={
            "truck_world": state.truck_world,
            "bed_world":   state.bed_world,
        },
    )


def _build_escort_obs(state, sim_time: float) -> Observation:
    return Observation(
        sim_time=sim_time,
        rgb_forward=state.rgb_forward,
        rgb_downward=None,
        depth=None,
        imu=None,
        gnss=None,
        uav_state={
            "x": state.uav_world[0], "y": state.uav_world[1], "z": state.uav_world[2],
            "yaw_rad": state.uav_yaw_rad,
        },
        extras={"ugv_world": state.ugv_world, "ugv_speed_ms": state.ugv_speed_ms},
    )


# ── single-episode driver ────────────────────────────────────────────────
def run_episode(
    *,
    task: str,
    mode: str,
    seed: int,
    episode_id: int,
    policy: UAVPolicy,
    out_dir: Path,
    control_hz: float = 10.0,
    coordinator_kwargs: Optional[Dict[str, Any]] = None,
    scenario_kwargs: Optional[Dict[str, Any]] = None,
) -> EpisodeTrace:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    trace_path = out_dir / f"{task}_{mode}_seed{seed}_ep{episode_id:03d}.jsonl"
    coord_kwargs = dict(coordinator_kwargs or {})
    scen_kwargs = dict(scenario_kwargs or {})

    if task == "landing":
        scen = LandingScenario(seed=seed, **scen_kwargs).setup()
        episode_len = LANDING_EPISODE_SECONDS
        instruction = LANDING_BASE_INSTRUCTION
        if mode.startswith("C2") and "oracle_phase_provider" not in coord_kwargs and (
                mode == "C2-Oracle" or mode == "C2-NoisyOracle"):
            coord_kwargs["oracle_phase_provider"] = lambda st: _oracle_landing_phase(st)
    elif task == "escort":
        scen = EscortScenario(seed=seed, **scen_kwargs).setup()
        episode_len = ESCORT_EPISODE_SECONDS
        instruction = ESCORT_BASE_INSTRUCTION
    else:
        raise ValueError(f"unknown task {task}")

    coord = build_coordinator(mode, task, **coord_kwargs)
    policy.reset(task_instruction=instruction, task_name=task, episode_id=episode_id)

    header = {
        "task": task, "mode": mode, "seed": seed, "episode_id": episode_id,
        "started_at": time.time(),
        "control_hz": control_hz,
        "instruction": instruction,
        "paper_strict_constants": _paper_constants_snapshot(),
    }
    writer = TraceWriter(trace_path, header)

    period = 1.0 / control_hz
    next_tick = time.time()
    tick_index = 0
    last_action: Optional[UAVAction] = None

    landing_in_view_seconds = 0.0
    landing_first_contact_t: Optional[float] = None
    landing_first_contact_pos: Optional[tuple] = None
    landing_after_contact_drift: float = 0.0
    landing_success: bool = False
    landing_terminated_reason: str = ""

    rsr_hold_start: Optional[float] = None
    occlusion_recovery_log: List[Dict[str, Any]] = []
    cur_occlusion_id: Optional[int] = None
    cur_occlusion_onset: Optional[float] = None

    try:
        while True:
            now = time.time()
            if now < next_tick:
                time.sleep(min(0.005, next_tick - now))
                continue
            next_tick += period

            # Step UGV at v0 (or C2-controlled speed) for landing.
            tgt_speed = coord.ugv_target_speed(_safe_state(scen, task), last_action)
            if task == "landing":
                scen.step_truck(tgt_speed)
            else:
                scen.step_ugv(tgt_speed)

            state = scen.get_state()
            if state.sim_time > episode_len:
                landing_terminated_reason = landing_terminated_reason or "time_limit"
                break

            cue_t0 = time.perf_counter()
            cue = coord.cue(state)
            cue_latency_ms = (time.perf_counter() - cue_t0) * 1e3

            # build observation
            if task == "landing":
                obs = _build_landing_obs(state, state.sim_time)
            else:
                obs = _build_escort_obs(state, state.sim_time)

            act_t0 = time.perf_counter()
            action = policy.act(obs, cue)
            act_latency_ms = (time.perf_counter() - act_t0) * 1e3

            scen.apply_uav_action(action)
            last_action = action

            # ── per-task metric bookkeeping ─────────────────────────────
            iou = 0.0
            target_in_view = False
            decoded_phase: Optional[str] = None

            if task == "landing":
                target_in_view = _is_target_in_landing_view(
                    state, scen.fov_forward, scen.image_w, scen.image_h
                )
                if target_in_view:
                    landing_in_view_seconds += period

                # contact detection: UAV xy distance to bed centre + z
                dx = state.uav_world[0] - state.bed_world[0]
                dy = state.uav_world[1] - state.bed_world[1]
                dz = state.uav_world[2] - state.bed_world[2]
                horiz = math.hypot(dx, dy)
                vert = abs(dz)
                # Treat "on bed" as horiz<=1.5 m, vert<=0.4 m (cargo-bed
                # half-extents from _carla_helpers).
                on_bed = (horiz <= 1.5 and vert <= 0.4)
                contact = on_bed
                if contact and landing_first_contact_t is None:
                    landing_first_contact_t = state.sim_time
                    landing_first_contact_pos = (state.uav_world[0], state.uav_world[1])
                if landing_first_contact_t is not None:
                    drift = math.hypot(
                        state.uav_world[0] - landing_first_contact_pos[0],
                        state.uav_world[1] - landing_first_contact_pos[1],
                    )
                    landing_after_contact_drift = max(landing_after_contact_drift, drift)
                    # success check at hold-end
                    if (state.sim_time - landing_first_contact_t) >= LANDING_TOUCHDOWN_HOLD_S:
                        if landing_after_contact_drift <= LANDING_TOUCHDOWN_DRIFT_M and not state.collision:
                            landing_success = True
                            landing_terminated_reason = "landed"
                            _record_tick(writer, tick_index, task, mode, seed, episode_id,
                                         state, target_in_view, iou, cue, decoded_phase,
                                         tgt_speed, action, act_latency_ms, cue_latency_ms,
                                         contact, state.collision)
                            tick_index += 1
                            break

                # decoded phase (also written into trace)
                if hasattr(coord, "_last_phase"):
                    decoded_phase = getattr(coord, "_last_phase", None)

            else:  # escort
                iou = _ugv_in_view_iou(state, scen.fov_forward,
                                       scen.image_w, scen.image_h)
                target_in_view = iou >= 0.15
                # Track per-occlusion recovery
                ev_idx = _which_occlusion(scen, state.sim_time)
                if ev_idx != cur_occlusion_id:
                    # transitions
                    if cur_occlusion_id is not None and cur_occlusion_onset is not None:
                        # closed without recovery
                        occlusion_recovery_log.append({
                            "event_id": cur_occlusion_id,
                            "onset_s": cur_occlusion_onset,
                            "recovered": False,
                            "rat_s": 15.0,  # cap
                        })
                    cur_occlusion_id = ev_idx
                    cur_occlusion_onset = state.sim_time if ev_idx is not None else None
                    rsr_hold_start = None

                if cur_occlusion_id is not None and cur_occlusion_onset is not None:
                    if iou >= 0.15:
                        if rsr_hold_start is None:
                            rsr_hold_start = state.sim_time
                        elif (state.sim_time - rsr_hold_start) >= 0.5:
                            rat = state.sim_time - cur_occlusion_onset
                            occlusion_recovery_log.append({
                                "event_id": cur_occlusion_id,
                                "onset_s": cur_occlusion_onset,
                                "recovered": True,
                                "rat_s": min(rat, 15.0),
                            })
                            cur_occlusion_id = None
                            cur_occlusion_onset = None
                            rsr_hold_start = None
                    else:
                        rsr_hold_start = None

            _record_tick(writer, tick_index, task, mode, seed, episode_id,
                         state, target_in_view, iou, cue, decoded_phase,
                         tgt_speed, action, act_latency_ms, cue_latency_ms,
                         contact=False, collision=getattr(state, "collision", False))
            tick_index += 1

    finally:
        # Close any in-flight occlusion event without recovery.
        if task == "escort" and cur_occlusion_id is not None and cur_occlusion_onset is not None:
            occlusion_recovery_log.append({
                "event_id": cur_occlusion_id,
                "onset_s": cur_occlusion_onset,
                "recovered": False,
                "rat_s": 15.0,
            })
        try:
            scen.teardown()
        except Exception:
            pass
        try:
            policy.close()
        except Exception:
            pass

    footer = {
        "finished_at": time.time(),
        "success": bool(landing_success) if task == "landing" else None,
        "extra": {
            "task": task,
            "tracking_seconds": landing_in_view_seconds if task == "landing" else None,
            "occlusion_log": occlusion_recovery_log if task == "escort" else None,
            "terminated_reason": landing_terminated_reason,
            "occlusions": (
                [{"onset_s": ev.onset_s, "duration_s": ev.duration_s, "kind": ev.kind}
                 for ev in scen.occlusions] if task == "escort" else None
            ),
        },
    }
    writer.close(footer)

    et = EpisodeTrace(
        task=task, mode=mode, seed=seed, episode_id=episode_id,
        started_at=header["started_at"], finished_at=footer["finished_at"],
        success=bool(landing_success) if task == "landing" else False,
        extra=footer["extra"],
    )
    et.records = []   # records live on disk; metrics layer reads them back
    return et


# ── multi-condition driver ───────────────────────────────────────────────
def run_eval(
    *,
    task: str,
    modes: List[str],
    policy_factory: Callable[[], UAVPolicy],
    seeds: List[int],
    episodes_per_seed: int,
    out_dir: Path,
    control_hz: float = 10.0,
    coordinator_kwargs_per_mode: Optional[Dict[str, Dict[str, Any]]] = None,
    scenario_kwargs: Optional[Dict[str, Any]] = None,
) -> Path:
    """Run the full grid (modes × seeds × episodes) and write traces."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    coord_kwargs = coordinator_kwargs_per_mode or {}

    for mode in modes:
        for seed in seeds:
            for ep in range(episodes_per_seed):
                policy = policy_factory()
                run_episode(
                    task=task, mode=mode, seed=seed, episode_id=ep,
                    policy=policy, out_dir=out_dir / mode / f"seed_{seed}",
                    control_hz=control_hz,
                    coordinator_kwargs=coord_kwargs.get(mode, {}),
                    scenario_kwargs=scenario_kwargs or {},
                )
    return out_dir


# ── helpers ──────────────────────────────────────────────────────────────
def _safe_state(scen, task):
    try:
        return scen.get_state()
    except Exception:
        # First tick before any state read — return a dummy holding the
        # nominal speed.
        from ..scenarios.landing import LandingEpisodeState
        from ..scenarios.escort  import EscortEpisodeState
        if task == "landing":
            return LandingEpisodeState()
        return EscortEpisodeState()


def _which_occlusion(scen, sim_time: float) -> Optional[int]:
    for i, ev in enumerate(scen.occlusions):
        if ev.onset_s <= sim_time < ev.onset_s + ev.duration_s:
            return i
    return None


def _record_tick(writer, tick_index, task, mode, seed, episode_id, state,
                 target_in_view, iou, cue, decoded_phase, tgt_speed,
                 action, act_latency_ms, cue_latency_ms, contact, collision):
    kind, payload = _action_to_payload(action)
    if task == "landing":
        rec = TraceRecord(
            sim_time=state.sim_time, wall_time=time.time(),
            tick_index=tick_index, task=task, mode=mode,
            seed=seed, episode_id=episode_id,
            uav_world=tuple(state.uav_world), uav_yaw_rad=state.uav_yaw_rad,
            ugv_world=tuple(state.truck_world), ugv_yaw_rad=state.truck_yaw_rad,
            ugv_speed_ms=state.truck_speed_ms,
            target_in_view=target_in_view, iou=0.0, occluded=False,
            contact=contact, collision=collision,
            bed_world=tuple(state.bed_world),
            cue_text=(cue.text if cue else None),
            cue_format=(cue.format if cue else None),
            decoded_phase=decoded_phase,
            ugv_target_speed_ms=tgt_speed,
            action_kind=kind, action_payload=payload,
            action_latency_ms=act_latency_ms, cue_latency_ms=cue_latency_ms,
        )
    else:
        rec = TraceRecord(
            sim_time=state.sim_time, wall_time=time.time(),
            tick_index=tick_index, task=task, mode=mode,
            seed=seed, episode_id=episode_id,
            uav_world=tuple(state.uav_world), uav_yaw_rad=state.uav_yaw_rad,
            ugv_world=tuple(state.ugv_world), ugv_yaw_rad=state.ugv_yaw_rad,
            ugv_speed_ms=state.ugv_speed_ms,
            target_in_view=target_in_view, iou=iou, occluded=state.occluded,
            contact=False, collision=False,
            cue_text=(cue.text if cue else None),
            cue_format=(cue.format if cue else None),
            decoded_phase=None, ugv_target_speed_ms=tgt_speed,
            action_kind=kind, action_payload=payload,
            action_latency_ms=act_latency_ms, cue_latency_ms=cue_latency_ms,
        )
    writer.write(rec)


def _oracle_landing_phase(state) -> str:
    """Oracle ground-truth phase (paper Eq. 2, App. C.7).

    phi* = approach   if d > 8 m
           descend    if 2 m < d <= 8 m  AND  cos(theta) >= 0.7
           hover      otherwise

    where:
      d     = Euclidean distance from UAV to cargo-bed centre,
      theta = angle between UAV velocity vector and the (NED) vertical.

    The paper's oracle definition does not emit `touchdown`; the touchdown
    transition is captured separately by the LSR metric (App. C.4).
    """
    dx = state.uav_world[0] - state.bed_world[0]
    dy = state.uav_world[1] - state.bed_world[1]
    dz = state.uav_world[2] - state.bed_world[2]
    d = math.sqrt(dx * dx + dy * dy + dz * dz)

    if d > 8.0:
        return LandingPhase.APPROACH

    vx, vy, vz = state.uav_velocity_ned
    v_norm = math.sqrt(vx * vx + vy * vy + vz * vz)
    # NED vertical (down) unit vector is (0, 0, +1); cos(theta) = vz / |v|.
    cos_theta = (vz / v_norm) if v_norm > 1e-6 else 0.0

    if 2.0 < d <= 8.0 and cos_theta >= 0.7:
        return LandingPhase.DESCEND

    return LandingPhase.HOVER


def _paper_constants_snapshot() -> Dict[str, Any]:
    from .. import constants as C
    return {k: getattr(C, k) for k in dir(C) if k.isupper() and not k.startswith("_")}
