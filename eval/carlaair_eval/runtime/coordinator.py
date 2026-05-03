"""Cooperation-mode coordinators (C0 / C1 / C2).

Paper §4.1 + Appendix C.2:
  C0  Independent execution. No communication.
  C1  UGV→UAV semantic prompting. UAV outputs only UAV actions.
  C2  (landing only) Bidirectional UAV-to-UGV action coupling. The
       magnitude of the UAV's commanded forward velocity is passed
       directly to the UGV's longitudinal controller, with no
       intermediate phase decoder or learned mapping (Eq. 1):

           v_UGV = v0 · clip( ‖v_UAV^fwd‖ / v_ref ,  0.5,  1.5 )

       v0 = 4.0 m/s, v_ref = 2.0 m/s.

The coordinator is the single object that:
   1) builds the partner cue passed to the policy (per-tick),
   2) decides UGV target speed for the next tick (C2 only),
   3) records timestamps for ECL.
"""
from __future__ import annotations

import math
import random
from typing import Optional

from ..api.cue import (
    LandingCueState, EscortCueState,
    build_landing_cue, build_escort_cue, CueFormat,
)
from ..api.policy import (
    PartnerCue, UAVAction,
    VelocityCommand, WaypointCommand, TrajectoryCommand, DiscreteCommand,
)
from ..constants import (
    C2_V0_MS, C2_V_REF_MS, C2_CLIP_LOW, C2_CLIP_HIGH,
)
from ..utils.coords import bearing_range_elevation


# ── C0 ───────────────────────────────────────────────────────────────────
class C0Coordinator:
    name = "C0"
    task = "landing"

    def __init__(self, task: str):
        self.task = task

    def cue(self, scenario_state) -> Optional[PartnerCue]:
        return None

    def ugv_target_speed(self, scenario_state, last_uav_action) -> float:
        return C2_V0_MS


# ── C1 ───────────────────────────────────────────────────────────────────
class C1Coordinator:
    name = "C1"

    def __init__(self, task: str, cue_format: str = CueFormat.SEMANTIC,
                 noise_seed: int = 0):
        self.task = task
        self.cue_format = cue_format
        self._noise_rng = random.Random(noise_seed)
        # `_phase` is a free-form string used by the C1 cue templates to
        # narrate the current landing stage ("approach" / "descend" /
        # "hover" / "touchdown"). It is set externally by the runner if
        # a downstream component decides on a phase label; otherwise it
        # stays at "approach".
        self._phase = "approach"

    def cue(self, scenario_state) -> Optional[PartnerCue]:
        if self.task == "landing":
            return self._landing_cue(scenario_state)
        if self.task == "escort":
            return self._escort_cue(scenario_state)
        return None

    def _landing_cue(self, st) -> PartnerCue:
        bearing, rng_m, elev = bearing_range_elevation(
            st.bed_world, st.uav_world, st.uav_yaw_rad
        )
        c, s = math.cos(-st.uav_yaw_rad), math.sin(-st.uav_yaw_rad)
        rx, ry = st.bed_world[0] - st.uav_world[0], st.bed_world[1] - st.uav_world[1]
        bx = rx * c - ry * s
        by = rx * s + ry * c
        cue_state = LandingCueState(
            bed_dx=bx, bed_dy=by,
            bed_range_m=rng_m,
            bed_bearing_deg=bearing,
            bed_elevation_deg=elev,
            truck_speed_ms=st.truck_speed_ms,
            truck_heading_deg=math.degrees(st.truck_yaw_rad),
            phase=self._phase,
        )
        return build_landing_cue(cue_state, fmt=self.cue_format,
                                 noise_rng=self._noise_rng)

    def _escort_cue(self, st) -> PartnerCue:
        ev = st.active_occlusion
        bearing, rng_m, elev = bearing_range_elevation(
            st.ugv_world, st.uav_world, st.uav_yaw_rad
        )
        cue_state = EscortCueState(
            occlusion_active=st.occluded,
            occlusion_kind=(ev.kind if ev else "none"),
            motion_intent=(ev.motion_intent if ev else "continues forward"),
            reappear_direction=(ev.reappear_direction if ev else "forward side"),
            truck_speed_ms=st.ugv_speed_ms,
            reappear_bearing_deg=bearing,
            reappear_range_m=rng_m,
            reappear_elevation_deg=elev,
            phase=("occlusion recovery" if st.occluded else "tracking"),
        )
        return build_escort_cue(cue_state, fmt=self.cue_format,
                                noise_rng=self._noise_rng)

    def ugv_target_speed(self, scenario_state, last_uav_action) -> float:
        return C2_V0_MS

    def update_landing_phase(self, phase: str) -> None:
        self._phase = phase


# ── C2 (landing only) ────────────────────────────────────────────────────
class C2Coordinator(C1Coordinator):
    """Bidirectional UAV-to-UGV action coupling.

    The UAV side runs identically to C1; in addition, every tick the
    coordinator extracts the magnitude of the UAV's commanded forward
    velocity and feeds it through the Eq. 1 controller to set the next
    UGV longitudinal speed setpoint. There is no phase decoder, no
    learned mapping, and no UAV-side reward shaping — the protocol is
    intentionally a naive form of bidirectional action coupling.
    """
    name = "C2"

    def __init__(self, task: str, cue_format: str = CueFormat.SEMANTIC,
                 noise_seed: int = 0,
                 inference_period_s: float = 1.0):
        if task != "landing":
            raise ValueError("C2 is defined only for the landing task (paper §4.1).")
        super().__init__(task=task, cue_format=cue_format, noise_seed=noise_seed)
        self._inference_period = inference_period_s

    # ── Eq. 1: UGV speed from UAV forward-velocity magnitude ────────────
    def _uav_forward_speed(self, action: Optional[UAVAction],
                           scenario_state) -> float:
        """Return the magnitude of the UAV's commanded forward velocity
        (m/s) for the current tick, derived from the baseline's native
        action interface (paper App. C.2).

        Mapping:
          * VelocityCommand:   sqrt(vx² + vy²)
          * WaypointCommand:   ‖waypoint − current_position‖ / inference_period
          * TrajectoryCommand: ‖p1 − p0‖ / dt
          * DiscreteCommand:   realized UAV forward speed at the same tick
                               (read from scenario_state.uav_velocity_ned)
        """
        if action is None:
            return 0.0

        if isinstance(action, VelocityCommand):
            return math.sqrt(action.vx * action.vx + action.vy * action.vy)

        if isinstance(action, WaypointCommand):
            uav = scenario_state.uav_world
            dx = action.x - uav[0]
            dy = action.y - uav[1]
            return math.sqrt(dx * dx + dy * dy) / max(self._inference_period, 1e-6)

        if isinstance(action, TrajectoryCommand) and len(action.points) >= 2:
            (x0, y0, _), (x1, y1, _) = action.points[0], action.points[1]
            dt = max(action.dt, 1e-6)
            dx = (x1 - x0) / dt
            dy = (y1 - y0) / dt
            return math.sqrt(dx * dx + dy * dy)

        if isinstance(action, DiscreteCommand):
            # For discrete-output baselines the paper specifies the
            # realized UAV forward speed from the simulator at the same
            # tick. We read it off the scenario state's UAV NED velocity.
            v = getattr(scenario_state, "uav_velocity_ned", (0.0, 0.0, 0.0))
            return math.sqrt(v[0] * v[0] + v[1] * v[1])

        return 0.0

    def ugv_target_speed(self, scenario_state, last_uav_action) -> float:
        v_fwd = self._uav_forward_speed(last_uav_action, scenario_state)
        factor = v_fwd / max(C2_V_REF_MS, 1e-6)
        factor = max(C2_CLIP_LOW, min(C2_CLIP_HIGH, factor))
        return C2_V0_MS * factor


# ── factory ──────────────────────────────────────────────────────────────
def build_coordinator(mode: str, task: str, **kwargs):
    mode = mode.upper()
    if mode == "C0":
        return C0Coordinator(task=task)
    if mode == "C1" or mode.startswith("C1-"):
        fmt = {
            "C1": CueFormat.SEMANTIC,
            "C1-SEM": CueFormat.SEMANTIC,
            "C1-NUM": CueFormat.NUMERIC,
            "C1-NOISY": CueFormat.NOISY,
            "C1-ORACLE-BEARING": CueFormat.ORACLE_BEARING,
        }.get(mode, CueFormat.SEMANTIC)
        return C1Coordinator(task=task, cue_format=fmt, **kwargs)
    if mode == "C2":
        return C2Coordinator(task=task, cue_format=CueFormat.SEMANTIC, **kwargs)
    raise ValueError(f"unknown cooperation mode: {mode}")
