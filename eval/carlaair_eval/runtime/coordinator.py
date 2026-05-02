"""Cooperation-mode coordinators (C0 / C1 / C2).

Paper §4.1 + Appendix C.2:
  C0  Independent execution. No communication.
  C1  UGV→UAV semantic prompting. UAV outputs only UAV actions.
  C2  (landing only) Bidirectional. UAV action → phase decoder → UGV
       longitudinal speed via Eq. (1):
           v_UGV = v0 (1 + α · 1[approach]  − β · 1[descend])
       v0 = 4.0 m/s, α = 0.25, β = 0.40.

The coordinator is the single object that:
   1) builds the partner cue passed to the policy (per-tick),
   2) decides UGV target speed for the next tick (C2 only),
   3) records timestamps for ECL.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

from ..api.cue import (
    LandingCueState, EscortCueState,
    build_landing_cue, build_escort_cue, CueFormat,
)
from ..api.phase_decoder import LandingPhase, default_phase_decoder, PhaseDecoder
from ..api.policy import PartnerCue, UAVAction
from ..constants import C2_V0_MS, C2_ALPHA_APPROACH, C2_BETA_DESCEND
from ..utils.coords import bearing_range_elevation


# ── C0 ───────────────────────────────────────────────────────────────────
class C0Coordinator:
    name = "C0"
    task = "landing"   # task is set after construction; default placeholder

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
        self._phase = LandingPhase.APPROACH

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
        # body-frame dx/dy for direction quantisation
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
        # Geometry to UGV (used by C1-Num and C1-Oracle-Bearing).
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
    name = "C2"

    def __init__(self, task: str, cue_format: str = CueFormat.SEMANTIC,
                 phase_decoder: Optional[PhaseDecoder] = None,
                 noise_seed: int = 0,
                 oracle_phase_provider: Optional[Callable[[any], str]] = None,
                 noisy_oracle_corruption: float = 0.0,
                 noisy_oracle_seed: int = 0):
        if task != "landing":
            raise ValueError("C2 is defined only for the landing task (paper §4.1).")
        super().__init__(task=task, cue_format=cue_format, noise_seed=noise_seed)
        self._decoder: PhaseDecoder = phase_decoder or default_phase_decoder
        self._oracle_phase = oracle_phase_provider
        self._oracle_noise_p = noisy_oracle_corruption
        self._oracle_rng = random.Random(noisy_oracle_seed)
        self._last_phase = LandingPhase.APPROACH

    def ugv_target_speed(self, scenario_state, last_uav_action) -> float:
        st = scenario_state
        # altitude above bed (positive = drone above bed)
        alt = max(0.0, st.bed_world[2] - st.uav_world[2] + 0.0) * -1.0
        alt = abs(st.uav_world[2] - st.bed_world[2])
        uav_state = {
            "x": st.uav_world[0], "y": st.uav_world[1], "z": st.uav_world[2],
        }

        if self._oracle_phase is not None:
            phase = self._oracle_phase(scenario_state)
            if self._oracle_noise_p > 0 and self._oracle_rng.random() < self._oracle_noise_p:
                # Paper App. C.7: noisy oracle uniformly samples an
                # incorrect phase from the oracle phase vocabulary
                # {approach, descend, hover} (touchdown is not in Eq. 2).
                vocab = [p for p in
                         (LandingPhase.APPROACH, LandingPhase.DESCEND,
                          LandingPhase.HOVER)
                         if p != phase]
                phase = self._oracle_rng.choice(vocab)
        elif last_uav_action is not None:
            phase = self._decoder(last_uav_action, uav_state, alt)
        else:
            phase = LandingPhase.APPROACH

        self._last_phase = phase
        self.update_landing_phase(phase)

        v = C2_V0_MS * (1.0
                        + C2_ALPHA_APPROACH * (1.0 if phase == LandingPhase.APPROACH else 0.0)
                        - C2_BETA_DESCEND   * (1.0 if phase == LandingPhase.DESCEND  else 0.0))
        return max(0.0, v)


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
    if mode == "C2-ORACLE":
        # C2 with VLA-decoded phase replaced by oracle ground-truth phase.
        if "oracle_phase_provider" not in kwargs:
            raise ValueError("C2-Oracle requires an oracle_phase_provider")
        return C2Coordinator(task=task, cue_format=CueFormat.SEMANTIC, **kwargs)
    if mode == "C2-NOISYORACLE":
        if "oracle_phase_provider" not in kwargs:
            raise ValueError("C2-NoisyOracle requires an oracle_phase_provider")
        kwargs.setdefault("noisy_oracle_corruption", 0.30)
        return C2Coordinator(task=task, cue_format=CueFormat.SEMANTIC, **kwargs)
    raise ValueError(f"unknown cooperation mode: {mode}")
