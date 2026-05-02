"""Cue builders. Prompt strings are taken VERBATIM from the paper.

Paper references:
  - Tbl. 6  (App. C.3, "Full prompt protocol"): C0 / C1-Sem / C2 base prompts
                                                + C1-Oracle-Bearing example
  - Tbl. C.5 (App. C.6, "Prompt variants for C1 ablation"):
            C1-Sem / C1-Num / C1-Noisy / C1-Oracle-Bearing for both tasks

A unit test (tests/test_prompts.py) asserts that build outputs equal the
paper strings character-for-character for every canonical example.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Dict, Optional

from .policy import PartnerCue


# ── verbatim base instructions (paper Tbl. 6) ───────────────────────────
LANDING_BASE_INSTRUCTION = (
    "Follow the moving truck, keep it in view, align above the flat rear "
    "cargo bed, and land safely on the cargo bed."
)
ESCORT_BASE_INSTRUCTION = "Follow the moving truck and keep it in view."

# Paper Tbl. 6 row "Landing / C1-Oracle-Bearing" uses a SHORTER base instruction
# (no "on the cargo bed" suffix).  Reproduced exactly so this variant is
# byte-identical to the printed prompt template.
LANDING_BASE_INSTRUCTION_ORACLE_BEARING = (
    "Follow the moving truck, keep it in view, align above the flat rear "
    "cargo bed, and land safely."
)


# ── cue formats (Tbl. C.5 prompt-format ablation) ───────────────────────
class CueFormat:
    SEMANTIC       = "semantic"        # C1-Sem (default)
    NUMERIC        = "numeric"         # C1-Num
    NOISY          = "noisy"           # C1-Noisy
    ORACLE_BEARING = "oracle_bearing"  # C1-Oracle-Bearing


# ── label vocabularies ──────────────────────────────────────────────────
def _direction_label(dx: float, dy: float) -> str:
    """Quantise (dx, dy) in UAV body frame (forward=+x, right=+y) to one
    of {forward, forward-left, forward-right, left, right, behind,
    behind-left, behind-right}."""
    ang = math.degrees(math.atan2(dy, dx))
    a = ((ang + 360.0) % 360.0)
    if a < 22.5 or a >= 337.5:   return "forward"
    if a < 67.5:                  return "forward-right"
    if a < 112.5:                 return "right"
    if a < 157.5:                 return "behind-right"
    if a < 202.5:                 return "behind"
    if a < 247.5:                 return "behind-left"
    if a < 292.5:                 return "left"
    return "forward-left"


def _speed_label(speed_ms: float) -> str:
    """Speed bins. The "nearly stopped" label appears in the paper Tbl. C.5
    C1-Noisy example; "moving slowly" appears in Tbl. 6 C1-Sem example."""
    if speed_ms < 0.5:  return "nearly stopped"
    if speed_ms < 1.5:  return "stopped"
    if speed_ms < 3.5:  return "moving slowly"
    if speed_ms < 6.0:  return "moving"
    return "moving fast"


_SPEED_LABEL_VOCAB = (
    "nearly stopped", "stopped", "moving slowly", "moving", "moving fast",
)
_DIRECTION_LABEL_VOCAB = (
    "forward", "forward-left", "forward-right",
    "left", "right",
    "behind", "behind-left", "behind-right",
)
_LANDING_PHASE_VOCAB = ("approach", "descend", "hover", "touchdown")
_ESCORT_PHASE_VOCAB  = ("occlusion recovery", "normal escort", "tracking")
# Paper Tbl. C.5 C1-Noisy escort row uses "rear-left side"; we keep the full
# 8-way vocabulary to match the noise-injection space.
_ESCORT_REAPPEAR_VOCAB = (
    "forward side", "forward-left side", "forward-right side",
    "left side", "right side",
    "rear side", "rear-left side", "rear-right side",
)


# ── landing cue ─────────────────────────────────────────────────────────
@dataclass
class LandingCueState:
    bed_dx: float              # cargo-bed offset in UAV body frame (+x forward)
    bed_dy: float
    bed_range_m: float
    bed_bearing_deg: float     # 0..360 (UAV body frame, 0 = forward, +CW)
    bed_elevation_deg: float
    truck_speed_ms: float
    truck_heading_deg: float = 0.0   # used by C1-Num (paper Tbl. C.5)
    phase: str = "approach"          # approach | descend | hover | touchdown


def build_landing_cue(state: LandingCueState, fmt: str = CueFormat.SEMANTIC,
                      noise_rng: Optional[random.Random] = None) -> PartnerCue:
    """Build the C1 landing cue. Output text matches:
       - Tbl. 6  for the SEMANTIC and ORACLE_BEARING canonical examples,
       - Tbl. C.5 for the NUMERIC and NOISY canonical examples,
    character-for-character (verified by tests/test_prompts.py).
    """
    if fmt == CueFormat.SEMANTIC:
        direction = _direction_label(state.bed_dx, state.bed_dy)
        spd = _speed_label(state.truck_speed_ms)
        text = (
            f"{LANDING_BASE_INSTRUCTION} Assistant hint: the cargo bed is "
            f"{direction}. The truck is {spd}. Current phase: {state.phase}. "
            f"Use the hint only to choose your next UAV action."
        )

    elif fmt == CueFormat.NUMERIC:
        # Paper Tbl. C.5 row "Landing / C1-Num":
        #   "State: truck speed = 2.0 m/s; truck heading = 15 deg;
        #    relative bearing to cargo bed = -30 deg;
        #    relative distance to cargo bed = 8.0 m; phase = approach."
        # Bearing is signed in [-180, 180].
        bearing_signed = ((state.bed_bearing_deg + 180.0) % 360.0) - 180.0
        text = (
            f"{LANDING_BASE_INSTRUCTION} State: "
            f"truck speed = {state.truck_speed_ms:.1f} m/s; "
            f"truck heading = {int(round(state.truck_heading_deg))} deg; "
            f"relative bearing to cargo bed = {int(round(bearing_signed))} deg; "
            f"relative distance to cargo bed = {state.bed_range_m:.1f} m; "
            f"phase = {state.phase}."
        )

    elif fmt == CueFormat.NOISY:
        # Paper App. C.6 protocol: each of {direction, motion, phase} fields
        # is independently corrupted with probability 0.30 by uniform
        # sampling from its full label vocabulary.
        rng = noise_rng or random.Random()
        direction = _direction_label(state.bed_dx, state.bed_dy)
        spd = _speed_label(state.truck_speed_ms)
        phase = state.phase
        if rng.random() < 0.30:
            direction = rng.choice(_DIRECTION_LABEL_VOCAB)
        if rng.random() < 0.30:
            spd = rng.choice(_SPEED_LABEL_VOCAB)
        if rng.random() < 0.30:
            phase = rng.choice(_LANDING_PHASE_VOCAB)
        text = (
            f"{LANDING_BASE_INSTRUCTION} Assistant hint: the cargo bed is "
            f"{direction}. The truck is {spd}. Current phase: {phase}. "
            f"Use the hint only to choose your next UAV action."
        )

    elif fmt == CueFormat.ORACLE_BEARING:
        text = (
            f"{LANDING_BASE_INSTRUCTION_ORACLE_BEARING} State update: "
            f"cargo bed at bearing {int(round(state.bed_bearing_deg))}°, "
            f"range {state.bed_range_m:.1f} m, "
            f"elevation {int(round(state.bed_elevation_deg)):+d}°. "
            f"Phase: {state.phase}. "
            f"Use the state update only to choose your next UAV action."
        )

    else:
        raise ValueError(f"unknown cue format: {fmt}")

    return PartnerCue(
        text=text,
        structured={
            "task": "landing",
            "phase": state.phase,
            "bed_dx": state.bed_dx,
            "bed_dy": state.bed_dy,
            "bed_range_m": state.bed_range_m,
            "bed_bearing_deg": state.bed_bearing_deg,
            "bed_elevation_deg": state.bed_elevation_deg,
            "truck_speed_ms": state.truck_speed_ms,
            "truck_heading_deg": state.truck_heading_deg,
        },
        format=fmt,
    )


# ── escort (occlusion-recovery) cue ─────────────────────────────────────
@dataclass
class EscortCueState:
    occlusion_active: bool
    occlusion_kind: str        # bridge | building | artifact | none
    motion_intent: str         # e.g. "continues forward"
    reappear_direction: str    # e.g. "forward-right side"
    truck_speed_ms: float = 0.0
    reappear_bearing_deg: float = 0.0
    reappear_range_m: float = 0.0
    reappear_elevation_deg: float = 0.0
    phase: str = "occlusion recovery"


def build_escort_cue(state: EscortCueState, fmt: str = CueFormat.SEMANTIC,
                     noise_rng: Optional[random.Random] = None) -> PartnerCue:
    """Build the C1 escort cue. Output text matches:
       - Tbl. 6  for the SEMANTIC canonical example,
       - Tbl. C.5 for the NUMERIC, NOISY, and ORACLE_BEARING canonical
                  examples,
    character-for-character (verified by tests/test_prompts.py).
    """
    if fmt == CueFormat.SEMANTIC:
        if state.occlusion_active:
            text = (
                f"{ESCORT_BASE_INSTRUCTION} Assistant hint: the truck is "
                f"temporarily occluded by the {state.occlusion_kind}. "
                f"The truck {state.motion_intent} and will reappear on "
                f"the {state.reappear_direction}. "
                f"Current phase: {state.phase}. "
                f"Use the hint only to recover visual contact."
            )
        else:
            text = (
                f"{ESCORT_BASE_INSTRUCTION} Assistant hint: the truck is "
                f"currently visible. Current phase: {state.phase}. "
                f"Use the hint only to keep the truck in view."
            )

    elif fmt == CueFormat.NUMERIC:
        # Paper Tbl. C.5 row "Escort / C1-Num":
        #   "State: occlusion = true; truck speed = 2.0 m/s;
        #    expected reappearance bearing = 35 deg;
        #    expected reappearance distance = 9.4 m;
        #    phase = occlusion recovery."
        bearing_signed = ((state.reappear_bearing_deg + 180.0) % 360.0) - 180.0
        text = (
            f"{ESCORT_BASE_INSTRUCTION} State: "
            f"occlusion = {'true' if state.occlusion_active else 'false'}; "
            f"truck speed = {state.truck_speed_ms:.1f} m/s; "
            f"expected reappearance bearing = {int(round(bearing_signed))} deg; "
            f"expected reappearance distance = {state.reappear_range_m:.1f} m; "
            f"phase = {state.phase}."
        )

    elif fmt == CueFormat.NOISY:
        # 30% independent label corruption (occlusion bool is preserved).
        rng = noise_rng or random.Random()
        kind = state.occlusion_kind
        reappear = state.reappear_direction
        phase = state.phase
        # The Tbl. C.5 noisy escort example omits the "by the X" clause
        # ("the truck is temporarily occluded.") — preserve that surface
        # form when corruption drops the occluder identity.
        kind_drop = rng.random() < 0.30
        if rng.random() < 0.30:
            reappear = rng.choice(_ESCORT_REAPPEAR_VOCAB)
        if rng.random() < 0.30:
            phase = rng.choice(_ESCORT_PHASE_VOCAB)

        if state.occlusion_active:
            occluder_clause = "" if kind_drop else f" by the {kind}"
            text = (
                f"{ESCORT_BASE_INSTRUCTION} Assistant hint: the truck is "
                f"temporarily occluded{occluder_clause}. "
                f"The truck will reappear on the {reappear}. "
                f"Current phase: {phase}. "
                f"Use the hint only to recover visual contact."
            )
        else:
            text = (
                f"{ESCORT_BASE_INSTRUCTION} Assistant hint: the truck is "
                f"currently visible. Current phase: {phase}. "
                f"Use the hint only to keep the truck in view."
            )

    elif fmt == CueFormat.ORACLE_BEARING:
        # Paper Tbl. C.5 row "Escort / C1-Oracle-Bearing":
        #   "State update: UGV at bearing 35°, range 9.4 m, elevation -12°.
        #    Phase: occlusion recovery.
        #    Use the state update only to recover visual contact."
        text = (
            f"{ESCORT_BASE_INSTRUCTION} State update: "
            f"UGV at bearing {int(round(state.reappear_bearing_deg))}°, "
            f"range {state.reappear_range_m:.1f} m, "
            f"elevation {int(round(state.reappear_elevation_deg)):+d}°. "
            f"Phase: {state.phase}. "
            f"Use the state update only to recover visual contact."
        )

    else:
        raise ValueError(f"unknown cue format: {fmt}")

    return PartnerCue(
        text=text,
        structured={
            "task": "escort",
            "occlusion_active": state.occlusion_active,
            "occlusion_kind": state.occlusion_kind,
            "motion_intent": state.motion_intent,
            "reappear_direction": state.reappear_direction,
            "truck_speed_ms": state.truck_speed_ms,
            "reappear_bearing_deg": state.reappear_bearing_deg,
            "reappear_range_m": state.reappear_range_m,
            "reappear_elevation_deg": state.reappear_elevation_deg,
            "phase": state.phase,
        },
        format=fmt,
    )
