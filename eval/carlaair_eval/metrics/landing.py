"""Landing metrics: TSR, LSR, CCR, CG.

Definitions from Appendix C.4:
  TSR  truck visible to UAV camera for >= K=3 s cumulative time before episode end
  LSR  UAV lands on rear cargo bed within 60 s + remains stable (no further
       displacement > 0.3 m within 2 s of first contact) + no collision
  CCR  LSR / max(TSR, 0.05)
  CG(Ck)  LSR(Ck) - LSR(C0)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from ..constants import (
    TSR_VISIBLE_SECONDS_K, CCR_EPSILON,
    LANDING_TOUCHDOWN_DRIFT_M, LANDING_TOUCHDOWN_HOLD_S,
)
from ..runtime.trace import load_trace, EpisodeTrace


@dataclass
class LandingEpisodeMetrics:
    seed: int
    episode_id: int
    mode: str
    tracking_seconds: float
    tracking_success: bool
    landing_success: bool
    collision: bool
    terminated_reason: str


# ── per-episode ──────────────────────────────────────────────────────────
def compute_landing_metrics(trace_path: Path) -> LandingEpisodeMetrics:
    et = load_trace(trace_path)

    in_view_seconds = 0.0
    last_t = None
    first_contact_t = None
    first_contact_xy = None
    drift_max = 0.0
    has_collision = False
    landing_success = False

    for r in et.records:
        if last_t is not None:
            dt = max(0.0, r.sim_time - last_t)
        else:
            dt = 0.0
        last_t = r.sim_time

        if r.target_in_view:
            in_view_seconds += dt

        if r.collision:
            has_collision = True

        if r.contact and first_contact_t is None:
            first_contact_t = r.sim_time
            first_contact_xy = (r.uav_world[0], r.uav_world[1])

        if first_contact_t is not None and first_contact_xy is not None:
            drift = math.hypot(r.uav_world[0] - first_contact_xy[0],
                               r.uav_world[1] - first_contact_xy[1])
            drift_max = max(drift_max, drift)
            if (r.sim_time - first_contact_t) >= LANDING_TOUCHDOWN_HOLD_S:
                if drift_max <= LANDING_TOUCHDOWN_DRIFT_M and not has_collision:
                    landing_success = True
                    break

    # Episode-level success may also be set by the runner footer.
    landing_success = landing_success or et.success

    return LandingEpisodeMetrics(
        seed=et.seed,
        episode_id=et.episode_id,
        mode=et.mode,
        tracking_seconds=in_view_seconds,
        tracking_success=in_view_seconds >= TSR_VISIBLE_SECONDS_K,
        landing_success=landing_success,
        collision=has_collision,
        terminated_reason=str(et.extra.get("terminated_reason", "")),
    )


# ── aggregate ────────────────────────────────────────────────────────────
def summarise_landing(metrics: List[LandingEpisodeMetrics],
                      baseline_lsr_c0: float = None) -> Dict[str, float]:
    """Aggregate per-episode metrics → TSR / LSR / CCR / CG.

    `baseline_lsr_c0` is LSR(C0) for the same baseline; required to compute
    CG. For the C0 row itself, pass baseline_lsr_c0 = its own LSR (CG = 0).
    """
    if not metrics:
        return {"TSR": 0.0, "LSR": 0.0, "CCR": 0.0, "CG": 0.0, "n": 0}
    n = len(metrics)
    tsr = sum(1 for m in metrics if m.tracking_success) / n
    lsr = sum(1 for m in metrics if m.landing_success) / n
    ccr = lsr / max(tsr, CCR_EPSILON)
    cg = (lsr - baseline_lsr_c0) if baseline_lsr_c0 is not None else 0.0
    return {"TSR": tsr, "LSR": lsr, "CCR": ccr, "CG": cg, "n": n}
