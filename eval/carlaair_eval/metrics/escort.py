"""Escort metrics: RSR, RAT.

Definitions from Appendix C.4:
  RSR  recovery success rate — visual contact (IoU >= 0.15) sustained
       >= 0.5 s within 15 s of occlusion onset
  RAT  re-acquisition time — onset → first frame satisfying IoU threshold;
       capped at 15 s for non-recovery
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

from ..constants import (
    RSR_IOU_THRESHOLD, RSR_HOLD_SECONDS, RAT_CAP_SECONDS,
)
from ..runtime.trace import load_trace


@dataclass
class EscortEpisodeMetrics:
    seed: int
    episode_id: int
    mode: str
    n_events: int
    n_recovered: int
    rats: List[float]   # per-event re-acquisition time (capped)


def compute_escort_metrics(trace_path: Path) -> EscortEpisodeMetrics:
    et = load_trace(trace_path)
    log = et.extra.get("occlusion_log", []) or []

    rats = [min(float(e.get("rat_s", RAT_CAP_SECONDS)), RAT_CAP_SECONDS)
            for e in log]
    n_events = len(log)
    n_rec = sum(1 for e in log if e.get("recovered"))
    return EscortEpisodeMetrics(
        seed=et.seed, episode_id=et.episode_id, mode=et.mode,
        n_events=n_events, n_recovered=n_rec, rats=rats,
    )


def summarise_escort(metrics: List[EscortEpisodeMetrics]) -> Dict[str, float]:
    """Aggregate per-episode metrics → RSR / RAT."""
    if not metrics:
        return {"RSR": 0.0, "RAT": RAT_CAP_SECONDS, "n": 0}
    total_events = sum(m.n_events for m in metrics)
    total_rec = sum(m.n_recovered for m in metrics)
    rats = [r for m in metrics for r in m.rats]
    rsr = (total_rec / total_events) if total_events > 0 else 0.0
    rat = (sum(rats) / len(rats)) if rats else RAT_CAP_SECONDS
    return {"RSR": rsr, "RAT": rat,
            "n": len(metrics), "events": total_events,
            "recovered": total_rec}
