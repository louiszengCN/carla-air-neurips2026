"""Timing metrics: DF (decision frequency) and ECL (effective coordination
latency).

Paper App. C.7 (Tbl. C.7 caption):
  "DF: realized decision frequency (Hz). ECL (Effective Coordination
   Latency): delay from policy inference completion to partner-side
   controller consumption; excludes UAV actuator delay."

Operational definition used by this suite:

  DF_episode  = 1 / median(Δt_tick) within the episode
  ECL_tick    = Δt_tick_next + cue_build_time_next + policy_inference_time_next
  ECL_episode = median over ticks within the episode

The paper reports values as "episode-level medians with IQR (P25–P75)
over 50 episodes per method", so the per-episode reducer is the median
and the across-episode reducer is also the median + IQR. We expose both.
"""
from __future__ import annotations

import statistics
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from ..runtime.trace import load_trace


def _percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * p
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    frac = k - lo
    return s[lo] * (1 - frac) + s[hi] * frac


def compute_timing_metrics(trace_path: Path) -> Dict[str, float]:
    """Per-episode timing reducer: median Δt → DF, median (Δt + cue + act)
    → ECL. Returns 0 for empty / single-tick traces.
    """
    et = load_trace(trace_path)
    if len(et.records) < 2:
        return {"DF_hz": 0.0, "ECL_ms": 0.0, "ticks": len(et.records)}

    dts = []
    for a, b in zip(et.records[:-1], et.records[1:]):
        dt = b.sim_time - a.sim_time
        if dt > 0:
            dts.append(dt)
    if not dts:
        return {"DF_hz": 0.0, "ECL_ms": 0.0, "ticks": len(et.records)}

    median_dt = statistics.median(dts)
    df = 1.0 / median_dt if median_dt > 0 else 0.0

    # Per-tick ECL = next tick interval + cue + act inference latencies.
    # The path is: partner-side state at tick t → cue → UAV inference at
    # tick t+1 → action applied; the partner consumes that action in its
    # control step at tick t+2. Within a single-process tick this equals
    # one inter-tick interval plus the cue + act construction times.
    ecl_per_tick: List[float] = []
    for r in et.records:
        ecl_per_tick.append(median_dt * 1e3 + r.action_latency_ms + r.cue_latency_ms)
    ecl = statistics.median(ecl_per_tick)

    return {"DF_hz": df, "ECL_ms": ecl, "ticks": len(et.records)}


def aggregate_timing(per_episode: Iterable[Dict[str, float]]) -> Dict[str, float]:
    """Across-episode reducer: median + IQR (P25, P75), matching the
    paper's reporting convention (Tbl. C.7).
    """
    eps = [e for e in per_episode if e.get("ticks", 0) > 1]
    if not eps:
        return {
            "DF_hz_median": 0.0, "DF_hz_p25": 0.0, "DF_hz_p75": 0.0,
            "ECL_ms_median": 0.0, "ECL_ms_p25": 0.0, "ECL_ms_p75": 0.0,
            "ECL_ms_p95": 0.0, "n_episodes": 0,
        }
    dfs  = [e["DF_hz"]  for e in eps]
    ecls = [e["ECL_ms"] for e in eps]
    return {
        "DF_hz_median":  statistics.median(dfs),
        "DF_hz_p25":     _percentile(dfs, 0.25),
        "DF_hz_p75":     _percentile(dfs, 0.75),
        "ECL_ms_median": statistics.median(ecls),
        "ECL_ms_p25":    _percentile(ecls, 0.25),
        "ECL_ms_p75":    _percentile(ecls, 0.75),
        "ECL_ms_p95":    _percentile(ecls, 0.95),
        "n_episodes":    len(eps),
    }
