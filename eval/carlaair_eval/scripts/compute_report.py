"""Compute the paper-format report from a directory of trace JSONLs.

Output:
  - <out>/landing_main.csv       (Table 2 schema)
  - <out>/landing_main.json      (with 95% CI per cell)
  - <out>/escort_main.csv        (Table 4 schema)
  - <out>/timing.csv             (DF / ECL per mode)

Schema chosen so a reviewer's code agent can diff against the paper tables.
"""
from __future__ import annotations

import csv
import json
import statistics
from pathlib import Path
from typing import Dict, List, Sequence

from ..metrics.landing import compute_landing_metrics, summarise_landing
from ..metrics.escort import compute_escort_metrics, summarise_escort
from ..metrics.timing import compute_timing_metrics, aggregate_timing
from ..metrics.stats import bootstrap_ci, sign_test, per_seed_mean


def _gather_traces(out_dir: Path, mode: str, seed: int) -> List[Path]:
    return sorted((out_dir / mode / f"seed_{seed}").glob("*.jsonl"))


def compute_report(out_dir: Path, task: str, modes: Sequence[str],
                   seeds: Sequence[int]) -> Dict:
    out_dir = Path(out_dir)
    summary: Dict = {"task": task, "modes": {}}

    if task == "landing":
        # Pre-compute LSR(C0) seeds for CG.
        lsr_c0_per_seed: List[List[float]] = []
        if "C0" in modes:
            for s in seeds:
                vals = []
                for path in _gather_traces(out_dir, "C0", s):
                    m = compute_landing_metrics(path)
                    vals.append(1.0 if m.landing_success else 0.0)
                lsr_c0_per_seed.append(vals)
            mean_lsr_c0 = statistics.mean([v for s in lsr_c0_per_seed for v in s]) if any(lsr_c0_per_seed) else 0.0
        else:
            mean_lsr_c0 = 0.0

        for mode in modes:
            tsr_seeds, lsr_seeds, ccr_seeds = [], [], []
            for s in seeds:
                tsr_v, lsr_v = [], []
                for path in _gather_traces(out_dir, mode, s):
                    m = compute_landing_metrics(path)
                    tsr_v.append(1.0 if m.tracking_success else 0.0)
                    lsr_v.append(1.0 if m.landing_success else 0.0)
                tsr_seeds.append(tsr_v)
                lsr_seeds.append(lsr_v)

            tsr_pt, tsr_lo, tsr_hi = bootstrap_ci(tsr_seeds)
            lsr_pt, lsr_lo, lsr_hi = bootstrap_ci(lsr_seeds)
            ccr = lsr_pt / max(tsr_pt, 0.05)
            cg = lsr_pt - mean_lsr_c0

            # 95% CI on CG via bootstrap on per-seed deltas.
            cg_seeds = [[v - mean_lsr_c0 for v in s] for s in lsr_seeds]
            cg_pt, cg_lo, cg_hi = bootstrap_ci(cg_seeds)

            summary["modes"][mode] = {
                "TSR": {"point": tsr_pt, "ci_low": tsr_lo, "ci_high": tsr_hi,
                        "std": _seed_std(tsr_seeds)},
                "LSR": {"point": lsr_pt, "ci_low": lsr_lo, "ci_high": lsr_hi,
                        "std": _seed_std(lsr_seeds)},
                "CCR": {"point": ccr},
                "CG":  {"point": cg_pt, "ci_low": cg_lo, "ci_high": cg_hi},
                "n_episodes": sum(len(s) for s in lsr_seeds),
            }

        # CSV
        with (out_dir / "landing_main.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["mode", "TSR_mean", "TSR_std",
                        "LSR_mean", "LSR_std",
                        "CCR_mean",
                        "CG_mean", "CG_ci_low", "CG_ci_high",
                        "n_episodes"])
            for mode, d in summary["modes"].items():
                w.writerow([
                    mode,
                    f"{d['TSR']['point']:.3f}", f"{d['TSR']['std']:.3f}",
                    f"{d['LSR']['point']:.3f}", f"{d['LSR']['std']:.3f}",
                    f"{d['CCR']['point']:.3f}",
                    f"{d['CG']['point']:.3f}", f"{d['CG']['ci_low']:.3f}", f"{d['CG']['ci_high']:.3f}",
                    d["n_episodes"],
                ])
        (out_dir / "landing_main.json").write_text(json.dumps(summary, indent=2))

    elif task == "escort":
        for mode in modes:
            rsr_seeds, rat_seeds = [], []
            for s in seeds:
                rsr_v, rat_v = [], []
                for path in _gather_traces(out_dir, mode, s):
                    m = compute_escort_metrics(path)
                    if m.n_events > 0:
                        rsr_v.append(m.n_recovered / m.n_events)
                        rat_v.extend(m.rats)
                rsr_seeds.append(rsr_v)
                rat_seeds.append(rat_v)
            rsr_pt, rsr_lo, rsr_hi = bootstrap_ci(rsr_seeds)
            rat_pt, rat_lo, rat_hi = bootstrap_ci(rat_seeds)
            summary["modes"][mode] = {
                "RSR": {"point": rsr_pt, "ci_low": rsr_lo, "ci_high": rsr_hi,
                        "std": _seed_std(rsr_seeds)},
                "RAT": {"point": rat_pt, "ci_low": rat_lo, "ci_high": rat_hi,
                        "std": _seed_std(rat_seeds)},
                "n_episodes": sum(len(s) for s in rsr_seeds),
            }
        with (out_dir / "escort_main.csv").open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["mode", "RSR_mean", "RSR_std",
                        "RAT_mean", "RAT_std",
                        "n_episodes"])
            for mode, d in summary["modes"].items():
                w.writerow([
                    mode,
                    f"{d['RSR']['point']:.3f}", f"{d['RSR']['std']:.3f}",
                    f"{d['RAT']['point']:.2f}", f"{d['RAT']['std']:.2f}",
                    d["n_episodes"],
                ])
        (out_dir / "escort_main.json").write_text(json.dumps(summary, indent=2))

    # Timing — per-mode median + IQR (P25, P75) + P95, matching paper Tbl. C.7.
    timing_rows = []
    for mode in modes:
        per_ep = []
        for s in seeds:
            for path in _gather_traces(out_dir, mode, s):
                per_ep.append(compute_timing_metrics(path))
        agg = aggregate_timing(per_ep)
        agg["mode"] = mode
        timing_rows.append(agg)
    with (out_dir / "timing.csv").open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["mode", "DF_hz_median", "DF_hz_IQR",
                    "ECL_ms_median", "ECL_ms_P25_P75", "ECL_ms_P95",
                    "n_episodes"])
        for r in timing_rows:
            w.writerow([
                r["mode"],
                f"{r['DF_hz_median']:.2f}",
                f"{r['DF_hz_p25']:.2f}-{r['DF_hz_p75']:.2f}",
                f"{r['ECL_ms_median']:.0f}",
                f"{r['ECL_ms_p25']:.0f}-{r['ECL_ms_p75']:.0f}",
                f"{r['ECL_ms_p95']:.0f}",
                r["n_episodes"],
            ])
    summary["timing"] = timing_rows
    return summary


def _seed_std(per_seed: List[List[float]]) -> float:
    means = [statistics.mean(s) if s else 0.0 for s in per_seed]
    if len(means) <= 1:
        return 0.0
    return statistics.stdev(means)
