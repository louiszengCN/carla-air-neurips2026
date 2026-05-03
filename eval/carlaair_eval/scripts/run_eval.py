#!/usr/bin/env python3
"""Single-command runner — execute one paper condition end to end.

Usage:
    # Run landing main grid (C0 / C1 / C2), 3 seeds × 50 episodes
    python -m carlaair_eval.scripts.run_eval \
        --config carlaair_eval/configs/landing_main.yaml \
        --policy carlaair_eval.examples.dummy_policy:DummyHoverPolicy \
        --out results/

    # Run only with the Rule-Coop-State reference (sanity check the suite)
    python -m carlaair_eval.scripts.run_eval \
        --config carlaair_eval/configs/landing_main.yaml \
        --policy carlaair_eval.reference.rule_coop_state:RuleCoopStateLanding \
        --out results/rule_coop/
"""
from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import yaml

from ..runtime.runner import run_eval
from .compute_report import compute_report


def _load_policy(spec: str):
    if ":" not in spec:
        raise ValueError(f"--policy must be 'module:Class', got: {spec}")
    mod_name, cls_name = spec.split(":", 1)
    mod = importlib.import_module(mod_name)
    return getattr(mod, cls_name)


def main(argv: List[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True, help="Path to a configs/*.yaml")
    p.add_argument("--policy", required=True,
                   help="Policy class spec, e.g. 'mypkg.policy:MyPolicy'")
    p.add_argument("--out", default="results/", help="Output directory")
    p.add_argument("--paper-strict", action="store_true", default=True,
                   help="Lock all paper constants (default: on)")
    p.add_argument("--seeds", nargs="*", type=int, default=None,
                   help="Override seeds (default: from config)")
    p.add_argument("--episodes-per-seed", type=int, default=None,
                   help="Override episodes per seed")
    p.add_argument("--modes", nargs="*", default=None, help="Override modes")
    args = p.parse_args(argv)

    cfg = yaml.safe_load(Path(args.config).read_text())
    task = cfg["task"]
    modes = args.modes or cfg["modes"]
    seeds = args.seeds or cfg["seeds"]
    episodes = args.episodes_per_seed or cfg["episodes_per_seed"]
    control_hz = float(cfg.get("control_hz", 10.0))

    policy_cls = _load_policy(args.policy)
    out_dir = Path(args.out)

    print(f"[run_eval] task={task}  modes={modes}  seeds={seeds}  "
          f"episodes={episodes}  control_hz={control_hz}")
    print(f"[run_eval] policy={args.policy}  out={out_dir}")

    run_eval(
        task=task,
        modes=modes,
        policy_factory=policy_cls,
        seeds=seeds,
        episodes_per_seed=episodes,
        out_dir=out_dir,
        control_hz=control_hz,
    )

    # Compute the per-condition report from the just-written traces.
    summary = compute_report(out_dir, task=task, modes=modes, seeds=seeds)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
