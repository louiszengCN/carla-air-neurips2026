"""Statistical helpers — bootstrap CI + sign test.

Paper App. C.4 ("Statistical analysis"):
  - Single-baseline 95% CI: hierarchical bootstrap clustered by seed,
    1000 resamples (paper-strict default).
  - Across-baseline trend: sign test on per-seed-mean CGs.
"""
from __future__ import annotations

import math
import random
import statistics
from typing import Callable, Dict, List, Sequence, Tuple

from ..constants import BOOTSTRAP_RESAMPLES, BOOTSTRAP_CI_LEVEL


def bootstrap_ci(per_seed_episode_values: Sequence[Sequence[float]],
                 statistic: Callable[[Sequence[float]], float] = lambda xs: sum(xs) / max(len(xs), 1),
                 n_resamples: int = BOOTSTRAP_RESAMPLES,
                 ci_level: float = BOOTSTRAP_CI_LEVEL,
                 rng_seed: int = 0) -> Tuple[float, float, float]:
    """Hierarchical bootstrap clustered by seed.

    `per_seed_episode_values[i]` is the list of per-episode values under
    seed i. Returns (point_estimate, ci_low, ci_high).

    Resampling protocol (clustered):
      1) sample seeds with replacement,
      2) for each sampled seed, sample its episodes with replacement,
      3) flatten and apply `statistic`.
    """
    rng = random.Random(rng_seed)
    seeds = list(per_seed_episode_values)
    if not seeds:
        return 0.0, 0.0, 0.0

    flat = [v for s in seeds for v in s]
    point = statistic(flat) if flat else 0.0

    samples = []
    n_seeds = len(seeds)
    for _ in range(n_resamples):
        bag = []
        for _ in range(n_seeds):
            seed_eps = rng.choice(seeds)
            if not seed_eps:
                continue
            for _ in range(len(seed_eps)):
                bag.append(rng.choice(seed_eps))
        samples.append(statistic(bag) if bag else 0.0)

    samples.sort()
    alpha = (1.0 - ci_level) / 2.0
    lo = samples[int(alpha * len(samples))]
    hi = samples[int((1.0 - alpha) * len(samples)) - 1]
    return point, lo, hi


def sign_test(deltas: Sequence[float]) -> Dict[str, float]:
    """Two-sided sign test: H0 — median(delta) = 0.

    `deltas` are per-baseline (or per-seed) signed differences. Ties are
    excluded (standard sign-test convention).

    Returns p-value plus the count of positive / negative / zero entries.
    """
    pos = sum(1 for d in deltas if d > 0)
    neg = sum(1 for d in deltas if d < 0)
    zero = sum(1 for d in deltas if d == 0)
    n = pos + neg
    if n == 0:
        return {"p_value": 1.0, "n_pos": pos, "n_neg": neg, "n_zero": zero}

    # Two-sided exact binomial w/ p=0.5
    k = min(pos, neg)
    # P(X <= k) under Binomial(n, 0.5)
    cum = 0.0
    for i in range(k + 1):
        cum += math.comb(n, i) * (0.5 ** n)
    p = min(1.0, 2.0 * cum)
    return {"p_value": p, "n_pos": pos, "n_neg": neg, "n_zero": zero}


def per_seed_mean(episode_values: Sequence[Sequence[float]]) -> List[float]:
    return [statistics.mean(s) if s else 0.0 for s in episode_values]
