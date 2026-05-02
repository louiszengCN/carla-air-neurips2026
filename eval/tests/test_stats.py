"""Sanity checks on bootstrap CI + sign test."""
from carlaair_eval.metrics.stats import bootstrap_ci, sign_test


def test_bootstrap_ci_recovers_mean():
    # Constant data → CI collapses to the mean.
    pt, lo, hi = bootstrap_ci([[0.5, 0.5, 0.5], [0.5], [0.5, 0.5]],
                               n_resamples=200)
    assert abs(pt - 0.5) < 1e-9
    assert abs(lo - 0.5) < 1e-9
    assert abs(hi - 0.5) < 1e-9


def test_sign_test_two_sided_zero():
    # All positive → p < 0.1 (n=5 all on same side).
    r = sign_test([0.1, 0.2, 0.05, 0.3, 0.4])
    assert r["n_pos"] == 5 and r["n_neg"] == 0
    assert r["p_value"] < 0.1


def test_sign_test_balanced():
    r = sign_test([+1.0, +1.0, -1.0, -1.0])
    assert abs(r["p_value"] - 1.0) < 1e-9
