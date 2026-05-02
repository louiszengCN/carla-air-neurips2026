"""Edge-case unit tests for landing metrics.

We craft synthetic JSONL traces that exercise each definition boundary
from Appendix C.4.
"""
from pathlib import Path
import json

from carlaair_eval.runtime.trace import TraceWriter, TraceRecord, load_trace
from carlaair_eval.metrics.landing import compute_landing_metrics


def _write_trace(tmp_path: Path, records: list, success_marker=False) -> Path:
    p = tmp_path / "ep.jsonl"
    w = TraceWriter(p, header={"task": "landing", "mode": "C0", "seed": 0,
                                "episode_id": 0, "started_at": 0.0})
    for r in records:
        w.write(r)
    w.close({"finished_at": 0.0, "success": success_marker, "extra": {}})
    return p


def _rec(t, in_view=False, contact=False, collision=False, uav_xy=(0.0, 0.0)):
    return TraceRecord(
        sim_time=t, wall_time=0.0, tick_index=int(t * 10),
        task="landing", mode="C0", seed=0, episode_id=0,
        uav_world=(uav_xy[0], uav_xy[1], 0.0), uav_yaw_rad=0.0,
        ugv_world=(0.0, 0.0, 0.0), ugv_yaw_rad=0.0, ugv_speed_ms=4.0,
        target_in_view=in_view, iou=0.0, occluded=False,
        contact=contact, collision=collision,
    )


def test_tsr_threshold_at_three_seconds(tmp_path):
    # 3.0 s of in-view → tracking_success True (>= K=3).
    recs = [_rec(t / 10.0, in_view=True) for t in range(31)]
    p = _write_trace(tmp_path, recs)
    m = compute_landing_metrics(p)
    assert m.tracking_success is True
    assert m.tracking_seconds >= 3.0


def test_tsr_just_below_threshold(tmp_path):
    # 2.9 s of in-view → tracking_success False.
    recs = [_rec(t / 10.0, in_view=True) for t in range(29)]
    p = _write_trace(tmp_path, recs)
    m = compute_landing_metrics(p)
    assert m.tracking_success is False


def test_lsr_drift_within_threshold(tmp_path):
    # Touchdown at t=10; drift 0.29 m within next 2 s; no collision → success.
    recs = [_rec(0.5, in_view=True)]
    recs.append(_rec(10.0, contact=True, uav_xy=(0.0, 0.0)))
    recs.append(_rec(10.5, contact=True, uav_xy=(0.1, 0.0)))
    recs.append(_rec(11.0, contact=True, uav_xy=(0.2, 0.0)))
    recs.append(_rec(12.0, contact=True, uav_xy=(0.29, 0.0)))
    p = _write_trace(tmp_path, recs)
    m = compute_landing_metrics(p)
    assert m.landing_success is True


def test_lsr_drift_exceeds_threshold(tmp_path):
    # 0.31 m drift within hold window → fail.
    recs = [_rec(10.0, contact=True, uav_xy=(0.0, 0.0))]
    recs.append(_rec(11.0, contact=True, uav_xy=(0.31, 0.0)))
    recs.append(_rec(12.0, contact=True, uav_xy=(0.31, 0.0)))
    p = _write_trace(tmp_path, recs)
    m = compute_landing_metrics(p)
    assert m.landing_success is False


def test_lsr_collision_invalidates_landing(tmp_path):
    recs = [_rec(10.0, contact=True), _rec(12.0, contact=True, collision=True)]
    p = _write_trace(tmp_path, recs)
    m = compute_landing_metrics(p)
    assert m.landing_success is False
