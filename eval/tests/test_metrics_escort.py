"""Edge cases for RSR / RAT (Appendix C.4)."""
from pathlib import Path

from carlaair_eval.runtime.trace import TraceWriter, TraceRecord, load_trace
from carlaair_eval.metrics.escort import compute_escort_metrics, summarise_escort


def _write_trace_with_log(tmp_path: Path, occlusion_log) -> Path:
    p = tmp_path / "ep.jsonl"
    w = TraceWriter(p, header={"task": "escort", "mode": "C0", "seed": 0,
                                "episode_id": 0, "started_at": 0.0})
    # one no-op record so loader is happy
    w.write(TraceRecord(
        sim_time=0.0, wall_time=0.0, tick_index=0, task="escort",
        mode="C0", seed=0, episode_id=0,
        uav_world=(0.0, 0.0, 0.0), uav_yaw_rad=0.0,
        ugv_world=(0.0, 0.0, 0.0), ugv_yaw_rad=0.0, ugv_speed_ms=4.0,
    ))
    w.close({"finished_at": 0.0, "success": False, "extra": {"occlusion_log": occlusion_log}})
    return p


def test_rsr_recovered_within_15s(tmp_path):
    log = [{"event_id": 0, "onset_s": 5.0, "recovered": True, "rat_s": 6.5}]
    p = _write_trace_with_log(tmp_path, log)
    m = compute_escort_metrics(p)
    assert m.n_recovered == 1 and m.n_events == 1
    assert m.rats == [6.5]


def test_rat_capped_for_unrecovered(tmp_path):
    log = [{"event_id": 0, "onset_s": 5.0, "recovered": False, "rat_s": 15.0}]
    p = _write_trace_with_log(tmp_path, log)
    m = compute_escort_metrics(p)
    assert m.rats == [15.0]
    summary = summarise_escort([m])
    assert summary["RSR"] == 0.0
    assert summary["RAT"] == 15.0


def test_rsr_aggregate_across_events(tmp_path):
    log = [
        {"event_id": 0, "onset_s": 5.0,  "recovered": True,  "rat_s": 4.0},
        {"event_id": 1, "onset_s": 30.0, "recovered": False, "rat_s": 15.0},
        {"event_id": 2, "onset_s": 60.0, "recovered": True,  "rat_s": 7.5},
    ]
    p = _write_trace_with_log(tmp_path, log)
    m = compute_escort_metrics(p)
    summary = summarise_escort([m])
    assert summary["RSR"] == 2 / 3
