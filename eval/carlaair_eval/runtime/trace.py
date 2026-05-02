"""Per-episode trace logging — JSONL, one record per control tick.

Each record captures the minimal information needed for every paper metric
(TSR, LSR, CCR, CG, RSR, RAT, DF, ECL) plus enough debug fields for visual
inspection.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Dict, List, Optional


@dataclass
class TraceRecord:
    sim_time: float
    wall_time: float
    tick_index: int
    task: str
    mode: str
    seed: int
    episode_id: int

    # actor poses
    uav_world: tuple
    uav_yaw_rad: float
    ugv_world: tuple
    ugv_yaw_rad: float
    ugv_speed_ms: float

    # task-specific signals
    target_in_view: bool = False
    iou: float = 0.0
    occluded: bool = False
    contact: bool = False
    collision: bool = False
    bed_world: Optional[tuple] = None
    bed_offset_body: Optional[tuple] = None

    # cooperation
    cue_text: Optional[str] = None
    cue_format: Optional[str] = None
    decoded_phase: Optional[str] = None
    ugv_target_speed_ms: float = 0.0

    # action
    action_kind: str = ""
    action_payload: Dict[str, Any] = field(default_factory=dict)

    # latency bookkeeping (filled by runner)
    action_latency_ms: float = 0.0
    cue_latency_ms: float = 0.0


@dataclass
class EpisodeTrace:
    task: str
    mode: str
    seed: int
    episode_id: int
    started_at: float
    finished_at: float = 0.0
    success: bool = False
    extra: Dict[str, Any] = field(default_factory=dict)
    records: List[TraceRecord] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["records"] = [asdict(r) for r in self.records]
        return d


class TraceWriter:
    """Streams TraceRecord JSON lines as the episode progresses."""

    def __init__(self, path: Path, header: Dict[str, Any]):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", buffering=1)
        self._fh.write(json.dumps({"__header__": header}) + "\n")

    def write(self, record: TraceRecord) -> None:
        self._fh.write(json.dumps(asdict(record), default=_json_default) + "\n")

    def close(self, footer: Dict[str, Any]) -> None:
        self._fh.write(json.dumps({"__footer__": footer}) + "\n")
        self._fh.close()


def _json_default(o):
    try:
        import numpy as np
        if isinstance(o, np.ndarray):
            return o.tolist()
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
    except ImportError:
        pass
    return str(o)


def load_trace(path: Path) -> EpisodeTrace:
    """Reverse of TraceWriter — read a JSONL trace into an EpisodeTrace."""
    path = Path(path)
    header: Dict[str, Any] = {}
    footer: Dict[str, Any] = {}
    records: List[TraceRecord] = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        obj = json.loads(line)
        if "__header__" in obj:
            header = obj["__header__"]
            continue
        if "__footer__" in obj:
            footer = obj["__footer__"]
            continue
        records.append(TraceRecord(**obj))
    et = EpisodeTrace(
        task=header.get("task", ""),
        mode=header.get("mode", ""),
        seed=header.get("seed", 0),
        episode_id=header.get("episode_id", 0),
        started_at=header.get("started_at", 0.0),
        finished_at=footer.get("finished_at", 0.0),
        success=footer.get("success", False),
        extra=footer.get("extra", {}),
        records=records,
    )
    return et
