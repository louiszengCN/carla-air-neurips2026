"""UAVPolicy interface — the single integration point for user policies.

The eval suite calls `policy.reset(...)` once per episode and `policy.act(...)`
once per control tick. The policy returns a `UAVAction` whose concrete
subclass selects the low-level wrapper (velocity / waypoint / trajectory /
discrete) — matching the heterogeneous baseline output regimes in the paper
(Appendix C.5, Table 7).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple, Dict, Any


# ── action types ─────────────────────────────────────────────────────────
@dataclass
class UAVAction:
    """Base class. Subclass instances are dispatched by the runtime."""
    pass


@dataclass
class VelocityCommand(UAVAction):
    """Continuous velocity command (NED, m/s) — AerialVLA regime.

    yaw_deg is absolute world yaw. Duration in seconds bounds the command.
    """
    vx: float
    vy: float
    vz: float
    yaw_deg: float = 0.0
    duration_s: float = 0.25


@dataclass
class WaypointCommand(UAVAction):
    """Single waypoint (NED, m) — SPF regime."""
    x: float
    y: float
    z: float
    speed_ms: float = 4.0


@dataclass
class TrajectoryCommand(UAVAction):
    """Dense trajectory (list of NED points sampled at `dt`) — OpenUAV regime."""
    points: Sequence[Tuple[float, float, float]]
    dt: float = 0.1


@dataclass
class DiscreteCommand(UAVAction):
    """{forward, left, right, up, down, hover} mapped to a 0.5 s velocity burst
    at 1.5 m/s — AerialVLN regime (Appendix C.5)."""
    action: str  # one of: forward, left, right, up, down, hover


# ── observation / cue ────────────────────────────────────────────────────
@dataclass
class Observation:
    """Per-tick UAV observation. Image fields may be None if disabled."""
    sim_time: float
    rgb_forward: Optional[Any] = None       # HxWx3 uint8 (np.ndarray)
    rgb_downward: Optional[Any] = None
    depth: Optional[Any] = None
    imu: Optional[Dict[str, float]] = None  # ax, ay, az, gx, gy, gz
    gnss: Optional[Dict[str, float]] = None # latitude, longitude, altitude
    uav_state: Optional[Dict[str, float]] = None
    extras: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PartnerCue:
    """C1 / C2 cue passed to the UAV. None under C0."""
    text: str                 # the assistant-hint prompt fragment (paper Tbl. 6)
    structured: Dict[str, Any] = field(default_factory=dict)
    format: str = "semantic"  # semantic | numeric | noisy | oracle_bearing


# ── policy ABC ───────────────────────────────────────────────────────────
class UAVPolicy(ABC):
    """User-implemented policy. Native action regime selected via return type."""

    @abstractmethod
    def reset(self, task_instruction: str, task_name: str, episode_id: int) -> None:
        """Called once per episode before the first `act`.

        `task_instruction` is the verbatim prompt from paper Tbl. 6
        (e.g. "Follow the moving truck, align above its rear cargo bed, ...").
        """

    @abstractmethod
    def act(self, observation: Observation, partner_cue: Optional[PartnerCue]) -> UAVAction:
        """Return the next UAV action. `partner_cue` is None under C0."""

    def close(self) -> None:
        """Optional cleanup hook called at end of episode."""
        pass
