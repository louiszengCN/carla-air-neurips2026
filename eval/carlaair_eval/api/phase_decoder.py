"""Default phase decoder for C2.

Paper (App. C.2): "The decoded phase signal is a deterministic function of
the VLA action magnitude and vertical velocity component."

We expose a default deterministic decoder; users may pass their own callable
into the C2 coordinator to substitute.
"""
from __future__ import annotations

import math
from typing import Callable

from .policy import UAVAction, VelocityCommand, WaypointCommand, TrajectoryCommand, DiscreteCommand


class LandingPhase:
    APPROACH  = "approach"
    DESCEND   = "descend"
    HOVER     = "hover"
    TOUCHDOWN = "touchdown"


# ── action → (|v|, vz, dz) projection ────────────────────────────────────
def _project_action(action: UAVAction, uav_state: dict) -> tuple[float, float]:
    """Return (action_magnitude, vertical_velocity_ned).

    NED convention: +z is DOWN, so a "descend" action has positive vz.
    """
    if isinstance(action, VelocityCommand):
        mag = math.sqrt(action.vx * action.vx + action.vy * action.vy + action.vz * action.vz)
        return mag, action.vz

    if isinstance(action, WaypointCommand):
        if uav_state is None:
            return action.speed_ms, 0.0
        dx = action.x - uav_state.get("x", 0.0)
        dy = action.y - uav_state.get("y", 0.0)
        dz = action.z - uav_state.get("z", 0.0)
        d = math.sqrt(dx * dx + dy * dy + dz * dz) or 1e-6
        # Project as a unit-speed velocity along the waypoint direction.
        vx = action.speed_ms * dx / d
        vy = action.speed_ms * dy / d
        vz = action.speed_ms * dz / d
        return math.sqrt(vx * vx + vy * vy + vz * vz), vz

    if isinstance(action, TrajectoryCommand) and len(action.points) >= 2:
        (x0, y0, z0), (x1, y1, z1) = action.points[0], action.points[1]
        dt = max(action.dt, 1e-6)
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt
        vz = (z1 - z0) / dt
        return math.sqrt(vx * vx + vy * vy + vz * vz), vz

    if isinstance(action, DiscreteCommand):
        # AerialVLN: 0.5 s burst at 1.5 m/s (App. C.5).
        BURST = 1.5
        mapping = {
            "forward": (BURST, 0.0),
            "left":    (BURST, 0.0),
            "right":   (BURST, 0.0),
            "up":      (BURST, -BURST),  # NED: up = -z
            "down":    (BURST, +BURST),
            "hover":   (0.0,    0.0),
        }
        mag, vz = mapping.get(action.action, (0.0, 0.0))
        return mag, vz

    return 0.0, 0.0


# ── default deterministic decoder ────────────────────────────────────────
def default_phase_decoder(action: UAVAction, uav_state: dict,
                          alt_above_bed_m: float) -> str:
    """Deterministic mapping from (|action|, vz, altitude-above-bed) to phase.

    Thresholds:
      - |v| < 0.3 m/s and |vz| < 0.3 m/s and alt < 0.5 m  -> TOUCHDOWN
      - vz > 0.5 m/s                                       -> DESCEND
      - alt < 1.0 m and |v| < 0.5 m/s                      -> HOVER
      - otherwise                                          -> APPROACH
    """
    mag, vz = _project_action(action, uav_state or {})

    if abs(mag) < 0.3 and abs(vz) < 0.3 and alt_above_bed_m < 0.5:
        return LandingPhase.TOUCHDOWN
    if vz > 0.5:
        return LandingPhase.DESCEND
    if alt_above_bed_m < 1.0 and mag < 0.5:
        return LandingPhase.HOVER
    return LandingPhase.APPROACH


PhaseDecoder = Callable[[UAVAction, dict, float], str]
