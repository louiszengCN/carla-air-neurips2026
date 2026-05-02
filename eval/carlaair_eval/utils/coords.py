"""Coordinate-frame helpers — mirror Eq. (1) in Appendix B.

CARLA: left-handed Unreal frame, centimetres equivalent (CARLA Python API
already exposes metres; we keep metres). Z-up.
AirSim: NED, metres, Z-down.

Transform applied per tick:
    p_NED = ((p - o)_x, (p - o)_y, -(p - o)_z)
    q_NED = (w, qx, qy, -qz) / ||...||
"""
from __future__ import annotations

import math
from typing import Tuple


def calibrate_offset(world, air_client) -> Tuple[float, float, float]:
    """Return (ox, oy, oz) such that  ned = (cx + ox, cy + oy, -cz + oz).

    Reads the spawned drone's CARLA pose and AirSim NED position in parallel.
    Falls back to (0, 0, 0) if no drone actor is found.
    """
    drone = None
    for a in world.get_actors():
        if "drone" in a.type_id.lower():
            drone = a
            break
    if drone is None:
        return 0.0, 0.0, 0.0
    cl = drone.get_location()
    ap = air_client.getMultirotorState().kinematics_estimated.position
    return ap.x_val - cl.x, ap.y_val - cl.y, ap.z_val - (-cl.z)


def carla_to_ned(x: float, y: float, z: float,
                 ox: float, oy: float, oz: float) -> Tuple[float, float, float]:
    return x + ox, y + oy, -z + oz


def ned_to_carla(nx: float, ny: float, nz: float,
                 ox: float, oy: float, oz: float) -> Tuple[float, float, float]:
    return nx - ox, ny - oy, -(nz - oz)


def yaw_to_quaternion(yaw_rad: float) -> Tuple[float, float, float, float]:
    """Z-axis yaw only — pitch/roll = 0."""
    half = 0.5 * yaw_rad
    return math.cos(half), 0.0, 0.0, math.sin(half)


def relative_in_body_frame(target_world: Tuple[float, float, float],
                           ego_world: Tuple[float, float, float],
                           ego_yaw_rad: float) -> Tuple[float, float, float]:
    """World → ego body frame (forward = +x, right = +y, down = +z (NED-style))."""
    dx = target_world[0] - ego_world[0]
    dy = target_world[1] - ego_world[1]
    dz = target_world[2] - ego_world[2]
    c, s = math.cos(-ego_yaw_rad), math.sin(-ego_yaw_rad)
    bx = dx * c - dy * s
    by = dx * s + dy * c
    return bx, by, dz


def bearing_range_elevation(target_world: Tuple[float, float, float],
                            ego_world: Tuple[float, float, float],
                            ego_yaw_rad: float) -> Tuple[float, float, float]:
    """Return (bearing_deg in [0,360), range_m, elevation_deg)."""
    bx, by, bz = relative_in_body_frame(target_world, ego_world, ego_yaw_rad)
    horiz = math.hypot(bx, by)
    rng = math.sqrt(horiz * horiz + bz * bz)
    bearing = (math.degrees(math.atan2(by, bx)) + 360.0) % 360.0
    elev = math.degrees(math.atan2(-bz, max(horiz, 1e-6)))  # NED z down → flip sign
    return bearing, rng, elev
