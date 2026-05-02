"""Shared CARLA / AirSim helpers used by both scenarios.

These wrap the patterns demonstrated in CarlaAir-v0.1.7/AGC/air_ground_convoy.py
so each scenario file stays focused on its task-specific logic.
"""
from __future__ import annotations

import math
import time
from typing import Optional, Tuple

# Lazy imports — let scenarios import this module without carla/airsim being
# importable at module load (e.g. unit-testing metric code on CI).
def _lazy_imports():
    import carla  # type: ignore
    import airsim  # type: ignore
    return carla, airsim


# ── world / client setup ─────────────────────────────────────────────────
def connect(carla_port: int = 2000, airsim_port: int = 41451, timeout_s: float = 15.0):
    carla, airsim = _lazy_imports()
    client = carla.Client("localhost", carla_port)
    client.set_timeout(timeout_s)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    air_client = airsim.MultirotorClient(port=airsim_port)
    air_client.confirmConnection()
    air_client.enableApiControl(True)
    air_client.armDisarm(True)
    return client, world, bp_lib, air_client


def cleanup_world(world) -> None:
    for s in world.get_actors().filter("sensor.*"):
        try: s.stop(); s.destroy()
        except Exception: pass
    for v in world.get_actors().filter("vehicle.*"):
        try: v.destroy()
        except Exception: pass


# ── truck control (P-controller on speed + heading) ──────────────────────
TRUCK_KP_SPEED  = 0.5
TRUCK_KP_STEER  = 1.0
TRUCK_LOOKAHEAD = 5.0


def compute_truck_control(truck, world_map, target_speed_ms: float):
    """Per-frame VehicleControl. Mirrors AGC/air_ground_convoy.py."""
    carla, _ = _lazy_imports()
    tf  = truck.get_transform()
    loc = tf.location
    yaw = math.radians(tf.rotation.yaw)

    wp = world_map.get_waypoint(loc, project_to_road=True,
                                lane_type=carla.LaneType.Driving)
    steer = 0.0
    if wp is not None:
        nxts = wp.next(TRUCK_LOOKAHEAD)
        if nxts:
            tgt = nxts[0].transform.location
            tgt_yaw = math.atan2(tgt.y - loc.y, tgt.x - loc.x)
            err = ((tgt_yaw - yaw + math.pi) % (2 * math.pi)) - math.pi
            steer = max(-0.5, min(0.5, TRUCK_KP_STEER * err))

    v = truck.get_velocity()
    spd = math.hypot(v.x, v.y)
    err_v = target_speed_ms - spd
    if err_v > 0:
        throttle, brake = min(0.6, TRUCK_KP_SPEED * err_v), 0.0
    else:
        throttle, brake = 0.0, min(0.5, -0.3 * err_v)
    return carla.VehicleControl(throttle=float(throttle),
                                steer=float(steer),
                                brake=float(brake))


# ── cargo-bed pose for the HGV truck ─────────────────────────────────────
# Local-frame offset on `vehicle.carlamotors.european_hgv` corresponding to
# the centre of the rear cargo bed. The HGV has BB.top ≈ 3.46 m; its rear
# cargo bed sits at roof height behind the cabin. The AGC demo locks the
# drone at (0, 0, 3.75) which is roughly cabin-roof centre — for landing on
# the rear bed we shift backward by ~3.5 m.
CARGO_BED_LOCAL = (-3.5, 0.0, 3.75)
CARGO_BED_HALF_EXTENTS = (1.6, 1.2, 0.05)  # x, y, z half-extents in metres


def cargo_bed_world(truck) -> Tuple[float, float, float]:
    carla, _ = _lazy_imports()
    local = carla.Location(*CARGO_BED_LOCAL)
    truck.get_transform().transform(local)
    return local.x, local.y, local.z


def cargo_bed_corners_world(truck):
    """8 world-space corners of the cargo-bed AABB (for IoU / collision tests)."""
    carla, _ = _lazy_imports()
    bx, by, bz = CARGO_BED_HALF_EXTENTS
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                local = carla.Location(
                    CARGO_BED_LOCAL[0] + sx * bx,
                    CARGO_BED_LOCAL[1] + sy * by,
                    CARGO_BED_LOCAL[2] + sz * bz,
                )
                truck.get_transform().transform(local)
                out.append((local.x, local.y, local.z))
    return out


# ── camera helpers ───────────────────────────────────────────────────────
def attach_rgb_camera(world, bp_lib, parent, x: float, y: float, z: float,
                      pitch: float = 0.0, yaw: float = 0.0,
                      width: int = 640, height: int = 360, fov: float = 90.0,
                      callback=None):
    carla, _ = _lazy_imports()
    bp = bp_lib.find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", str(fov))
    cam = world.spawn_actor(
        bp,
        carla.Transform(carla.Location(x=x, y=y, z=z),
                        carla.Rotation(pitch=pitch, yaw=yaw)),
        attach_to=parent,
    )
    if callback is not None:
        cam.listen(callback)
    return cam


def find_drone_actor(world):
    for a in world.get_actors():
        if "drone" in a.type_id.lower():
            return a
    return None
