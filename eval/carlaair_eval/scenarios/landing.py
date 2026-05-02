"""Cooperative Moving-Platform Landing scenario.

Paper §4.1 + Appendix C.1:
  "A UGV truck drives along an urban road while providing a flat rear cargo
   bed as the landing surface. The UAV receives the instruction:
   'Follow the moving truck, align above its rear cargo bed, and land safely.'
   The task consists of tracking, alignment, and landing, with a 60 s episode
   time limit. It is successful only when the UAV lands on the rear cargo bed
   without collision, side impact, or hard landing."

Defaults:
  - Town10HD
  - Truck blueprint: vehicle.carlamotors.european_hgv (matches AGC reference)
  - Truck nominal speed: v0 = 4.0 m/s (paper Eq. 1)
  - UAV-to-truck initial offset: 8 m above + 6 m behind cargo bed
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from ..api.cue import LANDING_BASE_INSTRUCTION
from ..constants import LANDING_EPISODE_SECONDS, C2_V0_MS
from ._carla_helpers import (
    connect, cleanup_world, compute_truck_control, cargo_bed_world,
    cargo_bed_corners_world, attach_rgb_camera, find_drone_actor,
)
from ..utils.coords import calibrate_offset, carla_to_ned


TRUCK_BP = "vehicle.carlamotors.european_hgv"
INIT_ABOVE_BED_M  = 8.0
INIT_BEHIND_BED_M = 6.0


@dataclass
class LandingEpisodeState:
    sim_time: float = 0.0
    truck_speed_ms: float = C2_V0_MS
    truck_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    truck_yaw_rad: float = 0.0
    bed_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    bed_corners: list = field(default_factory=list)
    uav_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    uav_yaw_rad: float = 0.0
    uav_velocity_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rgb_forward: any = None
    rgb_downward: any = None
    in_view: bool = False
    contact: bool = False
    collision: bool = False


@dataclass
class LandingScenario:
    """Builds + ticks the landing scenario inside CarlaAir."""

    spawn_index: int = 1
    truck_target_speed_ms: float = C2_V0_MS
    image_w: int = 640
    image_h: int = 360
    fov_forward: float = 90.0
    fov_downward: float = 90.0
    enable_downward_cam: bool = True
    seed: int = 0

    # internals
    _carla: any = None
    _airsim: any = None
    _client: any = None
    _world: any = None
    _bp_lib: any = None
    _air_client: any = None
    _truck: any = None
    _drone_actor: any = None
    _cam_fwd: any = None
    _cam_dn: any = None
    _collision_sensor: any = None
    _ox: float = 0.0
    _oy: float = 0.0
    _oz: float = 0.0
    _t0: float = 0.0
    _images: Dict[str, any] = field(default_factory=dict)
    _collision: bool = False

    # ── lifecycle ────────────────────────────────────────────────────────
    def setup(self):
        self._client, self._world, self._bp_lib, self._air_client = connect()
        cleanup_world(self._world)

        import carla, airsim
        self._carla, self._airsim = carla, airsim

        # spawn truck
        sp = self._world.get_map().get_spawn_points()[self.spawn_index]
        self._truck = self._world.spawn_actor(self._bp_lib.find(TRUCK_BP), sp)

        # collision sensor on truck so that any contact (UAV, traffic) is
        # captured in the episode trace.
        col_bp = self._bp_lib.find("sensor.other.collision")
        self._collision_sensor = self._world.spawn_actor(col_bp, carla.Transform(),
                                                         attach_to=self._truck)
        self._collision = False
        self._collision_sensor.listen(lambda _e: setattr(self, "_collision", True))

        # let AirSim drone exist; calibrate offset
        self._ox, self._oy, self._oz = calibrate_offset(self._world, self._air_client)
        self._drone_actor = find_drone_actor(self._world)

        # place the UAV: above the cargo bed by INIT_ABOVE_BED_M, behind by
        # INIT_BEHIND_BED_M (in truck body frame)
        bed_local = self._carla.Location(-3.5 - INIT_BEHIND_BED_M, 0.0,
                                         3.75 + INIT_ABOVE_BED_M)
        self._truck.get_transform().transform(bed_local)
        nx, ny, nz = carla_to_ned(bed_local.x, bed_local.y, bed_local.z,
                                  self._ox, self._oy, self._oz)
        yaw = math.radians(self._truck.get_transform().rotation.yaw)
        self._air_client.simSetVehiclePose(
            airsim.Pose(airsim.Vector3r(nx, ny, nz),
                        airsim.to_quaternion(0.0, 0.0, yaw)),
            True,
        )
        time.sleep(0.3)

        # cameras attached to the AirSim-mirrored drone actor
        self._images = {"forward": None, "downward": None}
        if self._drone_actor is not None:
            self._cam_fwd = attach_rgb_camera(
                self._world, self._bp_lib, self._drone_actor,
                x=0.4, y=0.0, z=-0.1, pitch=-30.0,
                width=self.image_w, height=self.image_h, fov=self.fov_forward,
                callback=self._make_listener("forward"),
            )
            if self.enable_downward_cam:
                self._cam_dn = attach_rgb_camera(
                    self._world, self._bp_lib, self._drone_actor,
                    x=0.0, y=0.0, z=-0.1, pitch=-90.0,
                    width=self.image_w, height=self.image_h, fov=self.fov_downward,
                    callback=self._make_listener("downward"),
                )
        self._t0 = time.time()
        return self

    def _make_listener(self, key: str):
        import numpy as np
        imgs = self._images
        def cb(img):
            arr = np.frombuffer(img.raw_data, dtype=np.uint8)
            arr = arr.reshape((img.height, img.width, 4))[:, :, :3][:, :, ::-1].copy()
            imgs[key] = arr
        return cb

    def teardown(self):
        try:
            for sensor in (self._cam_fwd, self._cam_dn, self._collision_sensor):
                if sensor is not None:
                    try: sensor.stop()
                    except Exception: pass
                    try: sensor.destroy()
                    except Exception: pass
            if self._truck is not None:
                self._truck.destroy()
        except Exception:
            pass

    # ── per-tick driving ─────────────────────────────────────────────────
    def step_truck(self, target_speed_ms: float) -> None:
        ctrl = compute_truck_control(self._truck, self._world.get_map(), target_speed_ms)
        self._truck.apply_control(ctrl)

    def get_state(self) -> LandingEpisodeState:
        tf = self._truck.get_transform()
        v = self._truck.get_velocity()
        bed = cargo_bed_world(self._truck)
        bed_corners = cargo_bed_corners_world(self._truck)

        kin = self._air_client.getMultirotorState().kinematics_estimated
        uav_ned = (kin.position.x_val, kin.position.y_val, kin.position.z_val)
        # AirSim returns NED; convert back to CARLA world for unified pose.
        uav_carla = (uav_ned[0] - self._ox, uav_ned[1] - self._oy,
                     -(uav_ned[2] - self._oz))

        # Use AirSim's reported yaw for the UAV.
        q = kin.orientation
        # quaternion → yaw
        siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
        cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
        uav_yaw = math.atan2(siny, cosy)

        return LandingEpisodeState(
            sim_time=time.time() - self._t0,
            truck_speed_ms=math.hypot(v.x, v.y),
            truck_world=(tf.location.x, tf.location.y, tf.location.z),
            truck_yaw_rad=math.radians(tf.rotation.yaw),
            bed_world=bed,
            bed_corners=bed_corners,
            uav_world=uav_carla,
            uav_yaw_rad=uav_yaw,
            uav_velocity_ned=(kin.linear_velocity.x_val,
                              kin.linear_velocity.y_val,
                              kin.linear_velocity.z_val),
            rgb_forward=self._images.get("forward"),
            rgb_downward=self._images.get("downward"),
            in_view=False,    # filled by metric layer
            contact=False,    # filled by metric layer
            collision=self._collision,
        )

    # ── send UAV action ──────────────────────────────────────────────────
    def apply_uav_action(self, action) -> None:
        from ..api.policy import (
            VelocityCommand, WaypointCommand, TrajectoryCommand, DiscreteCommand,
        )
        airsim = self._airsim
        if isinstance(action, VelocityCommand):
            self._air_client.moveByVelocityAsync(
                action.vx, action.vy, action.vz, action.duration_s,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, action.yaw_deg),
            )
        elif isinstance(action, WaypointCommand):
            self._air_client.moveToPositionAsync(
                action.x, action.y, action.z, action.speed_ms,
            )
        elif isinstance(action, TrajectoryCommand):
            # Sample first segment, command at constant speed via velocity.
            if len(action.points) >= 2:
                (x0, y0, z0), (x1, y1, z1) = action.points[0], action.points[1]
                dt = max(action.dt, 1e-3)
                self._air_client.moveByVelocityAsync(
                    (x1 - x0) / dt, (y1 - y0) / dt, (z1 - z0) / dt, dt,
                )
        elif isinstance(action, DiscreteCommand):
            BURST, DUR = 1.5, 0.5
            mp = {
                "forward": (BURST, 0.0, 0.0),
                "left":    (0.0, -BURST, 0.0),
                "right":   (0.0,  BURST, 0.0),
                "up":      (0.0,  0.0, -BURST),
                "down":    (0.0,  0.0,  BURST),
                "hover":   (0.0,  0.0,  0.0),
            }
            vx, vy, vz = mp.get(action.action, (0.0, 0.0, 0.0))
            self._air_client.moveByVelocityAsync(vx, vy, vz, DUR)
