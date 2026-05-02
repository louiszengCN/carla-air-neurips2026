"""Cooperative Occlusion-Recovery Escort scenario.

Paper §4.1 + Appendix C.1:
  "A UGV drives along an urban route and becomes temporarily occluded by
   bridges, buildings, or large artifacts. The UAV must escort the UGV and
   recover visual contact after the target becomes invisible. Each escort
   episode has a 90 s time limit."

Occlusion event distribution (App. C.4):
   1–3 events / episode; per-event duration uniform 4–12 s; geometry mix
   {bridge 40%, building 35%, artifact 25%}. Minimal release implements
   `bridge` only (constants.OCCLUSION_GEOMETRY_ENABLED).

The UGV path is driven along Town10HD using waypoint P-control, seeded from
`examples/trajectories/vehicle_20260501_191526.json` (a recorded bridge
passage that establishes a known occlusion location). We respawn at the
recorded start pose and let the P-controller carry the UGV through the
bridge for the rest of the episode.
"""
from __future__ import annotations

import json
import math
import os
import random
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from ..api.cue import ESCORT_BASE_INSTRUCTION
from ..constants import (
    ESCORT_EPISODE_SECONDS, OCCLUSION_DURATION_RANGE,
    OCCLUSIONS_PER_EPISODE, OCCLUSION_GEOMETRY_ENABLED, C2_V0_MS,
)
from ._carla_helpers import (
    connect, cleanup_world, compute_truck_control,
    attach_rgb_camera, find_drone_actor,
)
from ..utils.coords import calibrate_offset, carla_to_ned


# The trajectory file ships inside the eval package so the suite is fully
# self-contained. Override with env var `CARLAAIR_TRAJECTORY_DIR` if you
# keep recorded UGV trajectories elsewhere.
_TRAJ_NAME = "vehicle_20260501_191526.json"
_PKG_DEFAULT = Path(__file__).resolve().parent / "data" / _TRAJ_NAME
_ENV_DIR = os.environ.get("CARLAAIR_TRAJECTORY_DIR")
if _ENV_DIR and (Path(_ENV_DIR) / _TRAJ_NAME).exists():
    DEFAULT_TRAJECTORY_JSON = Path(_ENV_DIR) / _TRAJ_NAME
else:
    DEFAULT_TRAJECTORY_JSON = _PKG_DEFAULT
ESCORT_VEHICLE_BP = "vehicle.tesla.model3"   # matches recorded trajectory file
ESCORT_UAV_ALT_M  = 8.0                      # initial AGL above UGV roof


@dataclass
class OcclusionEvent:
    onset_s: float
    duration_s: float
    kind: str = "bridge"
    motion_intent: str = "continues forward"
    reappear_direction: str = "forward-right side"


@dataclass
class EscortEpisodeState:
    sim_time: float = 0.0
    ugv_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ugv_yaw_rad: float = 0.0
    ugv_speed_ms: float = 0.0
    ugv_corners: list = field(default_factory=list)
    uav_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    uav_yaw_rad: float = 0.0
    rgb_forward: any = None
    cam_pose_world: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    cam_yaw_pitch_roll_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    occluded: bool = False
    active_occlusion: Optional[OcclusionEvent] = None


def _sample_occlusions(rng: random.Random, episode_len_s: float) -> List[OcclusionEvent]:
    n = rng.randint(*OCCLUSIONS_PER_EPISODE)
    events = []
    margin = 5.0
    for i in range(n):
        # Spread events through (margin, episode_len - margin) without overlap.
        slot = (episode_len_s - 2 * margin) / max(n, 1)
        center = margin + (i + 0.5) * slot + rng.uniform(-slot * 0.2, slot * 0.2)
        duration = rng.uniform(*OCCLUSION_DURATION_RANGE)
        onset = max(margin, center - duration / 2.0)
        kind = rng.choice(OCCLUSION_GEOMETRY_ENABLED)
        events.append(OcclusionEvent(onset_s=onset, duration_s=duration, kind=kind))
    return events


@dataclass
class EscortScenario:
    trajectory_json: Path = DEFAULT_TRAJECTORY_JSON
    truck_target_speed_ms: float = C2_V0_MS
    image_w: int = 640
    image_h: int = 360
    fov_forward: float = 90.0
    seed: int = 0

    _carla: any = None
    _airsim: any = None
    _client: any = None
    _world: any = None
    _bp_lib: any = None
    _air_client: any = None
    _ugv: any = None
    _drone_actor: any = None
    _cam_fwd: any = None
    _ox: float = 0.0
    _oy: float = 0.0
    _oz: float = 0.0
    _t0: float = 0.0
    _images: Dict[str, any] = field(default_factory=dict)
    occlusions: List[OcclusionEvent] = field(default_factory=list)

    def setup(self):
        self._client, self._world, self._bp_lib, self._air_client = connect()
        cleanup_world(self._world)
        import carla, airsim
        self._carla, self._airsim = carla, airsim

        # Read recorded start pose from the supplied trajectory JSON.
        start_x, start_y, start_yaw_deg = 61.2646, 131.5464, 178.4411
        if self.trajectory_json and Path(self.trajectory_json).exists():
            try:
                data = json.loads(Path(self.trajectory_json).read_text())
                f0 = data["frames"][0]
                start_x = float(f0["transform"]["x"])
                start_y = float(f0["transform"]["y"])
                start_yaw_deg = float(f0["transform"]["yaw"])
            except Exception:
                pass

        spawn_tf = carla.Transform(
            carla.Location(x=start_x, y=start_y, z=0.3),
            carla.Rotation(pitch=0.0, yaw=start_yaw_deg, roll=0.0),
        )
        self._ugv = self._world.spawn_actor(
            self._bp_lib.find(ESCORT_VEHICLE_BP), spawn_tf
        )

        # Calibrate + place UAV directly above UGV.
        self._ox, self._oy, self._oz = calibrate_offset(self._world, self._air_client)
        self._drone_actor = find_drone_actor(self._world)

        nx, ny, nz = carla_to_ned(start_x, start_y, ESCORT_UAV_ALT_M,
                                  self._ox, self._oy, self._oz)
        self._air_client.simSetVehiclePose(
            airsim.Pose(airsim.Vector3r(nx, ny, nz),
                        airsim.to_quaternion(0.0, 0.0, math.radians(start_yaw_deg))),
            True,
        )
        time.sleep(0.3)

        # forward camera
        self._images = {"forward": None}
        if self._drone_actor is not None:
            self._cam_fwd = attach_rgb_camera(
                self._world, self._bp_lib, self._drone_actor,
                x=0.4, y=0.0, z=-0.1, pitch=-30.0,
                width=self.image_w, height=self.image_h, fov=self.fov_forward,
                callback=self._make_listener("forward"),
            )

        rng = random.Random(self.seed)
        self.occlusions = _sample_occlusions(rng, ESCORT_EPISODE_SECONDS)
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
            if self._cam_fwd is not None:
                try: self._cam_fwd.stop()
                except Exception: pass
                try: self._cam_fwd.destroy()
                except Exception: pass
            if self._ugv is not None:
                self._ugv.destroy()
        except Exception:
            pass

    # ── per-tick driving ────────────────────────────────────────────────
    def step_ugv(self, target_speed_ms: float) -> None:
        ctrl = compute_truck_control(self._ugv, self._world.get_map(), target_speed_ms)
        self._ugv.apply_control(ctrl)

    def active_occlusion(self, sim_time: float) -> Optional[OcclusionEvent]:
        for ev in self.occlusions:
            if ev.onset_s <= sim_time < ev.onset_s + ev.duration_s:
                return ev
        return None

    def get_state(self) -> EscortEpisodeState:
        tf = self._ugv.get_transform()
        v = self._ugv.get_velocity()
        # 8 corners of UGV bounding box, world space.
        bb = self._ugv.bounding_box
        ext = bb.extent
        corners = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    local = self._carla.Location(bb.location.x + sx * ext.x,
                                                 bb.location.y + sy * ext.y,
                                                 bb.location.z + sz * ext.z)
                    self._ugv.get_transform().transform(local)
                    corners.append((local.x, local.y, local.z))

        kin = self._air_client.getMultirotorState().kinematics_estimated
        uav_carla = (kin.position.x_val - self._ox,
                     kin.position.y_val - self._oy,
                     -(kin.position.z_val - self._oz))
        q = kin.orientation
        siny = 2.0 * (q.w_val * q.z_val + q.x_val * q.y_val)
        cosy = 1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
        uav_yaw = math.atan2(siny, cosy)

        # Camera world pose (offset (0.4, 0, -0.1) on the drone, pitch -30).
        # We approximate camera pose by the drone pose for the projection
        # geometry — sufficient for IoU since the camera offset is < 1 m.
        cam_pose_world = uav_carla
        cam_yaw_deg = math.degrees(uav_yaw)
        cam_pitch_deg = -30.0

        sim_time = time.time() - self._t0
        ev = self.active_occlusion(sim_time)
        return EscortEpisodeState(
            sim_time=sim_time,
            ugv_world=(tf.location.x, tf.location.y, tf.location.z),
            ugv_yaw_rad=math.radians(tf.rotation.yaw),
            ugv_speed_ms=math.hypot(v.x, v.y),
            ugv_corners=corners,
            uav_world=uav_carla,
            uav_yaw_rad=uav_yaw,
            rgb_forward=self._images.get("forward"),
            cam_pose_world=cam_pose_world,
            cam_yaw_pitch_roll_deg=(cam_yaw_deg, cam_pitch_deg, 0.0),
            occluded=ev is not None,
            active_occlusion=ev,
        )

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
            self._air_client.moveToPositionAsync(action.x, action.y, action.z,
                                                 action.speed_ms)
        elif isinstance(action, TrajectoryCommand):
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
