#!/usr/bin/env python3
"""
air_ground_convoy.py — Air-ground convoy: drone rides a truck, takes off, escorts, lands back
==============================================================================================

Full scenario (all while the truck is moving, no stopping):

    Phase A  MOUNTED    (t <  3s )  drone sits locked on the truck roof
    Phase B  FOLLOWING  (t <  8s )  truck starts driving, drone still locked to roof
    Phase C  TAKING_OFF (t < 13s )  drone arms, climbs vertically to 8 m above the roof
    Phase D  ESCORTING  (t < 25s )  drone flies in formation above the truck
    Phase E  LANDING    (t < 35s )  drone matches truck velocity, descends, locks on roof
    Phase F  LANDED                 drone pinned to roof again, scenario ends

Technique:
- During MOUNTED / FOLLOWING / LANDED, drone physics is overridden each frame with
  `simSetVehiclePose()` — it is rigidly pinned to the truck roof (zero drift).
- During TAKING_OFF / ESCORTING / LANDING, AirSim API control is active
  (`moveToPositionAsync` / `moveToZAsync`) so the drone flies using AirSim physics.
- Handoff is clean because simSetVehiclePose with `ignore_collision=True` resets
  kinematics before API control takes over, and before API release re-locks pose.

Usage:
    conda activate carlaAir
    python3 examples/air_ground_convoy.py            # Town10HD, 35 s
    python3 examples/air_ground_convoy.py --duration 50
    python3 examples/air_ground_convoy.py --spawn 42

Controls:
    ESC  quit early
"""
import argparse
import math
import sys
import time

import carla
import airsim
import numpy as np
import pygame


# ─────────────────────────── tunables ────────────────────────────
# 4-panel 2x2 layout: [truck chase | orbit] on top, [drone FPV | drone chase] below
PW, PH = 640, 360
DISPLAY_W, DISPLAY_H = PW * 2, PH * 2

TRUCK_BP    = "vehicle.carlamotors.european_hgv"
# Mount point in truck local frame. z=3.75 puts drone sitting just above the
# HGV roof top (BB.top = 3.46 m, +0.3 m clearance). Drone's CARLA physics is
# disabled so it cannot disturb the truck, but this height keeps the drone
# visually above the cabin rather than clipping into it.
# Stored as tuple — carla.Transform.transform() mutates Location in place.
ROOF_LOCAL_XYZ = (0.0, 0.0, 3.75)

TRUCK_TARGET_SPEED = 8.0   # m/s ≈ 28.8 km/h — gentle truck cruise speed
TRUCK_KP_SPEED     = 0.5   # throttle P-gain on speed error
TRUCK_KP_STEER     = 1.0   # steer P-gain on heading error (radians → steer)
TRUCK_WP_LOOKAHEAD = 5.0   # metres to look ahead for next waypoint

ESCORT_ALT  = 8.0    # metres above truck roof

# Phase timeline (seconds from start)
T_MOUNT_END     = 3.0
T_FOLLOW_END    = 8.0
T_TAKEOFF_END   = 13.0
T_ESCORT_END    = 25.0
T_LANDING_END   = 35.0
T_POST_LANDING  = 8.0    # extra seconds of LANDED cruising before the scenario ends

# Velocity controller gains (per-frame moveByVelocityAsync)
KP_HORIZ        = 1.5   # horizontal position error → velocity correction
KP_VERT         = 1.0   # vertical position error → velocity correction
V_MAX_HORIZ     = 20.0  # clamp horizontal command
V_MAX_VERT      = 3.0   # clamp vertical command

LANDING_LOCK_DIST = 0.5   # horizontal metres — snap zone around mount point
LANDING_LOCK_VERT = 0.35  # vertical metres   — snap zone around mount point
LANDING_FINAL_ALT = 0.15  # metres above roof at end of LANDING phase (tight touch-down)
TAKEOFF_CLIMB_RATE = 2.5  # m/s — vertical speed during takeoff phase

WEATHERS = [carla.WeatherParameters.ClearNoon]


# ─────────────────────────── helpers ─────────────────────────────
def cleanup_world(world):
    for s in world.get_actors().filter("sensor.*"):
        try: s.stop(); s.destroy()
        except Exception: pass
    for v in world.get_actors().filter("vehicle.*"):
        try: v.destroy()
        except Exception: pass


def carla_drone_actor(world):
    for a in world.get_actors():
        if "drone" in a.type_id.lower():
            return a
    return None


def calibrate_offset(world, air_client):
    """Return (ox, oy, oz) with: airsim_ned = (cx+ox, cy+oy, -cz+oz)."""
    drone = carla_drone_actor(world)
    if not drone:
        return 0.0, 0.0, 0.0
    cl = drone.get_location()
    ap = air_client.getMultirotorState().kinematics_estimated.position
    return ap.x_val - cl.x, ap.y_val - cl.y, ap.z_val - (-cl.z)


def carla_to_ned(x, y, z, ox, oy, oz):
    return x + ox, y + oy, -z + oz


def truck_roof_world(truck):
    """World-space Location of the roof mount point, respecting truck yaw/pitch/roll.

    NOTE: carla.Transform.transform() mutates its argument in place, so we must
    construct a fresh Location every call — never pass in a shared constant.
    """
    local = carla.Location(*ROOF_LOCAL_XYZ)
    truck.get_transform().transform(local)
    return local


def pose_lock_drone_to_truck(air_client, truck, ox, oy, oz,
                              offset_xyz=None, vehicle_name=""):
    """Pin drone pose to truck_pose * offset_xyz (yaw matches truck yaw).

    offset_xyz is the drone's position in the truck's local frame.
    - Default None → use ROOF_LOCAL_XYZ (nominal mount point).
    - At touch-down we freeze the drone's ACTUAL relative offset so it snaps
      in place (no visible teleport jump).
    """
    if offset_xyz is None:
        offset_xyz = ROOF_LOCAL_XYZ
    local = carla.Location(*offset_xyz)
    truck.get_transform().transform(local)   # in-place → world coords
    yaw_rad = math.radians(truck.get_transform().rotation.yaw)
    nx, ny, nz = carla_to_ned(local.x, local.y, local.z, ox, oy, oz)
    air_client.simSetVehiclePose(
        airsim.Pose(airsim.Vector3r(nx, ny, nz),
                    airsim.to_quaternion(0.0, 0.0, yaw_rad)),
        True, vehicle_name=vehicle_name)


def drone_offset_in_truck_frame(truck, drone_actor):
    """Return (lx, ly, lz): drone's current CARLA world pose expressed in the
    truck's local frame, so we can freeze the drone *where it is right now*
    and have it track the truck's motion with zero visual jump."""
    dl = drone_actor.get_location()
    tf = truck.get_transform()
    dx = dl.x - tf.location.x
    dy = dl.y - tf.location.y
    dz = dl.z - tf.location.z
    yaw = math.radians(tf.rotation.yaw)
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = dx * c - dy * s
    ly = dx * s + dy * c
    return (lx, ly, dz)


def horizontal_distance(a, b):
    return math.sqrt((a.x_val - b[0]) ** 2 + (a.y_val - b[1]) ** 2)


def compute_truck_control(truck, world_map, target_speed):
    """Direct per-frame VehicleControl — smooth P-controllers on speed + heading.

    Replaces Traffic Manager autopilot. TM's 20 Hz discrete waypoint updates
    and bang-bang throttle are what caused the visible cruise-mode jitter;
    continuous 30 Hz P-control produces smooth motion.

    Returns: carla.VehicleControl(throttle, steer, brake)
    """
    tf = truck.get_transform()
    loc = tf.location
    yaw_rad = math.radians(tf.rotation.yaw)

    # ─── steer: aim at waypoint LOOKAHEAD metres ahead in the current lane ──
    wp = world_map.get_waypoint(loc, project_to_road=True,
                                lane_type=carla.LaneType.Driving)
    steer = 0.0
    if wp is not None:
        next_wps = wp.next(TRUCK_WP_LOOKAHEAD)
        if next_wps:
            tgt = next_wps[0].transform.location
            dx = tgt.x - loc.x
            dy = tgt.y - loc.y
            tgt_yaw = math.atan2(dy, dx)
            err = ((tgt_yaw - yaw_rad + math.pi) % (2 * math.pi)) - math.pi
            steer = max(-0.5, min(0.5, TRUCK_KP_STEER * err))

    # ─── throttle / brake: P-controller on speed ──
    v = truck.get_velocity()
    spd = math.sqrt(v.x ** 2 + v.y ** 2)
    err_v = target_speed - spd
    if err_v > 0:
        throttle = min(0.6, TRUCK_KP_SPEED * err_v)
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(0.5, -0.3 * err_v)

    return carla.VehicleControl(throttle=float(throttle),
                                steer=float(steer),
                                brake=float(brake))


# ─────────────────────────── state machine ───────────────────────
class Phase:
    MOUNTED = "MOUNTED"
    FOLLOWING = "FOLLOWING"
    TAKING_OFF = "TAKING_OFF"
    ESCORTING = "ESCORTING"
    LANDING = "LANDING"
    LANDED = "LANDED"


def phase_from_time(t):
    if t < T_MOUNT_END:    return Phase.MOUNTED
    if t < T_FOLLOW_END:   return Phase.FOLLOWING
    if t < T_TAKEOFF_END:  return Phase.TAKING_OFF
    if t < T_ESCORT_END:   return Phase.ESCORTING
    if t < T_LANDING_END:  return Phase.LANDING
    return Phase.LANDED


# ─────────────────────────── main ────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float,
                    default=T_LANDING_END + T_POST_LANDING)
    ap.add_argument("--spawn", type=int, default=1,
                    help="Spawn point index (Town10HD sp[1] is a 150m straight)")
    ap.add_argument("--truck-speed", type=float, default=TRUCK_TARGET_SPEED,
                    help="Truck cruise speed in m/s (default: 8.0 m/s ≈ 28.8 km/h)")
    args = ap.parse_args()
    truck_target_speed = float(args.truck_speed)

    actors = []
    air_client = None
    original_settings = None

    try:
        print("\n  [init] Connecting to CarlaAir ...")
        client = carla.Client("localhost", 2000); client.set_timeout(15.0)
        world = client.get_world(); bp_lib = world.get_blueprint_library()
        cleanup_world(world)

        air_client = airsim.MultirotorClient(port=41451)
        air_client.confirmConnection()
        air_client.enableApiControl(True); air_client.armDisarm(True)

        # Async mode — TM autopilot and AirSim physics run freely at their own
        # native rates. Epic quality + 4 cameras would stall sync ticks and freeze
        # the truck; async keeps everything moving smoothly.
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        world.set_weather(WEATHERS[0])

        # Spawn truck
        truck_bp = bp_lib.find(TRUCK_BP)
        sp = world.get_map().get_spawn_points()[args.spawn]
        truck = world.spawn_actor(truck_bp, sp); actors.append(truck)
        print(f"  [init] Spawned {TRUCK_BP} at sp[{args.spawn}] "
              f"({sp.location.x:.1f}, {sp.location.y:.1f})")

        # Calibrate CARLA↔AirSim offset against the current drone
        ox, oy, oz = calibrate_offset(world, air_client)
        print(f"  [init] CARLA→NED offset = ({ox:.2f}, {oy:.2f}, {oz:.2f})")

        # Lock drone onto truck roof BEFORE truck starts moving
        pose_lock_drone_to_truck(air_client, truck, ox, oy, oz)
        time.sleep(0.3)

        # ─── 4 cameras, one per panel (FOV tuned per view) ───
        images = {"chase": None, "spec": None, "drone_fpv": None, "drone_chase": None}
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(PW))
        cam_bp.set_attribute("image_size_y", str(PH))

        def _listen(key):
            def cb(img):
                arr = np.frombuffer(img.raw_data, np.uint8).reshape((img.height, img.width, 4))
                images[key] = arr[:, :, :3][:, :, ::-1]
            return cb

        # Panel 1: truck chase — behind + slightly above truck, looking forward
        cam_bp.set_attribute("fov", "100")
        chase_cam = world.spawn_actor(cam_bp,
            carla.Transform(carla.Location(x=-12.0, z=5.5), carla.Rotation(pitch=-15)),
            attach_to=truck)
        chase_cam.listen(_listen("chase")); actors.append(chase_cam)

        # Panel 2: truck rear-view — cabin roof, yaw 180°, tilted more skyward
        # with wider FOV so the drone stays framed across its full flight path.
        cam_bp.set_attribute("fov", "130")
        rearview_cam = world.spawn_actor(cam_bp,
            carla.Transform(carla.Location(x=1.0, z=3.8),
                            carla.Rotation(pitch=22.0, yaw=180.0)),
            attach_to=truck)
        rearview_cam.listen(_listen("spec")); actors.append(rearview_cam)

        # Find the CARLA-side drone actor (mirrored AirSim pose) to attach drone cams
        drone_actor = carla_drone_actor(world)
        if drone_actor is None:
            raise RuntimeError("No CARLA drone actor found — AirSim drone not visible in CARLA")
        print(f"  [init] CARLA drone actor id={drone_actor.id} type={drone_actor.type_id}")

        # Disable the CARLA-side drone physics — AirSim still flies the drone on
        # its own internal physics thread; the CARLA mirror is now a kinematic
        # actor that only moves when AirSim syncs its pose. This guarantees the
        # drone cannot collide with or disturb the truck's physics for any reason.
        try:
            drone_actor.set_simulate_physics(False)
            print("  [init] CARLA drone physics disabled — pure kinematic mirror")
        except Exception as e:
            print(f"  [warn] could not disable drone physics: {e}")

        # Panel 3: drone FPV — nose-mounted, steeper downward tilt + wider FOV so
        # the truck is always visible below during escort / landing.
        cam_bp.set_attribute("fov", "120")
        drone_fpv_cam = world.spawn_actor(cam_bp,
            carla.Transform(carla.Location(x=0.4, z=-0.1), carla.Rotation(pitch=-30)),
            attach_to=drone_actor)
        drone_fpv_cam.listen(_listen("drone_fpv")); actors.append(drone_fpv_cam)

        # Panel 4: drone 3rd-person — behind + above drone
        cam_bp.set_attribute("fov", "100")
        drone_chase_cam = world.spawn_actor(cam_bp,
            carla.Transform(carla.Location(x=-4.0, z=1.5), carla.Rotation(pitch=-20)),
            attach_to=drone_actor)
        drone_chase_cam.listen(_listen("drone_chase")); actors.append(drone_chase_cam)

        # No Traffic Manager — we drive the truck directly from the main loop
        # using compute_truck_control() (smooth 30 Hz P-controllers). Cache the
        # HD map handle here so we don't re-fetch it every frame.
        world_map = world.get_map()

        # Pygame
        pygame.init()
        display = pygame.display.set_mode((DISPLAY_W, DISPLAY_H))
        pygame.display.set_caption(
            "CarlaAir — Air-Ground Convoy  |  drone rides + escorts + lands  |  ESC=quit")
        clock = pygame.time.Clock()
        font = pygame.font.SysFont("monospace", 18, bold=True)
        font_lg = pygame.font.SysFont("monospace", 26, bold=True)

        t0 = time.time()
        prev_phase = None
        truck_driving = False          # True once we start issuing apply_control each frame
        landed_lock = False            # once True, phase is forced to LANDED for the rest of the run
        landed_offset = None           # drone offset in truck frame at snap moment (no-jump lock)
        running = True

        print("\n  [run] Scenario start — scheduled phases:")
        print(f"        MOUNTED    0.0 - {T_MOUNT_END:4.1f}s")
        print(f"        FOLLOWING  {T_MOUNT_END:4.1f} - {T_FOLLOW_END:4.1f}s")
        print(f"        TAKING_OFF {T_FOLLOW_END:4.1f} - {T_TAKEOFF_END:4.1f}s")
        print(f"        ESCORTING  {T_TAKEOFF_END:4.1f} - {T_ESCORT_END:4.1f}s")
        print(f"        LANDING    {T_ESCORT_END:4.1f} - {T_LANDING_END:4.1f}s")
        print(f"        LANDED     {T_LANDING_END:4.1f}+\n")

        while running:
            clock.tick(30)
            t = time.time() - t0
            if t > args.duration:
                break

            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE:
                    running = False

            # Start driving the truck at end of MOUNTED phase
            if not truck_driving and t >= T_MOUNT_END - 0.2:
                truck_driving = True
                print(f"  [{t:5.1f}s] truck driver ENGAGED "
                      f"(target {truck_target_speed*3.6:.0f} km/h)")

            # Per-frame truck control (continuous, smooth, replaces TM autopilot)
            if truck_driving:
                truck.apply_control(
                    compute_truck_control(truck, world_map, truck_target_speed))

            phase = phase_from_time(t)
            if landed_lock:
                phase = Phase.LANDED   # sticky: once locked on roof, stay locked

            if phase != prev_phase:
                print(f"  [{t:5.1f}s] → phase {phase}")
                prev_phase = phase

                # Transition side-effects
                if phase == Phase.TAKING_OFF:
                    # Re-arm (pose-lock disabled physics) and start climb
                    air_client.enableApiControl(True)
                    air_client.armDisarm(True)
                    # command: climb to (truck roof z) - ESCORT_ALT in NED (Z is negative up)
                    roof = truck_roof_world(truck)
                    _, _, nz_target = carla_to_ned(roof.x, roof.y, roof.z + ESCORT_ALT,
                                                   ox, oy, oz)
                    air_client.moveToZAsync(nz_target, TAKEOFF_CLIMB_RATE)

                elif phase == Phase.ESCORTING:
                    pass  # handled per-frame

                elif phase == Phase.LANDING:
                    pass  # handled per-frame

                elif phase == Phase.LANDED:
                    air_client.cancelLastTask()

            # Per-frame drone control based on phase
            if phase in (Phase.MOUNTED, Phase.FOLLOWING, Phase.LANDED):
                pose_lock_drone_to_truck(air_client, truck, ox, oy, oz)

            elif phase == Phase.TAKING_OFF:
                # climb command already issued on phase entry; nothing to do
                pass

            elif phase in (Phase.ESCORTING, Phase.LANDING):
                # Unified velocity controller: feed-forward truck velocity + P-correction
                # toward a target point (roof + altitude_above_roof). LANDING linearly
                # ramps altitude_above_roof from ESCORT_ALT → 0.3m over the phase.
                roof = truck_roof_world(truck)
                yaw_deg = truck.get_transform().rotation.yaw

                if phase == Phase.ESCORTING:
                    alt_above_roof = ESCORT_ALT
                else:
                    # Ease-out descent: fast up front, gentler near the roof.
                    p = (t - T_ESCORT_END) / max(0.1, T_LANDING_END - T_ESCORT_END)
                    p = min(1.0, max(0.0, p))
                    ease = 1.0 - (1.0 - p) ** 2   # ease-out quadratic
                    alt_above_roof = ESCORT_ALT * (1.0 - ease) + LANDING_FINAL_ALT * ease

                tgt_nx, tgt_ny, tgt_nz = carla_to_ned(
                    roof.x, roof.y, roof.z + alt_above_roof, ox, oy, oz)

                ds = air_client.getMultirotorState().kinematics_estimated.position
                err_x = tgt_nx - ds.x_val
                err_y = tgt_ny - ds.y_val
                err_z = tgt_nz - ds.z_val

                # Truck velocity feed-forward (CARLA x/y share sign with AirSim NED x/y)
                tv = truck.get_velocity()
                vx_cmd = tv.x + KP_HORIZ * err_x
                vy_cmd = tv.y + KP_HORIZ * err_y
                vz_cmd = KP_VERT * err_z       # NED: +z is DOWN

                # Clamp
                vx_cmd = max(-V_MAX_HORIZ, min(V_MAX_HORIZ, vx_cmd))
                vy_cmd = max(-V_MAX_HORIZ, min(V_MAX_HORIZ, vy_cmd))
                vz_cmd = max(-V_MAX_VERT, min(V_MAX_VERT, vz_cmd))

                air_client.moveByVelocityAsync(
                    vx_cmd, vy_cmd, vz_cmd, 0.25,
                    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                    yaw_mode=airsim.YawMode(False, yaw_deg))

                if phase == Phase.LANDING:
                    dh = math.sqrt(err_x ** 2 + err_y ** 2)
                    dv = abs(err_z)
                    # Tight snap thresholds — drone is already close to the mount
                    # point; visible teleport on snap is ≤ 0.5 m horizontal.
                    # Failsafe: if we've reached the end of the phase, snap anyway.
                    snap = (p > 0.7 and dh < LANDING_LOCK_DIST and dv < LANDING_LOCK_VERT) \
                           or p >= 0.99
                    if snap:
                        print(f"  [{t:5.1f}s] touch-down lock — hor={dh:.2f}m vert={dv:.2f}m  p={p:.2f}")
                        try: air_client.cancelLastTask()
                        except Exception: pass
                        pose_lock_drone_to_truck(air_client, truck, ox, oy, oz)
                        landed_lock = True   # sticky: forces Phase.LANDED from next frame on

            # ─── render 2x2 grid ───
            display.fill((15, 15, 20))
            panels = [
                ("chase",        0,        0,       "Truck Chase"),
                ("spec",         PW,       0,       "Truck Rearview (→ drone)"),
                ("drone_fpv",    0,        PH,      "Drone FPV"),
                ("drone_chase",  PW,       PH,      "Drone 3rd-Person"),
            ]
            for name, px, py, label in panels:
                img = images.get(name)
                if img is not None:
                    try:
                        surf = pygame.surfarray.make_surface(img.swapaxes(0, 1))
                        if surf.get_width() != PW or surf.get_height() != PH:
                            surf = pygame.transform.scale(surf, (PW, PH))
                        display.blit(surf, (px, py))
                    except Exception:
                        pass
                lbl = font.render(label, True, (255, 255, 255))
                bg = pygame.Surface((lbl.get_width() + 14, lbl.get_height() + 6))
                bg.set_alpha(180); bg.fill((0, 0, 0))
                display.blit(bg, (px + 8, py + 8))
                display.blit(lbl, (px + 15, py + 11))

            # dividers
            pygame.draw.line(display, (100, 100, 100), (PW, 0), (PW, DISPLAY_H), 2)
            pygame.draw.line(display, (100, 100, 100), (0, PH), (DISPLAY_W, PH), 2)

            # HUD
            v = truck.get_velocity()
            spd = 3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            try:
                ds = air_client.getMultirotorState().kinematics_estimated.position
                roof = truck_roof_world(truck)
                _, _, nz_roof = carla_to_ned(roof.x, roof.y, roof.z, ox, oy, oz)
                alt = nz_roof - ds.z_val   # NED: negative z = up, so this = drone height above roof
            except Exception:
                alt = 0.0
            hud = (f" phase={phase:11s}   t={t:5.1f}/{args.duration:4.0f}s   "
                   f"truck={spd:4.0f} km/h   drone_alt={alt:+5.1f}m   ESC=quit")
            hs = font.render(hud, True, (0, 230, 180))
            hbg = pygame.Surface((DISPLAY_W, 28)); hbg.set_alpha(200); hbg.fill((0, 0, 0))
            display.blit(hbg, (0, DISPLAY_H - 28))
            display.blit(hs, (8, DISPLAY_H - 26))

            # Big phase label
            plabel = font_lg.render(phase, True, (255, 220, 60))
            pbg = pygame.Surface((plabel.get_width() + 20, plabel.get_height() + 8))
            pbg.set_alpha(180); pbg.fill((0, 0, 0))
            display.blit(pbg, (DISPLAY_W // 2 - plabel.get_width() // 2 - 10, 40))
            display.blit(plabel, (DISPLAY_W // 2 - plabel.get_width() // 2, 44))

            pygame.display.flip()

        print(f"\n  [end] Scenario complete ({time.time() - t0:.1f}s elapsed)")

    except KeyboardInterrupt:
        print("\n  [interrupt] user cancelled")
    except Exception as e:
        print(f"\n  [ERROR] {e}")
        import traceback; traceback.print_exc()
    finally:
        # release drone control, cleanup
        if air_client is not None:
            try:
                air_client.cancelLastTask()
                air_client.armDisarm(False)
                air_client.enableApiControl(False)
            except Exception: pass
        for a in actors:
            try:
                if hasattr(a, "stop"): a.stop()
            except Exception: pass
            try: a.destroy()
            except Exception: pass
        if original_settings is not None:
            try: world.apply_settings(original_settings)
            except Exception: pass
        try: pygame.quit()
        except Exception: pass
        print("  [cleanup] done.\n")


if __name__ == "__main__":
    sys.exit(main() or 0)
