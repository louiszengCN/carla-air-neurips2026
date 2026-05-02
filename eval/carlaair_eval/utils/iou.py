"""Image-space IoU between the UAV camera frustum and the UGV bounding box,
used by the RSR metric (paper App. C.4).

We approximate the UGV bbox as its 8 world-corner vertices, project each
corner with a pinhole intrinsic, take the AABB on the image plane, then
compute IoU against the camera image AABB. Corners whose camera-frame z<=0
fail the visibility test → IoU = 0.
"""
from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np


def _pinhole_intrinsics(fov_deg: float, w: int, h: int) -> np.ndarray:
    f = (w / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return np.array([[f, 0, w / 2.0],
                     [0, f, h / 2.0],
                     [0, 0, 1.0]])


def _world_to_cam(p_world: np.ndarray, cam_pos: np.ndarray,
                  cam_R: np.ndarray) -> np.ndarray:
    """cam_R: world→camera (3x3). Camera convention: +Z forward, +X right, +Y down."""
    return cam_R @ (p_world - cam_pos)


def project_box_aabb(corners_world: List[Tuple[float, float, float]],
                     cam_pos: Tuple[float, float, float],
                     cam_R: np.ndarray,
                     fov_deg: float, img_w: int, img_h: int) -> Tuple[float, float, float, float] | None:
    """Project an arbitrary set of world points into camera image and return
    the 2D AABB (x0, y0, x1, y1) clipped to image bounds.

    Returns None if all corners are behind the camera.
    """
    K = _pinhole_intrinsics(fov_deg, img_w, img_h)
    cam_pos_a = np.asarray(cam_pos, dtype=float)
    pts = []
    for c in corners_world:
        pc = _world_to_cam(np.asarray(c, dtype=float), cam_pos_a, cam_R)
        if pc[2] <= 0.05:
            continue
        u = K @ pc
        u = u / u[2]
        pts.append((float(u[0]), float(u[1])))
    if not pts:
        return None
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    x0, x1 = max(0.0, min(xs)), min(float(img_w), max(xs))
    y0, y1 = max(0.0, min(ys)), min(float(img_h), max(ys))
    if x1 <= x0 or y1 <= y0:
        return None
    return x0, y0, x1, y1


def iou_with_image(box_aabb: Tuple[float, float, float, float] | None,
                   img_w: int, img_h: int) -> float:
    """IoU between projected UGV bbox and the full image rectangle.

    Per paper definition: "IoU >= 0.15 between the UAV camera view and the
    UGV bounding box". We treat the UAV camera view as the image rectangle.
    """
    if box_aabb is None:
        return 0.0
    x0, y0, x1, y1 = box_aabb
    inter = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    area_box = max(0.0, (x1 - x0) * (y1 - y0))
    area_img = float(img_w) * float(img_h)
    union = area_img + area_box - inter
    if union <= 0:
        return 0.0
    return inter / union


def yaw_pitch_roll_to_R(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    """World→camera rotation for a CARLA-style camera with given yaw/pitch/roll.

    Camera convention used here: +Z forward, +X right, +Y down (right-handed).
    """
    y = math.radians(yaw_deg)
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    # World→body (yaw about world Z, pitch about body Y, roll about body X)
    Rz = np.array([[ math.cos(y),  math.sin(y), 0],
                   [-math.sin(y),  math.cos(y), 0],
                   [           0,            0, 1]])
    Ry = np.array([[ math.cos(p), 0, -math.sin(p)],
                   [           0, 1,            0],
                   [ math.sin(p), 0,  math.cos(p)]])
    Rx = np.array([[1,           0,            0],
                   [0, math.cos(r),  math.sin(r)],
                   [0,-math.sin(r),  math.cos(r)]])
    R_body = Rx @ Ry @ Rz
    # Body (forward=+x, left=+y, up=+z) → camera (forward=+z, right=+x, down=-y)
    body_to_cam = np.array([[ 0, -1,  0],
                            [ 0,  0, -1],
                            [ 1,  0,  0]])
    return body_to_cam @ R_body
