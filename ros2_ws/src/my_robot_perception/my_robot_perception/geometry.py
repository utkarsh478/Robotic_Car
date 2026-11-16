"""3-D geometry for the detectors (numpy only, no ROS).  SPEC §9.1 steps 4-5, §9.2.

Conventions: depth images hold z-depth (metres along the optical axis), camera_optical_link is
x right, y down, z forward. Points are transformed with 4x4 matrices T_map_optical.
"""
import math

import numpy as np


def backproject(u, v, z, K):
    """pixels (u, v) with z-depth -> (N,3) points in the optical frame."""
    fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]
    x = (u - cx) / fx * z
    y = (v - cy) / fy * z
    return np.column_stack([x, y, z])


def transform(T, pts):
    return pts @ T[:3, :3].T + T[:3, 3]


def quat_to_matrix(x, y, z, w, t=(0.0, 0.0, 0.0)):
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = t
    return T


# ----------------------------------------------------------------------------- cylinder
def cylinder_axis(pts, cam_xy, radius, z_band=(0.01, 0.14), iters=15, height=0.15, weights=None,
                  min_side=30, cap_tol=0.015):
    """Axis (x, y) of an upright cylinder of known radius from points on its visible surface.

    Side surface (z inside `z_band`; the top cap would pull the fit): known-radius circle fit
    (Gauss-Newton) in the ground plane. Start value: the points' mean pushed away from the camera
    by pi/4 * r, which is exactly where the mean of a half circle sampled uniformly across the
    image (not in angle) lies.
    Top cap (DC-35): when fewer than `min_side` side points are seen (the side facing the camera
    is unlit and not red), the cap - a disk at z = height, lit from above - gives the axis as its
    centroid, weighted by depth^2 (`weights`) so the near half's denser pixels do not pull it.
    Returns (xy, info) or (None, info).
    """
    side = pts[(pts[:, 2] >= z_band[0]) & (pts[:, 2] <= z_band[1])]
    info = {"n_side": int(len(side))}
    if len(side) < min_side:
        cap_sel = np.abs(pts[:, 2] - height) <= cap_tol
        info["n_cap"] = int(cap_sel.sum())
        if info["n_cap"] < 10:
            return None, info
        w = np.ones(int(cap_sel.sum())) if weights is None else np.asarray(weights, float)[cap_sel]
        c = (pts[cap_sel, :2] * w[:, None]).sum(axis=0) / w.sum()
        info["method"] = "top-cap"
        return c, info
    p = side[:, :2]
    mean = p.mean(axis=0)
    view = mean - np.asarray(cam_xy, float)
    view /= max(np.linalg.norm(view), 1e-9)
    c0 = mean + (math.pi / 4.0) * radius * view
    c = c0.copy()
    for _ in range(iters):
        d = p - c
        rho = np.linalg.norm(d, axis=1)
        rho = np.maximum(rho, 1e-9)
        res = rho - radius
        J = -d / rho[:, None]                       # d(res)/dc
        H = J.T @ J
        if np.linalg.cond(H) > 1e8:
            break
        step = np.linalg.solve(H, -J.T @ res)
        c = c + step
        if np.linalg.norm(step) < 1e-6:
            break
    rms = float(np.sqrt(np.mean((np.linalg.norm(p - c, axis=1) - radius) ** 2)))
    info.update(rms=rms, fit_shift=float(np.linalg.norm(c - c0)))
    # The circle fit is exact on clean depth but amplifies depth noise (a 5 cm wide arc).
    # If its residual says the surface is noisy, or it wandered off, use the start value: an
    # average over all points, unbiased for a uniformly sampled half circle.
    if not np.all(np.isfinite(c)) or info["fit_shift"] > radius or rms > 0.2 * radius:
        info["method"] = "mean+push"
        return c0, info
    info["method"] = "circle-fit"
    return c, info


def cylinder_axis_spec(pts_center, cam_xy, radius):
    """SPEC §9.1 baseline: the back-projected centroid pushed r along the horizontal ray."""
    view = np.asarray(pts_center[:2], float) - np.asarray(cam_xy, float)
    view /= max(np.linalg.norm(view), 1e-9)
    return np.asarray(pts_center[:2], float) + radius * view


def plausible_cylinder(pts, radius, height, tol=0.04):
    """Reject blobs that cannot be the target: too wide, too tall, or floating."""
    if len(pts) < 10:
        return False, "too few 3-D points"
    z = pts[:, 2]
    if np.percentile(z, 95) > height + tol:
        return False, f"too tall (z95 {np.percentile(z, 95):.2f} m)"
    if np.percentile(z, 5) > height + tol:          # a cap-only view sits AT the height (DC-35)
        return False, "floating"
    xy = pts[:, :2] - np.median(pts[:, :2], axis=0)
    spread = float(np.percentile(np.linalg.norm(xy, axis=1), 95))
    if spread > 2.0 * radius + tol:
        return False, f"too wide ({spread:.2f} m)"
    return True, ""


# ----------------------------------------------------------------------------- box
def box_floor_centre(pts, cam_xy, size=0.30, height=0.15, z_band=(0.005, 0.17)):
    """Centre of the box floor (x, y) and yaw from points on an open-top box of known size.

    Yaw from the minimum-area rectangle of the points in the ground plane. Then per box axis:
    if the camera lies outside the box's extent along that axis, the nearest face is an OUTER
    wall (always visible) -> centre = that face + size/2; otherwise (looking along the other
    axis) both side walls are seen from the inside/edge -> centre = midpoint.
    Returns (xy, yaw, info) or (None, None, info).
    """
    import cv2   # local: geometry stays importable without OpenCV for the cylinder code
    sel = pts[(pts[:, 2] >= z_band[0]) & (pts[:, 2] <= z_band[1])]
    info = {"n": int(len(sel))}
    if len(sel) < 20:
        return None, None, info
    p = sel[:, :2]
    (_, _), (_, _), ang = cv2.minAreaRect(p.astype(np.float32))
    yaw = math.radians(ang) % (math.pi / 2)
    axes = [np.array([math.cos(yaw), math.sin(yaw)]), np.array([-math.sin(yaw), math.cos(yaw)])]
    cam = np.asarray(cam_xy, float)
    centre = np.zeros(2)
    for k, a in enumerate(axes):
        s = p @ a
        lo, hi = np.percentile(s, 1), np.percentile(s, 99)
        c_cam = cam @ a
        if c_cam < lo:
            ck = lo + size / 2
        elif c_cam > hi:
            ck = hi - size / 2
        else:
            ck = 0.5 * (lo + hi)
        info[f"extent{k}"] = float(hi - lo)
        centre += ck * a
    return centre, yaw, info


def plausible_box(pts, size=0.30, height=0.15, tol=0.06):
    if len(pts) < 20:
        return False, "too few 3-D points"
    if np.percentile(pts[:, 2], 95) > height + tol:
        return False, "too tall"
    import cv2
    (_, _), (w, h), _ = cv2.minAreaRect(pts[:, :2].astype(np.float32))
    if max(w, h) > size + tol:
        return False, f"too large ({max(w, h):.2f} m side)"
    return True, ""


# ----------------------------------------------------------------------------- tracking
class Tracker:
    """SPEC §9.1 step 7: an observation within `gate` of a track updates it, otherwise starts
    a new one; tracks unseen for `timeout` s are dropped. Mean/std over the last `window`
    observations, so early, far-away estimates stop mattering once the robot is close."""

    def __init__(self, gate=0.15, timeout=10.0, window=30):
        self.gate, self.timeout, self.window = gate, timeout, window
        self.tracks = []

    def reset(self):
        self.tracks = []

    def update(self, xy, t, extra=None):
        xy = np.asarray(xy, float)
        best, dbest = None, self.gate
        for tr in self.tracks:
            d = float(np.linalg.norm(tr["mean"] - xy))
            if d < dbest:
                best, dbest = tr, d
        if best is None:
            best = {"obs": [], "count": 0, "mean": xy, "last": t, "extra": extra}
            self.tracks.append(best)
        best["obs"].append(xy)
        best["obs"] = best["obs"][-self.window:]
        best["count"] += 1
        best["mean"] = np.mean(best["obs"], axis=0)
        best["last"] = t
        best["extra"] = extra
        return best

    def prune(self, t):
        self.tracks = [tr for tr in self.tracks if t - tr["last"] <= self.timeout]

    def best(self):
        if not self.tracks:
            return None
        return max(self.tracks, key=lambda tr: tr["count"])

    @staticmethod
    def stddev(tr):
        obs = np.asarray(tr["obs"])
        if len(obs) < 2:
            return float("nan")
        return float(np.sqrt(np.mean(np.sum((obs - obs.mean(axis=0)) ** 2, axis=1))))
