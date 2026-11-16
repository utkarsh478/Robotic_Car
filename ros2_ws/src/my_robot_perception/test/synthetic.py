"""Tiny numpy ray caster for perception tests: Lambert-shaded RGB + z-depth images of upright
cylinders, oriented boxes, open-top boxes, the ground and a wall, seen by the robot's camera
(same mounting as sensors.xacro and the same intrinsics as Gazebo: hfov 1.2, 640x480)."""
import math

import numpy as np

W, H, HFOV = 640, 480, 1.2
FX = (W / 2) / math.tan(HFOV / 2)
K = [[FX, 0.0, W / 2], [0.0, FX, H / 2], [0.0, 0.0, 1.0]]
SUN = np.array([-0.4, 0.2, -0.9]) / np.linalg.norm([-0.4, 0.2, -0.9])

RED = (0.85, 0.05, 0.05)
YELLOW = (0.9, 0.8, 0.05)
GREEN = (0.1, 0.7, 0.15)
O1_ORANGE = (0.75, 0.45, 0.2)       # mission_world obstacle O1
ARM_ORANGE = (0.95, 0.50, 0.10)     # robot arm material


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def tf(xyz, r, p, y):
    T = np.eye(4)
    T[:3, :3] = rpy(r, p, y)
    T[:3, 3] = xyz
    return T


def camera_pose(x, y, yaw):
    """T_map_optical for the robot's base_footprint at (x, y, yaw) — sensors.xacro numbers."""
    return (tf([x, y, 0.0], 0, 0, yaw) @ tf([0.20, -0.12, 0.075 + 0.425], 0, 0.60, 0)
            @ tf([0, 0, 0], -math.pi / 2, 0, -math.pi / 2))


def robot_pose_for(target_xy, rng, bearing, heading):
    """Robot pose so that the CAMERA sees `target_xy` at horizontal range `rng` and `bearing`
    (relative to the robot heading)."""
    a = heading + bearing
    cam = np.asarray(target_xy, float) - rng * np.array([math.cos(a), math.sin(a)])
    off = np.array([[math.cos(heading), -math.sin(heading)], [math.sin(heading), math.cos(heading)]]) @ [0.20, -0.12]
    base = cam - off
    return float(base[0]), float(base[1]), heading


class Scene:
    def __init__(self, wall_y=None):
        self.objs = []          # (kind, params, color)
        self.wall_y = wall_y

    def cylinder(self, xy, r=0.025, h=0.15, color=RED, side_color=None):
        """side_color: render the side in another colour (e.g. an unlit, non-red side, DC-35)"""
        self.objs.append(("cyl", (np.asarray(xy, float), r, h, side_color), color))
        return self

    def box(self, xy, size, z0=0.0, yaw=0.0, color=O1_ORANGE):
        self.objs.append(("box", (np.asarray(xy, float), np.asarray(size, float), z0, yaw), color))
        return self

    def open_box(self, xy, yaw=0.0, outer=0.30, h=0.15, t=0.01, color=GREEN):
        c = np.asarray(xy, float)
        R = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        self.box(c, (outer, outer, t), 0.0, yaw, color)                                   # floor
        for dx, dy, sx, sy in ((outer / 2 - t / 2, 0, t, outer), (-(outer / 2 - t / 2), 0, t, outer),
                               (0, outer / 2 - t / 2, outer - 2 * t, t), (0, -(outer / 2 - t / 2), outer - 2 * t, t)):
            self.box(c + R @ [dx, dy], (sx, sy, h), 0.0, yaw, color)
        return self

    def render(self, T_map_opt):
        v, u = np.mgrid[0:H, 0:W]
        # OpenCV/ROS pinhole convention: integer pixel coordinates are pixel centres
        d_opt = np.stack([(u - K[0][2]) / FX, (v - K[1][2]) / FX, np.ones_like(u, float)], -1).reshape(-1, 3)
        d = d_opt @ T_map_opt[:3, :3].T
        o = T_map_opt[:3, 3]
        n = len(d)
        best = np.full(n, np.inf)
        col = np.zeros((n, 3))
        nrm = np.zeros((n, 3))

        def take(t, normal, color):
            m = (t > 1e-6) & (t < best)
            best[m] = t[m]
            nrm[m] = normal[m] if normal.ndim == 2 else normal
            col[m] = color

        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(d[:, 2] < 0, -o[2] / d[:, 2], np.inf)        # ground
            take(t, np.array([0, 0, 1.0]), (0.45, 0.45, 0.45))
            if self.wall_y is not None:
                t = (self.wall_y - o[1]) / d[:, 1]
                take(np.where(t > 0, t, np.inf), np.array([0, -1.0, 0]), (0.6, 0.6, 0.6))
            for kind, prm, color in self.objs:
                if kind == "cyl":
                    c, r, h, side_color = prm
                    ox, oy = o[0] - c[0], o[1] - c[1]
                    a = d[:, 0] ** 2 + d[:, 1] ** 2
                    b = 2 * (ox * d[:, 0] + oy * d[:, 1])
                    cc = ox * ox + oy * oy - r * r
                    disc = b * b - 4 * a * cc
                    ts = (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a)
                    z = o[2] + ts * d[:, 2]
                    ts = np.where((disc >= 0) & (z >= 0) & (z <= h), ts, np.inf)
                    pn = np.column_stack([ox + ts * d[:, 0], oy + ts * d[:, 1], np.zeros(n)]) / r
                    take(ts, np.nan_to_num(pn), color if side_color is None else side_color)
                    tc = (h - o[2]) / d[:, 2]
                    px, py = ox + tc * d[:, 0], oy + tc * d[:, 1]
                    take(np.where((tc > 0) & (px * px + py * py <= r * r), tc, np.inf), np.array([0, 0, 1.0]), color)
                else:
                    c, s, z0, yaw = prm
                    cy, sy = math.cos(-yaw), math.sin(-yaw)
                    ol = np.array([cy * (o[0] - c[0]) - sy * (o[1] - c[1]), sy * (o[0] - c[0]) + cy * (o[1] - c[1]),
                                   o[2] - (z0 + s[2] / 2)])
                    dl = np.column_stack([cy * d[:, 0] - sy * d[:, 1], sy * d[:, 0] + cy * d[:, 1], d[:, 2]])
                    t1 = (-s / 2 - ol) / dl
                    t2 = (s / 2 - ol) / dl
                    tmin = np.minimum(t1, t2)
                    tmax = np.maximum(t1, t2)
                    tn = np.nanmax(tmin, axis=1)
                    tf_ = np.nanmin(tmax, axis=1)
                    hit = (tn <= tf_) & (tf_ > 0)
                    axis = np.nanargmax(tmin, axis=1)
                    nl = np.zeros((n, 3))
                    nl[np.arange(n), axis] = -np.sign(dl[np.arange(n), axis])
                    nw = np.column_stack([math.cos(yaw) * nl[:, 0] - math.sin(yaw) * nl[:, 1],
                                          math.sin(yaw) * nl[:, 0] + math.cos(yaw) * nl[:, 1], nl[:, 2]])
                    take(np.where(hit, tn, np.inf), nw, color)
        shade = 0.35 + 0.65 * np.clip(-(nrm @ SUN), 0, 1)
        rgb = np.clip(col * shade[:, None] * 255, 0, 255).astype(np.uint8).reshape(H, W, 3)
        depth = best.reshape(H, W).astype(np.float32)
        depth[~np.isfinite(depth)] = np.inf
        return rgb, depth
