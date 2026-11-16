#!/usr/bin/env python3
"""Verify the SRDF stow/transport states against SPEC §3.5 (+ DC-20 grasp height).  Phase 7.

Checks for each arm state (fingers open 0.04 for stow; for transport also with the target
held: fingers 0.025, cylinder r 0.025 h 0.15, top 0.025 above tool0 along the tool axis):
  - joints inside limits with >= 0.05 rad margin
  - tool pointing straight down (< 0.02 rad)
  - tool0 base-x in [-0.20, 0.17]; tool0 >= 0.33 above ground
  - held object bottom >= chassis top + 0.03
  - every arm/gripper/object surface point inside the Nav2 footprint with >= 0.02 m margin
  - clearance >= 0.02 m between every moving part (and the object) and the chassis/sensors,
    and between non-adjacent arm links
  - forearm, wrist, gripper and object outside the camera frustum (+0.03 rad margin)
--search: grid-search (yaw 0, top-down) for the state with the largest worst-case margin.
"""
import argparse
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_acm import SRDF, load_robot  # noqa: E402
from robot_collision import Shape, make_tf, pair_clearance  # noqa: E402

ARM = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint"]
FINGERS = ["left_finger_joint", "right_finger_joint"]
GROUND_Z = 0.0                        # base_footprint is on the ground
CHASSIS_TOP = 0.075 + 0.11            # base_link z + chassis top in base_link
FOOTPRINT = (-0.27, 0.31, -0.27, 0.27)   # x_min, x_max, y_min, y_max (nav2 footprint)
BASE_LINK_Z = 0.075
CAM = {"xyz": (0.20, -0.12, 0.075 + 0.425), "pitch": 0.60, "hfov": 1.2, "w": 640, "h": 480}
STATIC = ["base_link", "left_wheel_link", "right_wheel_link", "front_caster_link", "rear_caster_link",
          "lidar_link", "camera_mast_link", "camera_link", "arm_base_link"]
MOVING = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_link", "gripper_base_link",
          "left_finger_link", "right_finger_link"]
IN_VIEW_CHECK = ["forearm_link", "wrist_link", "gripper_base_link", "left_finger_link", "right_finger_link"]
OBJECT_TOUCH = {"wrist_link", "gripper_base_link", "left_finger_link", "right_finger_link"}
H = 0.005


def srdf_states():
    txt = open(SRDF).read()
    out = {}
    for name, group, body in re.findall(r'<group_state name="([^"]+)" group="([^"]+)">(.*?)</group_state>', txt, re.S):
        out[(name, group)] = {j: float(v) for j, v in re.findall(r'<joint name="([^"]+)" value="([^"]+)"', body)}
    return out


def frustum_angles(p):
    """angular margin of point(s) p (base_footprint) w.r.t. the camera frustum; > 0 = outside."""
    c = np.asarray(CAM["xyz"])
    sp, cp = math.sin(CAM["pitch"]), math.cos(CAM["pitch"])
    fwd, up, left = np.array([cp, 0, -sp]), np.array([sp, 0, cp]), np.array([0, 1, 0])
    d = p - c
    depth = d @ fwd
    half_h = CAM["hfov"] / 2
    half_v = math.atan(math.tan(half_h) * CAM["h"] / CAM["w"])
    with np.errstate(divide="ignore", invalid="ignore"):
        ah = np.abs(np.arctan2(d @ left, depth)) - half_h
        av = np.abs(np.arctan2(d @ up, depth)) - half_v
    m = np.maximum(ah, av)
    return np.where(depth <= 0.05, np.pi, m)          # behind / at the lens: not visible


def evaluate(robot, q_arm, fingers, with_object):
    q = {j: np.array([v]) for j, v in zip(ARM, q_arm)}
    q.update({j: np.array([fingers]) for j in FINGERS})
    T = robot.fk(q)
    tool = T["tool0"][0]
    m = {}
    lims = {j["name"]: (j["lower"], j["upper"]) for j in robot.moving}
    m["joint margin"] = min(min(v - lims[j][0], lims[j][1] - v) for j, v in zip(ARM, q_arm))
    m["tool tilt from down"] = math.acos(max(-1.0, min(1.0, -tool[2, 2])))
    m["tool0 base-x"] = tool[0, 3]
    m["tool0 above ground"] = tool[2, 3] - GROUND_Z
    parts = {l: (robot.shapes[l][0], T[l]) for l in MOVING}
    if with_object:
        obj = Shape("cylinder", [0.025, 0.15], make_tf([0, 0, 0.05], [0, 0, 0]), "object")
        parts["object"] = (obj, T["tool0"])
        bottom = tool[:3, 3] + tool[:3, 2] * 0.125
        m["object bottom - chassis top"] = bottom[2] - CHASSIS_TOP
    # footprint + frustum on surface samples
    fp, view = [], []
    for name, (s, Tl) in parts.items():
        P = s.surface_points(H)
        W = (Tl[0] @ s.tf_link)[:3, :3] @ P.T + (Tl[0] @ s.tf_link)[:3, 3:4]
        W = W.T
        x0, x1, y0, y1 = FOOTPRINT
        fp.append(np.min(np.c_[W[:, 0] - x0, x1 - W[:, 0], W[:, 1] - y0, y1 - W[:, 1]]))
        if name in IN_VIEW_CHECK or name == "object":
            view.append(frustum_angles(W).min())
    m["footprint margin"] = min(fp)
    m["camera frustum margin (rad)"] = min(view)
    # clearances
    adj = robot.adjacent_pairs()
    cache = {}
    worst = (1e9, "")
    for name, (s, Tl) in parts.items():
        others = [(o, robot.shapes[o][0], T[o]) for o in STATIC + MOVING if o != name]
        for o, so, To in others:
            if name == "object":
                if o in OBJECT_TOUCH:
                    continue
            elif o in MOVING and MOVING.index(o) <= MOVING.index(name):
                continue
            elif tuple(sorted((name, o))) in adj:
                continue
            d = float(pair_clearance(s, Tl, so, To, cache, H)[0])
            if d < worst[0]:
                worst = (d, f"{name}-{o}")
    m["min clearance"] = worst[0]
    m["closest pair"] = worst[1]
    return m


LIMITS = {
    "joint margin": (">=", 0.05),
    "tool tilt from down": ("<=", 0.02),
    "tool0 base-x": ("in", (-0.20, 0.17)),
    "tool0 above ground": (">=", 0.33),
    "object bottom - chassis top": (">=", 0.03),
    "footprint margin": (">=", 0.02),
    "camera frustum margin (rad)": (">=", 0.03),
    "min clearance": (">=", 0.02),
}


def passes(m):
    res = {}
    for k, (op, lim) in LIMITS.items():
        if k not in m:
            continue
        v = m[k]
        res[k] = (lim[0] <= v <= lim[1]) if op == "in" else (v >= lim if op == ">=" else v <= lim)
    return res


def worst_margin(m):
    """smallest normalised margin across the limits (for --search)."""
    terms = []
    for k, (op, lim) in LIMITS.items():
        if k not in m:
            continue
        v = m[k]
        if op == "in":
            terms.append(min(v - lim[0], lim[1] - v) / 0.05)
        elif op == ">=":
            terms.append((v - lim) / max(abs(lim), 0.05))
        else:
            terms.append((lim - v) / lim)
    return min(terms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf")
    ap.add_argument("--search", action="store_true")
    a = ap.parse_args()
    robot = load_robot(a.urdf)

    if a.search:
        best = []
        for q1 in np.arange(-0.95, 1.95, 0.05):
            for q2 in np.arange(0.0, 2.55, 0.05):
                q3 = math.pi - q1 - q2
                if not -1.95 <= q3 <= 1.95:
                    continue
                q = (0.0, q1, q2, q3)
                m = evaluate(robot, q, 0.025, True)
                if all(passes(m).values()):
                    best.append((worst_margin(m), q, m))
        best.sort(key=lambda b: -b[0])
        print(f"{len(best)} feasible top-down states; best 8:")
        for s, q, m in best[:8]:
            print(f"  score {s:5.2f}  q=({', '.join(f'{v:+.3f}' for v in q)})  tool0 x {m['tool0 base-x']:.3f} "
                  f"h {m['tool0 above ground']:.3f}  clear {m['min clearance']:.3f} ({m['closest pair']})  "
                  f"fp {m['footprint margin']:.3f}  view {m['camera frustum margin (rad)']:.2f}")
        return

    states = srdf_states()
    fails = 0
    for name in ("stow", "transport"):
        q = states[(name, "arm")]
        q_arm = [q[j] for j in ARM]
        cases = [("fingers open", 0.04, False)]
        if name == "transport":
            cases.append(("holding target", 0.025, True))
        for label, f, obj in cases:
            m = evaluate(robot, q_arm, f, obj)
            res = passes(m)
            print(f"{name} ({label}) q=({', '.join(f'{v:+.4f}' for v in q_arm)}):")
            for k, ok in res.items():
                op, lim = LIMITS[k]
                lim_s = f"[{lim[0]}, {lim[1]}]" if op == "in" else f"{op} {lim}"
                extra = f"  ({m['closest pair']})" if k == "min clearance" else ""
                print(f"  {'PASS' if ok else 'FAIL'}  {k:30s} {m[k]:+.4f}  {lim_s}{extra}")
                fails += 0 if ok else 1
    print(f"\nnamed states: {'ALL PASS' if fails == 0 else f'{fails} FAILURE(S)'}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
