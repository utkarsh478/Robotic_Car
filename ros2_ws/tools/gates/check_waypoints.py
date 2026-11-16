#!/usr/bin/env python3
"""P11 task 4: validate the mission's search waypoints and standoffs.

Offline (world geometry, mission_world.sdf via check_map.boundary_samples):
  - every waypoint >= 0.6 m from T1-T3 (SPEC §12)
  - the robot can spin in place at every waypoint: at 16 headings its footprint (+0.05 m) holds no
    obstacle boundary point
  - from every waypoint within camera range (2.7 m) of a target, at least one of the mission's
    standoff candidates (0.80 m, rotations 0/+-30/+-60/+-90 deg) is free of obstacles
  - the home-box standoff (0.85 m, approached from home) is free
--live: the same poses checked against Nav2's global costmap (/global_costmap/get_costmap) with
the mission's own test (core.pose_free: no lethal cell under the padded footprint, centre below
inscribed). Needs Nav2 running.
Exit code = number of failures.
"""
import argparse
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(WS, "src", "my_robot_mission"))
import check_map  # noqa: E402
from my_robot_mission import core  # noqa: E402

WORLD = os.path.join(WS, "src", "my_robot_bringup", "worlds", "mission_world.sdf")
PARAMS = os.path.join(WS, "src", "my_robot_mission", "config", "mission_params.yaml")
TARGETS = {"T1": (3.3, 2.4), "T2": (0.9, 2.5), "T3": (3.4, -1.2)}
BOX = (-3.0, -0.9)
fails = 0


def report(name, ok, detail=""):
    global fails
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  -> {detail}" if detail else ""), flush=True)
    fails += 0 if ok else 1


def load():
    with open(PARAMS) as f:
        p = yaml.safe_load(f)["mission_fsm"]["ros__parameters"]
    w = p["search_waypoints"]
    return [(w[i], w[i + 1]) for i in range(0, len(w), 2)], p


def footprint_hits(points, x, y, yaw, fp, pad):
    front, left, rear, right = fp
    c, s = math.cos(yaw), math.sin(yaw)
    for px, py in points:
        if abs(px - x) > 0.6 or abs(py - y) > 0.6:
            continue
        lx, ly = c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)
        if rear - pad <= lx <= front + pad and right - pad <= ly <= left + pad:
            return (round(px, 2), round(py, 2))
    return None


def standoffs(p):
    home = p["home"]
    out = []
    for name, t in TARGETS.items():
        for i, w in enumerate(load()[0]):
            if math.hypot(w[0] - t[0], w[1] - t[1]) <= 2.7:
                out.append((f"{name} from waypoint {i + 1}", t, w, p["target"]["standoff"]))
    out.append(("home box from home", BOX, home[:2], p["box"]["standoff"]))
    return out


def offline():
    wps, p = load()
    fp, rots = p["footprint"], p["standoff_rotations"]
    pts = [q for _, (_, ps) in check_map.boundary_samples(WORLD).items() for q in ps]
    for i, (x, y) in enumerate(wps):
        dt = min(math.hypot(x - t[0], y - t[1]) for t in TARGETS.values())
        report(f"waypoint {i + 1} ({x}, {y}) >= 0.6 m from T1-T3", dt >= 0.6, f"{dt:.2f} m")
        hits = [footprint_hits(pts, x, y, k * math.pi / 8, fp, 0.05) for k in range(16)]
        bad = [h for h in hits if h]
        dmin = min(math.hypot(px - x, py - y) for px, py in pts)
        report(f"waypoint {i + 1}: spin in place clear (footprint +0.05 m, 16 headings)", not bad,
               f"nearest obstacle {dmin:.2f} m" + (f", hits {bad[0]}" if bad else ""))
    for name, obj, frm, dist in standoffs(p):
        free = []
        for sx, sy, syaw, rot in core.standoff_candidates(obj, frm, dist, rots):
            if not footprint_hits(pts, sx, sy, syaw, fp, 0.05):
                free.append(rot)
        report(f"standoff for {name}: a free candidate exists", bool(free),
               f"free rotations {[int(r) for r in free]}" if free else "none free")
    covered = {n: [i + 1 for i, w in enumerate(wps) if math.hypot(w[0] - t[0], w[1] - t[1]) <= 2.7]
               for n, t in TARGETS.items()}
    report("every target is within 2.7 m (camera range) of some waypoint", all(covered.values()), str(covered))


def live():
    import rclpy
    from nav2_msgs.srv import GetCostmap
    from rclpy.node import Node

    wps, p = load()
    fp, pad, rots = p["footprint"], p["footprint_pad"], p["standoff_rotations"]
    rclpy.init()
    n = Node("p11_check_waypoints")
    cli = n.create_client(GetCostmap, "/global_costmap/get_costmap")
    if not cli.wait_for_service(timeout_sec=60.0):
        report("live: /global_costmap/get_costmap available", False)
        return
    fut = cli.call_async(GetCostmap.Request())
    rclpy.spin_until_future_complete(n, fut, timeout_sec=30.0)
    if not fut.done():
        report("live: costmap received", False)
        return
    m = fut.result().map
    md = m.metadata
    g = core.Grid(md.resolution, md.origin.position.x, md.origin.position.y, md.size_x, md.size_y, bytes(m.data))
    report("live: global costmap received", True, f"{md.size_x} x {md.size_y} cells at {md.resolution} m")
    for i, (x, y) in enumerate(wps):
        ok = [core.pose_free(g, x, y, k * math.pi / 4, fp, pad) for k in range(8)]
        report(f"live: waypoint {i + 1} ({x}, {y}) free at 8 headings", all(ok), f"{sum(ok)}/8")
    for name, obj, frm, dist in standoffs(p):
        free = [int(r) for sx, sy, syaw, r in core.standoff_candidates(obj, frm, dist, rots)
                if core.pose_free(g, sx, sy, syaw, fp, pad)]
        report(f"live: standoff for {name}", bool(free), f"free rotations {free}" if free else "none free")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true")
    a = ap.parse_args()
    live() if a.live else offline()
    sys.exit(fails)
