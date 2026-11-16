#!/usr/bin/env python3
"""Phase-4 gate: drive Nav2 through tools/gates/nav_goals.yaml and score every goal against
Gazebo ground truth. Run while nav_sim.launch.xml is up.

PASS criteria (PHASES.md P4):
  every goal SUCCEEDED within 120 s (sim)
  final error vs truth < 0.15 m and < 0.15 rad
  robot stopped 1 s after each goal (|v| < 0.02 m/s, |w| < 0.05 rad/s)
  zero contacts with walls/obstacles (true pose vs world geometry, see nav_common)
  AMCL map->base_footprint vs truth < 0.10 m for the whole run (2 Hz samples)
Writes results/p04_nav.json. Exit 0 = all PASS.
"""
import json
import os
import sys

import rclpy
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_common import WS, NavMonitor  # noqa: E402

GOALS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nav_goals.yaml")
OUT = os.path.join(WS, "results", "p04_nav.json")
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def main():
    rclpy.init()
    m = NavMonitor("p04_nav_runner")
    ready = m.wait_ready()
    if ready is None:
        print("FAIL  not ready after 90 s: need /navigate_to_pose, TF map->base_footprint, /evaluator/robot_odom")
        sys.exit(1)
    print(f"nav ready after {ready:.1f} s; AMCL at ({m.amcl_pose()[0]:.2f}, {m.amcl_pose()[1]:.2f})", flush=True)
    m.spin_sim(2.0)

    goals = yaml.safe_load(open(GOALS))["goals"]
    rows = []
    for i, g in enumerate(goals, 1):
        r = m.go(g["x"], g["y"], g["yaw"], timeout=120.0)
        r["name"] = g["name"]
        rows.append(r)
        print(f"[{i:2d}] {g['name']:<28s} {r['status']:<10s} {r.get('time', 0):6.1f} s  "
              f"err {r.get('pos_err', float('nan')):.3f} m / {r.get('yaw_err', float('nan')):.3f} rad  "
              f"contacts {r.get('contacts', 0)}", flush=True)
        if r["status"] != "SUCCEEDED":
            m.spin_sim(3.0)  # let recoveries settle before the next goal

    ok_goals = [r for r in rows if r["status"] == "SUCCEEDED" and r["time"] <= 120.0]
    report(f"goals reached: {len(ok_goals)}/{len(rows)} SUCCEEDED within 120 s", len(ok_goals) == len(rows))
    worst_p = max((r.get("pos_err", 9) for r in rows), default=9)
    worst_y = max((r.get("yaw_err", 9) for r in rows), default=9)
    report("final error vs truth < 0.15 m and < 0.15 rad (all goals)", worst_p < 0.15 and worst_y < 0.15,
           f"worst {worst_p:.3f} m / {worst_y:.3f} rad")
    moving = [r["name"] for r in rows if abs(r["speed_1s_after"][0]) >= 0.02 or abs(r["speed_1s_after"][1]) >= 0.05]
    report("stopped 1 s after every goal", not moving, f"still moving after: {moving}" if moving else "")
    report("zero contacts with walls/obstacles", not m.contacts,
           f"{len(m.contacts)} contact samples, first at t {m.contacts[0][0]} s pose ({m.contacts[0][1]}, "
           f"{m.contacts[0][2]}) touching ({m.contacts[0][3]}, {m.contacts[0][4]})" if m.contacts else "")
    if m.amcl_err:
        worst = max(m.amcl_err, key=lambda e: e[1])
        mean = sum(e[1] for e in m.amcl_err) / len(m.amcl_err)
        report("AMCL vs truth < 0.10 m throughout", worst[1] < 0.10,
               f"max {worst[1]:.3f} m at t {worst[0]:.0f} s, mean {mean:.3f} m, "
               f"max yaw {max(e[2] for e in m.amcl_err):.3f} rad, {len(m.amcl_err)} samples")
    else:
        report("AMCL vs truth < 0.10 m throughout", False, "no samples")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(os.path.join(os.path.dirname(OUT), "p04_amcl_err.csv"), "w") as f:
        f.write("t_sim,err_xy,err_yaw\n")
        f.writelines(f"{t:.2f},{e:.4f},{y:.4f}\n" for t, e, y in m.amcl_err)
    with open(OUT, "w") as f:
        json.dump({"goals": rows, "contacts": m.contacts,
                   "amcl_err_max": max((e[1] for e in m.amcl_err), default=None),
                   "amcl_err_mean": (sum(e[1] for e in m.amcl_err) / len(m.amcl_err)) if m.amcl_err else None},
                  f, indent=2)
    n_fail = results.count(False)
    print(f"\nP04 navigation: {len(results) - n_fail}/{len(results)} PASS  (details: results/p04_nav.json, results/p04_amcl_err.csv)", flush=True)
    m.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
