#!/usr/bin/env python3
"""Phase-7 gate, part 2: reachability sweep through MoveIt (plan only, from stow).

Grid (PHASES P7): base-x 0.30..0.50 step 0.025, y -0.15..0.15 step 0.05, tool0 height above
ground {0.11, 0.21, 0.29}, top-down. For every point:
  analytic IK (ik_cli) -> is the goal state collision-free?  (MoveIt /check_state_validity,
  and independently our geometry model, robot_collision.py, as a cross-check) -> if it is,
  MoveIt plans stow -> goal (OMPL RRTConnect, 2 s, 2 attempts).
PASS:
  - >= 95 % of IK-feasible, collision-free goals plan;  p95 planning time < 1.0 s
  - MoveIt and the geometry model agree on which goals are in collision, except borderline
    cases (|clearance| < 3 mm, both are approximations of the same shapes)
Goals that are themselves in collision (the arm would hit the camera mast/camera or lidar) are
reported, not counted as planner failures: no planner can reach a goal that collides.
Writes results/p07_reach_sweep.csv.
"""
import csv
import os
import re
import sys

import numpy as np
import rclpy

from moveit_common import ARM, FINGERS, WS, MoveItClient, ik, ik_cli_path, require, srdf_states

sys.path.insert(0, os.path.join(WS, "src", "my_robot_moveit_config", "scripts"))
from gen_acm import SRDF, load_robot  # noqa: E402
from robot_collision import Shape, make_tf, pair_clearance  # noqa: E402

BORDERLINE = 0.003
GROUND_LINKS = {"left_wheel_link", "right_wheel_link", "front_caster_link", "rear_caster_link", "base_link"}


def geometry_model():
    robot = load_robot(None)
    disabled = {tuple(sorted(m)) for m in re.findall(r'link1="([^"]+)" link2="([^"]+)"', open(SRDF).read())}
    links = sorted(robot.shapes)
    pairs = [(a, b) for i, a in enumerate(links) for b in links[i + 1:] if (a, b) not in disabled]
    floor = Shape("box", [3.0, 3.0, 0.02], make_tf([0, 0, -0.012], [0, 0, 0]), "floor")

    def clearance(q_arm):
        q = {j: np.array([v]) for j, v in zip(ARM, q_arm)}
        q.update({f: np.array([0.04]) for f in FINGERS})
        T = robot.fk(q)
        cache = {}
        worst = (9.0, "")
        for a, b in pairs:
            d = float(pair_clearance(robot.shapes[a][0], T[a], robot.shapes[b][0], T[b], cache, 0.003)[0])
            if d < worst[0]:
                worst = (d, f"{a}-{b}")
        for link in links:
            if link in GROUND_LINKS:
                continue
            d = float(pair_clearance(robot.shapes[link][0], T[link], floor, np.eye(4)[None], cache, 0.003)[0])
            if d < worst[0]:
                worst = (d, f"{link}-floor")
        return worst
    return clearance


def main():
    rclpy.init()
    n = MoveItClient("reach_sweep")
    require(n.wait_ready() is not None, "move_group not ready")
    require(n.add_box("floor", "base_footprint", (3.0, 3.0, 0.02), (0.0, 0.0, -0.012)), "could not add floor")
    n.spin_for(1.0)
    stow = srdf_states()["stow"][1]
    start = {**stow, **{f: 0.04 for f in FINGERS}}
    cli = ik_cli_path()
    model = geometry_model()

    rows = []
    for x in np.arange(0.30, 0.5001, 0.025):
        for y in np.arange(-0.15, 0.1501, 0.05):
            for h in (0.11, 0.21, 0.29):
                x, y = round(float(x), 3), round(float(y), 3)
                row = {"x": x, "y": y, "h": h}
                ok, q, txt = ik(cli, x, y, h)
                row["ik"] = ok
                if ok:
                    d, pair = model(q)
                    row.update(model_clearance=round(d, 4), model_pair=pair)
                    valid, contacts = n.state_valid({**dict(zip(ARM, q)), **{f: 0.04 for f in FINGERS}})
                    row.update(moveit_valid=valid, contacts=";".join("-".join(c) for c in contacts))
                    if valid:
                        r = n.go("arm", dict(zip(ARM, q)), plan_only=True, start=start)
                        row.update(plan=r["name"], planning_time=r["planning_time"])
                rows.append(row)
                tag = ("IK fail" if not ok else
                       f"in collision ({row['contacts']})" if not row["moveit_valid"] else
                       f"{row['plan']} {row['planning_time'] or 0.0:.3f} s")
                print(f"  x {x:.3f} y {y:+.2f} h {h:.2f}: {tag}", flush=True)

    os.makedirs(os.path.join(WS, "results"), exist_ok=True)
    out = os.path.join(WS, "results", "p07_reach_sweep.csv")
    keys = ["x", "y", "h", "ik", "moveit_valid", "contacts", "model_clearance", "model_pair", "plan", "planning_time"]
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in keys})

    feasible = [r for r in rows if r["ik"]]
    valid = [r for r in feasible if r["moveit_valid"]]
    planned = [r for r in valid if r.get("plan") == "SUCCESS"]
    times = sorted(r["planning_time"] for r in planned)
    p95 = times[int(np.ceil(0.95 * len(times))) - 1] if times else float("inf")
    disagree = [r for r in feasible if (r["model_clearance"] >= 0) != bool(r["moveit_valid"])
                and abs(r["model_clearance"]) >= BORDERLINE]
    borderline = [r for r in feasible if (r["model_clearance"] >= 0) != bool(r["moveit_valid"])
                  and abs(r["model_clearance"]) < BORDERLINE]
    blocked = [r for r in feasible if not r["moveit_valid"]]

    print(f"\ngrid {len(rows)} points, IK-feasible {len(feasible)}, goal in collision {len(blocked)}, "
          f"collision-free goals {len(valid)}, planned {len(planned)}")
    if blocked:
        print("goals in collision (not reachable by any planner):")
        for r in blocked:
            print(f"    x {r['x']:.3f} y {r['y']:+.2f} h {r['h']:.2f}  {r['contacts']}")
    ok = []

    def report(name, cond, detail):
        ok.append(cond)
        print(f"{'PASS' if cond else 'FAIL'}  {name}  -> {detail}", flush=True)
    rate = len(planned) / max(1, len(valid))
    report(">= 95 % of IK-feasible collision-free goals plan", rate >= 0.95,
           f"{len(planned)}/{len(valid)} = {100 * rate:.1f} %")
    report("p95 planning time < 1.0 s", p95 < 1.0, f"p95 {p95:.3f} s, max {times[-1] if times else 0:.3f} s")
    report("MoveIt collision verdicts match the geometry model", not disagree,
           f"{len(disagree)} disagreements, {len(borderline)} borderline (<{BORDERLINE * 1000:.0f} mm)")
    for r in disagree:
        print(f"      x {r['x']} y {r['y']} h {r['h']}: MoveIt valid={r['moveit_valid']} ({r['contacts']}), "
              f"model clearance {r['model_clearance']} ({r['model_pair']})")
    # where pick (P9: base x 0.40 +- 0.02, y +- 0.03) and place (P10: x 0.45) happen
    centre = [r for r in feasible if abs(r["y"]) <= 0.05 and 0.375 <= r["x"] <= 0.45]
    report("mission band (x 0.375..0.45, |y| <= 0.05, all heights) fully plannable",
           all(r.get("plan") == "SUCCESS" for r in centre), f"{sum(r.get('plan') == 'SUCCESS' for r in centre)}/{len(centre)}")
    print(f"wrote {out}")
    fails = ok.count(False)
    print(f"\nP07 reach sweep: {len(ok) - fails}/{len(ok)} PASS", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
