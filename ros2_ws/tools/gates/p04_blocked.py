#!/usr/bin/env python3
"""Phase-4 gate: blocked path. Run after nav_runner.py (robot at home), nav_sim still up.

The only way from the home zone to the middle zone is the top gap of W1 (x = -1.0,
y 1.5..3.0, 1.5 m wide). Two 0.3 x 0.3 blocks are spawned in it at y = 1.95 and 2.55,
leaving three 0.3 m slots, narrower than the robot (0.54 m). They are NOT in the saved
map: Nav2 plans through the gap, must discover the blocks with the lidar, and then has
no valid route. Expected: Nav2 aborts (or the gate cancels at 150 s) without touching
anything, and the robot stops.

PASS: goal not SUCCEEDED; zero contacts (world + blocks); stopped 1 s after the end;
      the robot never passed the gap (x stays < -0.9).
"""
import json
import os
import subprocess
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_common import WS, NavMonitor, box_points  # noqa: E402

BLOCKS = [("block_a", -1.0, 1.95), ("block_b", -1.0, 2.55)]
GOAL = (0.4, 1.6, 0.0)          # middle zone, only reachable through the top gap
OUT = os.path.join(WS, "results", "p04_blocked.json")
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def main():
    model = os.path.join(get_package_share_directory("my_robot_bringup"), "models", "test_block", "model.sdf")
    for name, x, y in BLOCKS:
        r = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-name", name, "-file", model,
                            "-x", str(x), "-y", str(y), "-z", "0.0"], capture_output=True, text=True, timeout=60)
        print(f"spawned {name} at ({x}, {y}): {'ok' if r.returncode == 0 else 'FAILED ' + r.stderr[-200:]}", flush=True)

    rclpy.init()
    extra = [p for _, x, y in BLOCKS for p in box_points(x, y, 0.3, 0.3)]
    m = NavMonitor("p04_blocked", extra_obstacle_points=extra)
    if m.wait_ready() is None:
        print("FAIL  nav not ready")
        sys.exit(1)
    max_x = [-9.0]

    def track_x():
        if m.gt:
            max_x[0] = max(max_x[0], m.gt[0])
    m.create_timer(0.2, track_x)

    print(f"goal ({GOAL[0]}, {GOAL[1]}) behind the blocked gap; waiting up to 150 s ...", flush=True)
    r = m.go(*GOAL, timeout=150.0)
    print(f"result: {r['status']} after {r['time']} s, robot at {r['final_truth']}", flush=True)

    report("blocked goal not reported as SUCCEEDED", r["status"] != "SUCCEEDED", r["status"])
    # Informational: SPEC accepts either Nav2 aborting by itself or the gate cancelling at 150 s.
    print(f"INFO  {'Nav2 aborted by itself' if r['status'] == 'ABORTED' else 'gate cancelled the goal'}"
          f" after {r['time']} s", flush=True)
    report("zero contacts (walls, obstacles, blocks)", not m.contacts,
           f"{len(m.contacts)} samples, first {m.contacts[0]}" if m.contacts else "")
    v, w = r["speed_1s_after"]
    report("stopped 1 s after the end", abs(v) < 0.02 and abs(w) < 0.05, f"v {v:+.3f}, w {w:+.3f}")
    report("never passed the gap (max x < -0.9)", max_x[0] < -0.9, f"max x {max_x[0]:.2f}")

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({"result": r, "max_x": max_x[0], "contacts": m.contacts}, open(OUT, "w"), indent=2)
    n_fail = results.count(False)
    print(f"\nP04 blocked path: {len(results) - n_fail}/{len(results)} PASS", flush=True)
    m.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
