#!/usr/bin/env python3
"""Phase-3 evaluator / world check (run while mission_sim.launch.xml is up with objects).

  - /evaluator/summary reports robot, target and distractor
  - the ground truth is world-frame: robot at home (-3, -2), target at T1, distractor at (0.5, 0.8)
  - the target stands upright and stays still (static scene, nothing touching it)
  - isolation: only the evaluator (and this gate node) subscribe to /evaluator/* topics
Exit 0 = all PASS.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

EXPECT = {"robot": (-3.0, -2.0, 0.05), "target": (3.3, 2.4, 0.02), "distractor": (0.5, 0.8, 0.02)}
GT_TOPICS = ["/evaluator/robot_odom", "/evaluator/target_odom", "/evaluator/distractor_odom", "/evaluator/summary"]
ALLOWED_SUBSCRIBERS = {"evaluator", "p03_eval_check"}
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


class Check(Node):
    def __init__(self):
        super().__init__("p03_eval_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.last = None
        self.create_subscription(String, "/evaluator/summary", self._cb, 10)

    def _cb(self, msg):
        self.last = json.loads(msg.data)


def main():
    rclpy.init()
    c = Check()
    w0 = time.time()
    while time.time() - w0 < 60:
        rclpy.spin_once(c, timeout_sec=0.1)
        if c.last and all(c.last.get(k) for k in EXPECT):
            break
    s = c.last or {}
    for k in EXPECT:
        report(f"evaluator reports {k}", bool(s.get(k)), "" if s.get(k) else "null: model not spawned or not bridged")
    if not all(s.get(k) for k in EXPECT):
        sys.exit(1)

    for k, (ex, ey, tol) in EXPECT.items():
        x, y = s[k][0], s[k][1]
        err = math.hypot(x - ex, y - ey)
        hint = ""
        if k == "robot" and math.hypot(x, y) < 0.1:
            hint = "  (reads ~origin: OdometryPublisher is relative to start, evaluator needs a spawn offset)"
        report(f"{k} ground truth at ({ex}, {ey}) +/- {tol} m (world frame)", err <= tol,
               f"({x:.3f}, {y:.3f}), err {err:.3f} m{hint}")

    first = s["target"]
    w1 = time.time()
    while time.time() - w1 < 5.0:
        rclpy.spin_once(c, timeout_sec=0.1)
    moved = math.hypot(c.last["target"][0] - first[0], c.last["target"][1] - first[1])
    report("target stays still over 5 s", moved < 0.005, f"moved {moved * 1000:.1f} mm")
    report("target upright (model origin on the ground)", abs(c.last["target"][2]) < 0.01,
           f"z {c.last['target'][2]:.3f} m")

    for topic in GT_TOPICS:
        subs = {i.node_name for i in c.get_subscriptions_info_by_topic(topic)}
        extra = subs - ALLOWED_SUBSCRIBERS
        report(f"isolation: {topic} subscribers within {sorted(ALLOWED_SUBSCRIBERS)}", not extra,
               f"unexpected: {sorted(extra)}" if extra else f"{sorted(subs)}")

    n_fail = results.count(False)
    print(f"\nP03 evaluator checks: {len(results) - n_fail}/{len(results)} PASS", flush=True)
    c.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
