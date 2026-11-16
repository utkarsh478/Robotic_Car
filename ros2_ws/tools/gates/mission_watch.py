#!/usr/bin/env python3
"""P11 gate: watch one full mission and score it against ground truth.

GATE TOOLING ONLY (CLAUDE.md rule 4): reads /evaluator/{robot,target}_odom; the mission never does.
Listens to /mission/state and /mission/result (latched) until the mission ends or --timeout
(wall seconds) passes. Success needs ALL of:
  - /mission/result success (MISSION_COMPLETE)
  - target inside the home box: centre within the inner bounds minus 0.01, bottom below the rim,
    standing (tilt < 10 deg)                                       (SPEC §13 placement success)
  - zero chassis contacts with the world (nav_common: robot body vs obstacle boundaries, 2 Hz)
Prints the state timeline and a final line  RESULT {json}.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nav_common import NavMonitor  # noqa: E402

BOX = (-3.0, -0.9)
INNER_HALF = 0.14
RIM = 0.15
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def tilt_deg(q):
    # angle between the body z axis and world z
    zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


class Watch(NavMonitor):
    def __init__(self):
        super().__init__("p11_mission_watch")
        self.states = []          # (sim t, wall t, state)
        self.result = None
        self.target = None
        self.create_subscription(String, "/mission/state", self._state, LATCHED)
        self.create_subscription(String, "/mission/result", self._result, LATCHED)
        self.create_subscription(Odometry, "/evaluator/target_odom", self._target, qos_profile_sensor_data)

    def _state(self, m):
        if not self.states or self.states[-1][2] != m.data:
            self.states.append((self.now(), time.time(), m.data))
            print(f"      [sim {self.now():7.1f} s] {m.data}", flush=True)

    def _result(self, m):
        self.result = json.loads(m.data)

    def _target(self, m):
        p = m.pose.pose
        self.target = (p.position.x, p.position.y, p.position.z, tilt_deg(p.orientation))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trial", default="p11")
    ap.add_argument("--timeout", type=float, default=1800.0, help="wall seconds")
    a = ap.parse_args()
    rclpy.init()
    n = Watch()
    w0 = time.time()
    while time.time() - w0 < a.timeout and n.result is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.result is not None:
        n.spin_sim(2.0, wall_limit=10.0)                 # let the object settle
    wall = time.time() - w0
    t = n.target
    off = None if t is None else (t[0] - BOX[0], t[1] - BOX[1])
    inside = t is not None and abs(off[0]) <= INNER_HALF - 0.01 and abs(off[1]) <= INNER_HALF - 0.01
    below = t is not None and t[2] < RIM
    upright = t is not None and t[3] < 10.0
    res = n.result or {}
    ok = bool(res.get("success")) and inside and below and upright and not n.contacts
    out = {
        "trial": a.trial, "success": ok, "mission_success": res.get("success"), "reason": res.get("reason"),
        "final_state": n.states[-1][2] if n.states else None,
        "mission_time_sim": res.get("mission_time"), "wall_time": round(wall, 1),
        "target_offset_mm": None if off is None else [round(off[0] * 1000), round(off[1] * 1000)],
        "target_bottom_z": None if t is None else round(t[2], 3), "target_tilt_deg": None if t is None else round(t[3], 1),
        "inside_box": inside, "below_rim": below, "upright": upright,
        "chassis_contacts": len(n.contacts), "first_contact": n.contacts[0] if n.contacts else None,
        "states": res.get("states"),
    }
    if n.result is None:
        out["reason"] = f"no /mission/result within {a.timeout:.0f} s wall (last state {out['final_state']})"
    print("RESULT " + json.dumps(out), flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
