#!/usr/bin/env python3
"""Phase-10 gate: /place_object trials at the home box, the way the mission will use it.

Each trial (robot in the home zone facing the box at (-3.0, -0.9), +y):
  1. reset (detach, park the target, open, stow); robot teleported so the box is at base-x 0.95
     (lateral offset yb, heading offset dyaw); target placed at base-x 0.55 in front of it
     (0.55: the whole cylinder is in view; at 0.40 the camera sees only its top edge)
  2. pick it exactly as in P9: estimate at 0.55, creep to 0.40, /pick_object   [set-up for the place]
  3. box estimate from box_detector (frame odom, >= 10 observations)        [not ground truth]
  4. creep on odometry until the box centre is at base-x 0.48 (DC-38: 0.45 would put the box
     wall inside the lidar puck)
  5. PlaceObject(box centre, z = floor 0.01 + 0.075 + 0.02, box yaw)
Checks (PHASES P10):
  - >= 9/10 placements: result success AND ground truth: target inside the box (centre within
    the inner bounds minus 0.01), standing (tilt < 10 deg), bottom below the rim
  - after every place: arm at stow, fingers open, Gazebo attach off
The box is static in Gazebo, so "box displacement" cannot be measured; arm-vs-box safety comes
from MoveIt (every pose/waypoint collision-checked with the box walls in the scene).
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from my_robot_interfaces.action import PlaceObject
from my_robot_interfaces.msg import Detection
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

import pick_trials as pt

BOX = (-3.0, -0.9)              # home_box floor centre, mission_world.sdf (static, yaw 0)
INNER_HALF = 0.14               # 0.30 outer, 0.01 walls
RIM = 0.15
PLACE_X = 0.48
ERR = {getattr(PlaceObject.Result, k): k for k in dir(type(PlaceObject.Result)) if k.isupper()}
# (box lateral offset, target lateral offset, heading offset) per trial
TRIALS = [(0.0, 0.0, 0.0), (0.02, -0.01, 0.03), (-0.02, 0.01, -0.03), (0.03, 0.0, 0.05), (-0.03, 0.02, -0.05),
          (0.01, -0.02, 0.02), (-0.01, 0.0, -0.02), (0.025, 0.01, 0.04), (-0.025, -0.01, -0.04), (0.0, 0.015, 0.0)]


class PlaceTrials(pt.Trials):
    def __init__(self):
        super().__init__()
        self.place = ActionClient(self, PlaceObject, "/place_object")
        self.box_track = None
        self.create_subscription(Detection, "/perception/box", self._box, 20)
        self.box_reset = self.create_client(Trigger, "/box_detector/reset")

    def _box(self, m):
        self.box_track = m

    def box_estimate(self, timeout=15.0):
        self.call(self.box_reset, Trigger.Request())
        self.box_track = None
        t0 = self.now()
        while self.now() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.box_track is not None and self.box_track.observation_count >= 10:
                return self.box_track
        return None

    def reset_only(self):
        self.call(self.attach_cli, pt.SetBool.Request(data=False))
        pt.teleport("target_cylinder", *pt.PARK_TARGET, 0.0, 0.001)
        self.jtc(self.grip, pt.FINGERS, [0.04, 0.04], 1.5)
        self.jtc(self.arm, pt.ARM, pt.srdf_state("stow"), 3.0)
        self.spin_sim(0.5)

    def run_place(self, box):
        goal = PlaceObject.Goal()
        goal.place_pose = PoseStamped()
        goal.place_pose.header.frame_id = box.header.frame_id
        goal.place_pose.pose.position.x = box.pose.position.x
        goal.place_pose.pose.position.y = box.pose.position.y
        goal.place_pose.pose.position.z = 0.01 + 0.075 + 0.02
        goal.place_pose.pose.orientation = box.pose.orientation
        stages = []

        def fb(m):
            stages.append(m.feedback.stage)
            self.cur_stage = m.feedback.stage
        if not self.place.wait_for_server(timeout_sec=10):
            return None, stages, 0.0
        t0 = time.time()
        h = self.wait_future(self.place.send_goal_async(goal, feedback_callback=fb), 10)
        if not h or not h.accepted:
            return None, stages, 0.0
        r = self.wait_future(h.get_result_async(), 150)
        return (r.result if r else None), stages, time.time() - t0


def main():
    rclpy.init()
    n = PlaceTrials()
    w0 = time.time()
    while time.time() - w0 < 120 and not (n.pick.server_is_ready() and n.place.server_is_ready()
                                          and {"robot", "target"} <= set(n.gt) and all(j in n.js for j in pt.ARM)):
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.place.server_is_ready():
        print("FAIL  /place_object not available (is pick_place_server running?)")
        sys.exit(1)
    print("PASS  /pick_object, /place_object, ground truth and joint states available", flush=True)
    pt.teleport("distractor_cylinder", *pt.PARK_DISTRACTOR, 0.0, 0.001)
    stow = pt.srdf_state("stow")

    placed, after_ok, setups, times = 0, 0, 0, []
    for i, (yb, yt, dyaw) in enumerate(TRIALS):
        label = f"place {i + 1:2d} (box y {yb:+.3f}, target y {yt:+.3f}, yaw {dyaw:+.2f})"
        n.reset_only()
        yaw = math.pi / 2 + dyaw
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = BOX[0] - (c * 0.95 - s * yb), BOX[1] - (s * 0.95 + c * yb)
        pt.teleport("my_robot", rx, ry, yaw)
        n.spin_sim(1.0)
        tx, ty = rx + c * 0.55 - s * yt, ry + s * 0.55 + c * yt
        pt.teleport("target_cylinder", tx, ty, 0.0, 0.001)
        n.spin_sim(1.0)
        est, _ = n.estimate()
        if est is None:
            print(f"      {label}: setup failed: no target estimate", flush=True)
            n.diagnose_view(f"place_{i + 1}")
            continue
        if n.creep(est, 0.40) is None:
            print(f"      {label}: setup failed: no TF during the creep to the target", flush=True)
            continue
        res, _ = n.run_pick(est)
        if not (res and res.success and n.attached):
            print(f"      {label}: setup failed: pick {pt.ERR.get(res.error_code, '?') if res else 'no result'} "
                  f"'{res.message if res else ''}'", flush=True)
            continue
        box = n.box_estimate()
        if box is None:
            print(f"      {label}: setup failed: no box estimate (>= 10 observations)", flush=True)
            continue
        final = n.creep(box, PLACE_X)
        if final is None:
            print(f"      {label}: setup failed: no TF during the creep", flush=True)
            continue
        setups += 1
        r = n.gt["robot"]
        dx, dy = BOX[0] - r[0], BOX[1] - r[1]
        truth = (math.cos(r[3]) * dx + math.sin(r[3]) * dy, -math.sin(r[3]) * dx + math.cos(r[3]) * dy)
        print(f"      {label}: box estimate after creep base ({final[0]:.3f}, {final[1]:+.3f}), truth "
              f"({truth[0]:.3f}, {truth[1]:+.3f}), error {math.hypot(final[0] - truth[0], final[1] - truth[1]) * 1000:.1f} mm",
              flush=True)
        n.trace, n.cur_stage = [], "start"
        res, stages, wall = n.run_place(box)
        n.spin_sim(1.5)
        t = n.gt["target"]
        off = (t[0] - BOX[0], t[1] - BOX[1])
        inside = abs(off[0]) <= INNER_HALF - 0.01 and abs(off[1]) <= INNER_HALF - 0.01
        upright = n.target_tilt < 10.0
        below_rim = t[2] < RIM
        good = bool(res and res.success and inside and upright and below_rim)
        placed += good
        if res and res.success:
            times.append(wall)
        stowed = max(abs(n.js[a] - q) for a, q in zip(pt.ARM, stow)) < 0.05
        open_ = min(n.js.get(f, 0.0) for f in pt.FINGERS) >= 0.035
        after = stowed and open_ and n.attached is False
        after_ok += after
        code = ERR.get(res.error_code, "?") if res else "no result"
        print(f"      {label}: {'OK  ' if good else 'FAIL'} {code} '{res.message if res else ''}'", flush=True)
        print(f"         truth: target {off[0] * 1000:+.0f}/{off[1] * 1000:+.0f} mm from the box centre, tilt "
              f"{n.target_tilt:.1f} deg, bottom z {t[2]:.3f}; after: stowed {stowed}, open {open_}, "
              f"attached {n.attached}, {wall:.1f} s", flush=True)
        if not good:
            n.print_trace(stages)
            if res and ERR.get(res.error_code) == "RELEASE_NOT_CONFIRMED":
                n.diagnose_view(f"place_{i + 1}")      # what the camera sees now (arm stowed)
        n.trace = None

    pt.report(f">= 9/10 placements inside the box, standing (set-ups completed: {setups}/10)", placed >= 9,
              f"{placed}/10")
    pt.report("after every place: arm stowed, fingers open, attach off", after_ok == setups and setups > 0,
              f"{after_ok}/{setups}")
    if times:
        print(f"      place time: mean {sum(times) / len(times):.1f} s, max {max(times):.1f} s", flush=True)
    n.reset_only()
    fails = pt.results.count(False)
    print(f"\nP10 place trials: {len(pt.results) - fails}/{len(pt.results)} PASS", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
