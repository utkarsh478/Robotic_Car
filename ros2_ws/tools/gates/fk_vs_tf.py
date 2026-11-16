#!/usr/bin/env python3
"""Phase-6 gate: does OUR C++ kinematics (ik_cli -> libarm_ik) agree with the URDF as Gazebo
and robot_state_publisher see it? Catches any mismatch between SPEC/IK constants and the model.

For 20 targets in front of the robot (16 top-down, 4 tilted approaches):
  1. ik_cli ik  -> joint solution
  2. drive the real arm there (JTC), via a high "ready" pose so the gripper never sweeps low
  3. ik_cli fk on the MEASURED joints  vs  TF base_link -> tool0     (position and approach axis)
PASS: max position error < 1 mm, max approach-axis error < 0.01 rad (C++ FK vs TF);
      tracking (TF vs commanded target) < 5 mm is reported too.
Run while my_robot_gazebo.launch.xml (test_world, arm on) is up. Exit 0 = all PASS.
"""
import math
import random
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint"]
ARM_BASE = (0.05, 0.0, 0.11)          # arm_base_link in base_link
ARM_BASE_ABOVE_GROUND = 0.075 + 0.11
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def ik_cli(*args):
    r = subprocess.run(["ros2", "run", "my_robot_manipulation", "ik_cli", *[f"{a:.9f}" if isinstance(a, float) else a
                                                                           for a in args]],
                       capture_output=True, text=True, timeout=30)
    return r.returncode, r.stdout.split()


def quat_to_z_axis(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


class Check(Node):
    def __init__(self):
        super().__init__("fk_vs_tf", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)

    def _js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, s):
        t0, w0 = self.now(), time.time()
        while self.now() - t0 < s and time.time() - w0 < 60:
            rclpy.spin_once(self, timeout_sec=0.02)

    def move(self, q, dur):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q]
        pt.time_from_start = Duration(seconds=dur).to_msg()
        goal.trajectory.points = [pt]
        f = self.arm.send_goal_async(goal)
        w0 = time.time()
        while not f.done() and time.time() - w0 < 10:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not f.done() or not f.result().accepted:
            return False
        rf = f.result().get_result_async()
        w0 = time.time()
        while not rf.done() and time.time() - w0 < dur * 4 + 20:
            rclpy.spin_once(self, timeout_sec=0.02)
        return rf.done() and rf.result().status == GoalStatus.STATUS_SUCCEEDED


def main():
    rclpy.init()
    n = Check()
    w0 = time.time()
    while time.time() - w0 < 60 and not (n.arm.server_is_ready() and all(j in n.js for j in ARM)):
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.arm.server_is_ready():
        print("FAIL  /arm_controller/follow_joint_trajectory not available")
        sys.exit(1)

    rc, out = ik_cli("ik", 0.30, 0.0, 0.42 - ARM_BASE_ABOVE_GROUND)
    if rc != 0:
        print(f"FAIL  ik_cli not working: {out}")
        sys.exit(1)
    ready = [float(v) for v in out[1:5]]

    rng = random.Random(6)
    targets = []
    while len(targets) < 20:
        x = rng.uniform(0.30, 0.50) - ARM_BASE[0]                 # base_link x 0.30..0.50
        y = rng.uniform(-0.20, 0.20)
        h = rng.uniform(0.13, 0.45)                               # tool0 above ground
        pitch = math.pi if len(targets) < 16 else rng.uniform(2.2, 2.8)
        rc, out = ik_cli("ik", x, y, h - ARM_BASE_ABOVE_GROUND, pitch)
        if rc == 0:
            targets.append(((x, y, h - ARM_BASE_ABOVE_GROUND, pitch), [float(v) for v in out[1:5]], " ".join(out[5:])))

    worst_p = worst_a = worst_track = 0.0
    failed_moves = 0
    for i, ((x, y, z, pitch), q_cmd, branch) in enumerate(targets, 1):
        ok = n.move(ready, 2.0) and n.move(q_cmd, 2.5)
        n.spin_sim(0.5)
        if not ok:
            failed_moves += 1
            print(f"[{i:2d}] move FAILED", flush=True)
            continue
        q_m = [n.js[j] for j in ARM]
        _, fk = ik_cli("fk", *q_m)
        fx, fy, fz, fpitch = (float(v) for v in fk[:4])
        p_fk = (fx + ARM_BASE[0], fy + ARM_BASE[1], fz + ARM_BASE[2])
        tr = n.tf.lookup_transform("base_link", "tool0", Time()).transform
        p_tf = (tr.translation.x, tr.translation.y, tr.translation.z)
        d_fk = (math.sin(fpitch) * math.cos(q_m[0]), math.sin(fpitch) * math.sin(q_m[0]), math.cos(fpitch))
        d_tf = quat_to_z_axis(tr.rotation)
        e_p = math.dist(p_fk, p_tf)
        e_a = math.acos(max(-1.0, min(1.0, sum(a * b for a, b in zip(d_fk, d_tf)))))
        e_track = math.dist(p_tf, (x + ARM_BASE[0], y + ARM_BASE[1], z + ARM_BASE[2]))
        worst_p, worst_a, worst_track = max(worst_p, e_p), max(worst_a, e_a), max(worst_track, e_track)
        print(f"[{i:2d}] {branch:<18s} pitch {pitch:.2f}  FK-vs-TF {e_p * 1000:6.3f} mm / {e_a:.5f} rad  "
              f"tracking {e_track * 1000:5.2f} mm", flush=True)
    n.move(ready, 2.0)
    n.move([0.0, 0.0, 0.0, 0.0], 2.5)

    report("all 20 IK solutions executed", failed_moves == 0, f"{failed_moves} failed")
    report("C++ FK vs TF position < 1 mm (20 poses)", worst_p < 0.001, f"max {worst_p * 1000:.3f} mm")
    report("C++ FK vs TF approach axis < 0.01 rad", worst_a < 0.01, f"max {worst_a:.5f} rad")
    report("arm tracking: TF vs IK target < 5 mm", worst_track < 0.005, f"max {worst_track * 1000:.2f} mm")
    n_fail = results.count(False)
    print(f"\nP06 FK vs TF: {len(results) - n_fail}/{len(results)} PASS", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
