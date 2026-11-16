#!/usr/bin/env python3
"""Phase-5 gate checks: arm + gripper controllers, startup detach, stall grasp.
Run while my_robot_gazebo.launch.xml (test_world, robot at origin, with_arm) is up.
GATE TOOLING: spawns the target and reads /evaluator/target_odom (allowed for gates).

Test poses were computed with the SPEC §11.1 analytic IK and checked through the URDF chain
(tool0 error 0.0 mm, approach axis straight down):
  GRASP    (0, 0.9696, 1.6527, 0.5192): tool0 at base (0.40, 0, 0.125 above ground) = cylinder
           top - 0.025 (DC-20), fingertips 0.095 above ground
  PREGRASP (0, 0.6858, 1.6527, 0.8030): same, 0.10 higher
Exit 0 = all PASS.
"""
import math
import os
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint"]
FINGERS = ["left_finger_joint", "right_finger_joint"]
ZERO = [0.0, 0.0, 0.0, 0.0]
GRASP = [0.0, 0.9696, 1.6527, 0.5192]
PREGRASP = [0.0, 0.6858, 1.6527, 0.8030]
CONFIGS = [("cfg_a", [0.6, 0.4, 0.6, 0.4]), ("cfg_b", [-0.7, 0.3, 1.1, -0.6]),
           ("cfg_c", [1.2, -0.5, 1.6, 0.8]), ("zero", ZERO), ("horizontal", [0.0, 1.5708, 0.0, 0.0])]
TARGET_XY = (0.40, 0.0)
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


class P05(Node):
    def __init__(self):
        super().__init__("p05_checks", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.js = {}
        self.imu = None
        self.target = None
        self.attached = None
        self.attached_history = []
        self.contact_t = {"left": -1.0, "right": -1.0}
        self.create_subscription(JointState, "/joint_states", self._js, 20)
        self.create_subscription(Imu, "/imu", lambda m: setattr(self, "imu", m), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/evaluator/target_odom", self._target, qos_profile_sensor_data)
        self.create_subscription(Bool, "/gripper/attached", self._att,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for side in ("left", "right"):
            self.create_subscription(Contacts, f"/gripper/{side}_contact",
                                     lambda m, s=side: self._contact(s, m), qos_profile_sensor_data)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)

    # ---------- callbacks ----------
    def _js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def _target(self, m):
        p = m.pose.pose.position
        self.target = (p.x, p.y, p.z)

    def _att(self, m):
        self.attached = m.data
        self.attached_history.append((round(self.now(), 2), m.data))

    def _contact(self, side, m):
        for c in m.contacts:
            if "target_cylinder" in c.collision1.name or "target_cylinder" in c.collision2.name:
                self.contact_t[side] = self.now()
                return

    # ---------- helpers ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, s, wall=60.0):
        t0, w0 = self.now(), time.time()
        while self.now() - t0 < s and time.time() - w0 < wall:
            rclpy.spin_once(self, timeout_sec=0.02)

    def move(self, client, joints, pos, dur):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        pt.time_from_start = Duration(seconds=dur).to_msg()
        goal.trajectory.points = [pt]
        f = client.send_goal_async(goal)
        w0 = time.time()
        while not f.done() and time.time() - w0 < 10:
            rclpy.spin_once(self, timeout_sec=0.02)
        gh = f.result() if f.done() else None
        if gh is None or not gh.accepted:
            return "REJECTED", None
        rf = gh.get_result_async()
        w0 = time.time()
        while not rf.done() and time.time() - w0 < dur * 4 + 20:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not rf.done():
            return "TIMEOUT", None
        st = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED"}.get(
            rf.result().status, str(rf.result().status))
        return st, rf.result().result.error_code

    def q(self, joints):
        return [self.js.get(j, float("nan")) for j in joints]


def main():
    rclpy.init()
    n = P05()
    w0 = time.time()
    while time.time() - w0 < 60:
        rclpy.spin_once(n, timeout_sec=0.1)
        if all(j in n.js for j in ARM + FINGERS) and n.arm.server_is_ready() and n.grip.server_is_ready():
            break
    report("arm + finger joint states and both trajectory action servers up",
           all(j in n.js for j in ARM + FINGERS) and n.arm.server_is_ready() and n.grip.server_is_ready())

    # ---------- 1. spawn target, verify the startup attach is undone and it stays still ----------
    model = os.path.join(get_package_share_directory("my_robot_bringup"), "models", "target_cylinder", "model.sdf")
    r = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-name", "target_cylinder", "-file", model,
                        "-x", str(TARGET_XY[0]), "-y", str(TARGET_XY[1]), "-z", "0.001"],
                       capture_output=True, text=True, timeout=60)
    report("target_cylinder spawned at base x 0.40", r.returncode == 0, r.stderr[-200:] if r.returncode else "")
    w0 = time.time()
    while n.target is None and time.time() - w0 < 20:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.spin_sim(1.0)
    p0 = n.target
    n.spin_sim(10.0)
    p1 = n.target
    moved = math.dist(p0[:2], p1[:2]) if p0 and p1 else float("nan")
    report("target does not move in the 10 s after spawning (< 5 mm)", moved < 0.005,
           f"{moved * 1000:.1f} mm; final ({p1[0]:.3f}, {p1[1]:.3f}, z {p1[2]:.3f})" if p1 else "no pose")
    saw_attach = any(a for _, a in n.attached_history)
    # Gazebo reports the state only on change, so "never attached" leaves it unknown (None).
    # The 10 s no-motion check above is the physical proof; this checks the reported state.
    report("grasp attach not ATTACHED after spawn (manager undid any startup attach)",
           n.attached is not True,
           f"state {n.attached}; history {n.attached_history} "
           f"({'startup attach seen and undone' if saw_attach else 'no attach event seen'})")

    # ---------- 2. zero config: tool0 position ----------
    st, _ = n.move(n.arm, ARM, ZERO, 2.0)
    n.spin_sim(1.0)
    try:
        t = n.tf.lookup_transform("base_link", "tool0", Time()).transform.translation
        err = math.dist((t.x, t.y, t.z), (0.05, 0.0, 0.84))
        report("tool0 at zero config = base_link (0.05, 0, 0.84) +/- 0.002", err <= 0.002,
               f"({t.x:.4f}, {t.y:.4f}, {t.z:.4f}), err {err * 1000:.1f} mm")
    except Exception as e:  # noqa: BLE001
        report("tool0 at zero config", False, f"TF: {e}")

    # ---------- 3. gripper open/close with nothing between the fingers ----------
    st_c, _ = n.move(n.grip, FINGERS, [0.0, 0.0], 1.5)
    n.spin_sim(0.5)
    qc = n.q(FINGERS)
    st_o, _ = n.move(n.grip, FINGERS, [0.04, 0.04], 1.5)
    n.spin_sim(0.5)
    qo = n.q(FINGERS)
    report("gripper closes empty to 0.000 +/- 0.002", st_c == "SUCCEEDED" and all(abs(v) <= 0.002 for v in qc),
           f"{st_c}, q {[round(v, 4) for v in qc]}")
    report("gripper opens to 0.040 +/- 0.002", st_o == "SUCCEEDED" and all(abs(v - 0.04) <= 0.002 for v in qo),
           f"{st_o}, q {[round(v, 4) for v in qo]}")

    # ---------- 4. arm configurations ----------
    worst = 0.0
    bad = []
    for name, pos in CONFIGS:
        st, code = n.move(n.arm, ARM, pos, 3.0)
        n.spin_sim(0.5)
        err = max(abs(a - b) for a, b in zip(n.q(ARM), pos))
        worst = max(worst, err)
        print(f"      {name:<10s} {st:<9s} max joint err {err:.4f} rad", flush=True)
        if st != "SUCCEEDED" or err >= 0.02:
            bad.append(name)
    report("5 arm configurations reached, every joint within 0.02 rad", not bad,
           f"worst {worst:.4f} rad" + (f"; failed {bad}" if bad else ""))

    # ---------- 5. no tipping with the arm horizontal (last config) ----------
    n.spin_sim(2.0)
    roll, pitch = rpy(n.imu.orientation) if n.imu else (9, 9)
    report("arm horizontal: base roll/pitch < 0.05 rad (no tipping)", abs(roll) < 0.05 and abs(pitch) < 0.05,
           f"roll {roll:+.4f}, pitch {pitch:+.4f} rad")

    # ---------- 6. stall grasp on the target ----------
    n.move(n.arm, ARM, ZERO, 2.5)
    n.move(n.grip, FINGERS, [0.04, 0.04], 1.0)
    before = n.target
    st_pg, _ = n.move(n.arm, ARM, PREGRASP, 3.0)
    st_g, _ = n.move(n.arm, ARM, GRASP, 2.0)
    n.spin_sim(0.5)
    t_close = n.now()
    st_close, code = n.move(n.grip, FINGERS, [0.0, 0.0], 1.5)
    n.spin_sim(1.0)
    qs = n.q(FINGERS)
    report("arm reached pregrasp and grasp poses", st_pg == "SUCCEEDED" and st_g == "SUCCEEDED", f"{st_pg}, {st_g}")
    report("fingers stall on the cylinder at q in [0.020, 0.030]", all(0.020 <= v <= 0.030 for v in qs),
           f"q {[round(v, 4) for v in qs]} (cylinder radius 0.025)")
    report("gripper goal not aborted while stalled", st_close == "SUCCEEDED", f"{st_close}, error_code {code}")
    got = {s: n.contact_t[s] >= t_close for s in ("left", "right")}
    report("both finger contact sensors report contact with target_cylinder", all(got.values()), str(got))
    disp = math.dist(before[:2], n.target[:2]) if before and n.target else float("nan")
    report("target not knocked over by the approach (moved < 0.02 m, still upright)",
           disp < 0.02 and abs(n.target[2]) < 0.01, f"moved {disp * 1000:.1f} mm, z {n.target[2]:.3f}")
    report("grasp attach still not ATTACHED (only Phase 9 attaches, after verification)", n.attached is not True,
           str(n.attached))

    # ---------- release and retract ----------
    n.move(n.grip, FINGERS, [0.04, 0.04], 1.0)
    n.move(n.arm, ARM, PREGRASP, 2.0)
    n.move(n.arm, ARM, ZERO, 3.0)

    n_fail = results.count(False)
    print(f"\nP05 checks: {len(results) - n_fail}/{len(results)} PASS", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
