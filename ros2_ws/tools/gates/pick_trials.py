#!/usr/bin/env python3
"""Phase-9 gate: /pick_object trials in mission_world, the way the mission will use it.

Each trial (target at E = (0.25, 2.0), robot facing about -y):
  1. reset: detach, park the target, open the gripper, stow the arm, put the target back at E
  2. teleport the robot so the target is at base-x 0.55, lateral offset y0
  3. perception (frame odom) -> target estimate (>= 10 observations)        [not ground truth]
  4. creep straight ahead on odometry until the estimate is at base-x x_goal  (mission FINAL_ALIGN)
  5. PickObject(estimate in odom, 0.05, 0.15)
Checks (PHASES P9):
  - >= 9/10 picks succeed: result success, ground-truth target lifted > 5 cm, Gazebo attach on
  - missed grasp, 3/3: target teleported 0.2 m away when the descent starts ->
    GRASP_NOT_DETECTED, never attached, fingers open again, arm back at stow
  - hold, 3/3: after a pick, drive 2 m and turn 90 deg; the target moves < 0.02 m relative to tool0
  - the arm never knocks the target before the fingers close (ground truth moves < 5 mm)
Ground truth (/evaluator/*) is used only to place things and to score.
"""
import math
import os
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Twist
from my_robot_interfaces.action import PickObject
from my_robot_interfaces.msg import Detection
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
WORLD = "mission_world"
E = (0.25, 2.0)
PARK_TARGET = (1.2, -2.4)
PARK_DISTRACTOR = (-0.5, -2.4)
ARM = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint"]
FINGERS = ["left_finger_joint", "right_finger_joint"]
# rosidl puts message constants on the metaclass as properties
ERR = {getattr(PickObject.Result, k): k for k in dir(type(PickObject.Result)) if k.isupper()}
# (lateral offset y0, creep goal base-x, heading offset) per trial
TRIALS = [(-0.03, 0.40, 0.10), (-0.02, 0.39, -0.05), (-0.01, 0.41, 0.0), (0.0, 0.38, 0.12), (0.01, 0.42, -0.10),
          (0.02, 0.40, 0.05), (0.03, 0.395, -0.12), (-0.025, 0.405, 0.08), (0.015, 0.40, -0.03), (0.0, 0.385, 0.0)]
N_HOLD = 3          # hold test after each of the first 3 successful picks
MISSED = [(0.0, 0.40, 0.0), (0.02, 0.40, 0.05), (-0.02, 0.40, -0.05)]
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def teleport(name, x, y, yaw=0.0, z=0.01):
    req = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    r = subprocess.run(["ign", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "ignition.msgs.Pose",
                        "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", req],
                       capture_output=True, text=True, timeout=15)
    return "true" in r.stdout


def srdf_state(name):
    txt = open(os.path.join(WS, "src", "my_robot_moveit_config", "config", "my_robot.srdf")).read()
    body = re.search(rf'<group_state name="{name}" group="arm">(.*?)</group_state>', txt, re.S).group(1)
    vals = dict(re.findall(r'<joint name="([^"]+)" value="([^"]+)"', body))
    return [float(vals[j]) for j in ARM]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Trials(Node):
    def __init__(self):
        super().__init__("pick_trials", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.gt = {}
        for k in ("robot", "target"):
            self.create_subscription(Odometry, f"/evaluator/{k}_odom", lambda m, k=k: self._gt(k, m), 20)
        self.js = {}
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.track = None
        self.create_subscription(Detection, "/perception/target", self._det, 20)
        self.n_raw = 0
        self.create_subscription(Detection, "/perception/target_raw", self._raw, 50)
        self.attached = None
        self.attached_ever = False
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/gripper/attached", self._att, latched)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.pick = ActionClient(self, PickObject, "/pick_object")
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.attach_cli = self.create_client(SetBool, "/gripper/set_attached")
        self.reset_cli = self.create_client(Trigger, "/target_detector/reset")
        self.stage_log = []
        self.on_stage = None
        self.target_tilt = 0.0
        self.bridge = CvBridge()
        self.info = None
        self.create_subscription(CameraInfo, "/camera/camera_info", lambda m: setattr(self, "info", m), 5)
        self.attach_events = []
        self.create_subscription(String, "/gripper/attach_state", self._attach_state, 10)
        self.trace = None
        self.cur_stage = ""
        self.create_timer(0.2, self._sample)

    def _attach_state(self, m):
        self.attach_events.append((self.now(), m.data))

    def _sample(self):
        if self.trace is not None and "target" in self.gt:
            t = self.gt["target"]
            self.trace.append((self.now(), self.cur_stage, t[0], t[1], t[2], self.target_tilt,
                               self.js.get(FINGERS[0], -1), self.js.get(FINGERS[1], -1)))

    def _gt(self, k, m):
        p = m.pose.pose
        self.gt[k] = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
        if k == "target":
            q = p.orientation
            # tilt of the cylinder axis from vertical
            self.target_tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))

    def _js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p

    def _det(self, m):
        self.track = m
        self.n_track_msgs = getattr(self, "n_track_msgs", 0) + 1

    def _raw(self, m):
        self.n_raw += 1

    def _att(self, m):
        self.attached = m.data
        if m.data:
            self.attached_ever = True

    # ------------------------------------------------------------- helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, s):
        t0, w0 = self.now(), time.time()
        while self.now() - t0 < s and time.time() - w0 < 4 * s + 10:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_future(self, f, timeout):
        w0 = time.time()
        while not f.done() and time.time() - w0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
        return f.result() if f.done() else None

    def call(self, cli, req, timeout=8.0):
        if not cli.wait_for_service(timeout_sec=5.0):
            return None
        return self.wait_future(cli.call_async(req), timeout)

    def jtc(self, client, joints, q, dur):
        if not client.wait_for_server(timeout_sec=10):
            return False
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = joints
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(seconds=dur).to_msg()
        g.trajectory.points = [pt]
        h = self.wait_future(client.send_goal_async(g), 10)
        if not h or not h.accepted:
            return False
        r = self.wait_future(h.get_result_async(), dur * 4 + 20)
        return bool(r) and r.status == GoalStatus.STATUS_SUCCEEDED

    def base_tf(self, target, source):
        try:
            return self.tf.lookup_transform(target, source, Time())
        except TransformException:
            return None

    def reset_scene(self):
        self.call(self.attach_cli, SetBool.Request(data=False))
        teleport("target_cylinder", *PARK_TARGET, 0.0, 0.001)
        self.jtc(self.grip, FINGERS, [0.04, 0.04], 1.5)
        self.jtc(self.arm, ARM, srdf_state("stow"), 3.0)
        teleport("target_cylinder", *E, 0.0, 0.001)
        self.spin_sim(1.0)

    def place_robot(self, y0, bx, dyaw):
        yaw = -math.pi / 2 + dyaw
        c, s = math.cos(yaw), math.sin(yaw)
        x = E[0] - (c * bx - s * y0)
        y = E[1] - (s * bx + c * y0)
        teleport("my_robot", x, y, yaw)
        self.spin_sim(1.5)
        g = self.gt.get("robot")
        return g is not None and math.hypot(g[0] - x, g[1] - y) < 0.03

    def estimate(self, timeout=15.0):
        """track with >= 10 observations; also returns (messages seen, seconds waited) for the log"""
        self.call(self.reset_cli, Trigger.Request())
        self.track = None
        self.n_track_msgs = 0
        self.n_raw = 0
        t0 = self.now()
        while self.now() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.track is not None and self.track.observation_count >= 10:
                return self.track, self.now() - t0
        return None, self.now() - t0

    def target_in_base(self, est):
        """estimate (odom) -> base_footprint (x, y), using the CURRENT odom->base transform"""
        tr = self.base_tf("odom", "base_footprint")
        if tr is None:
            return None
        bx, by, byaw = tr.transform.translation.x, tr.transform.translation.y, yaw_of(tr.transform.rotation)
        dx, dy = est.pose.position.x - bx, est.pose.position.y - by
        return (math.cos(byaw) * dx + math.sin(byaw) * dy, -math.sin(byaw) * dx + math.cos(byaw) * dy)

    def creep(self, est, x_goal, speed=0.05, timeout=15.0):
        t0 = self.now()
        while self.now() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.01)
            p = self.target_in_base(est)
            if p is None:
                continue
            if p[0] <= x_goal:
                break
            tw = Twist()
            tw.linear.x = min(speed, max(0.01, (p[0] - x_goal) * 1.0))
            self.cmd.publish(tw)
        for _ in range(5):
            self.cmd.publish(Twist())
            self.spin_sim(0.05)
        self.spin_sim(1.0)
        return self.target_in_base(est)

    def drive(self, v, w, seconds):
        t0 = self.now()
        while self.now() - t0 < seconds:
            tw = Twist()
            tw.linear.x, tw.angular.z = v, w
            self.cmd.publish(tw)
            self.spin_sim(0.05)
        for _ in range(5):
            self.cmd.publish(Twist())
            self.spin_sim(0.05)

    def gt_target_in_base(self):
        r, t = self.gt["robot"], self.gt["target"]
        dx, dy = t[0] - r[0], t[1] - r[1]
        return (math.cos(r[3]) * dx + math.sin(r[3]) * dy, -math.sin(r[3]) * dx + math.cos(r[3]) * dy)

    def object_vs_tool0(self):
        """|GT object origin - expected (tool0 + 0.125 m along tool z)| in the world, metres."""
        tr = self.base_tf("base_footprint", "tool0")
        if tr is None or "robot" not in self.gt:
            return None
        rx, ry, _, ryaw = self.gt["robot"]
        q = tr.transform.rotation
        zx = 2 * (q.x * q.z + q.w * q.y)
        zy = 2 * (q.y * q.z - q.w * q.x)
        zz = 1 - 2 * (q.x * q.x + q.y * q.y)
        px = tr.transform.translation.x + 0.125 * zx
        py = tr.transform.translation.y + 0.125 * zy
        pz = tr.transform.translation.z + 0.125 * zz
        wx = rx + math.cos(ryaw) * px - math.sin(ryaw) * py
        wy = ry + math.sin(ryaw) * px + math.cos(ryaw) * py
        t = self.gt["target"]
        return (t[0] - wx, t[1] - wy, t[2] - pz)

    def grab_frame(self, timeout=3.0):
        got = {}
        subs = [self.create_subscription(Image, "/camera/image", lambda m: got.setdefault("rgb", m), 2),
                self.create_subscription(Image, "/camera/depth_image", lambda m: got.setdefault("depth", m), 2)]
        w0 = time.time()
        while time.time() - w0 < timeout and len(got) < 2:
            rclpy.spin_once(self, timeout_sec=0.02)
        for sub in subs:
            self.destroy_subscription(sub)
        return got if len(got) == 2 else None

    def diagnose_view(self, tag):
        """Why did perception see nothing? Ground truth of the target + the detector's own verdict
        on one frame (same pipeline, run here) + the frame saved to /tmp."""
        g = self.gt.get("target")
        b = self.gt_target_in_base()
        print(f"         truth: target world ({g[0]:.3f}, {g[1]:.3f}, z {g[2]:.3f}), tilt {self.target_tilt:.1f} deg, "
              f"base ({b[0]:.3f}, {b[1]:+.3f}); Gazebo attach state {self.attach_events[-1][1] if self.attach_events else '?'}",
              flush=True)
        fr = self.grab_frame()
        if fr is None or self.info is None:
            print("         no camera frame", flush=True)
            return
        try:
            from my_robot_perception import pipeline
            rgb = self.bridge.imgmsg_to_cv2(fr["rgb"], "rgb8")
            depth = self.bridge.imgmsg_to_cv2(fr["depth"], "passthrough").astype(np.float32)
            tr = self.tf.lookup_transform("odom", fr["rgb"].header.frame_id, Time())
            q, t = tr.transform.rotation, tr.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            T = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z], [0, 0, 0, 1]])
            K = np.array(self.info.k).reshape(3, 3)
            xy, info = pipeline.detect_target(rgb, depth, K, T)
            hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
            red = ((hsv[..., 0] <= 10) | (hsv[..., 0] >= 170)) & (hsv[..., 1] >= 60)
            mask = info["mask"] > 0
            print(f"         detector on this frame: {'target at ' + str(np.round(xy, 3)) if xy is not None else 'nothing'}; "
                  f"{info['blobs']} blobs >= min_area, rejected: {info['rejected']}; red-ish px {int(red.sum())}, "
                  f"mask px {int(mask.sum())}", flush=True)
            if red.any():
                v = hsv[..., 2][red]
                sat = hsv[..., 1][red]
                ys, xs = np.nonzero(red)
                print(f"         red-ish pixels: u {xs.min()}..{xs.max()}, v {ys.min()}..{ys.max()}, "
                      f"S p5 {np.percentile(sat, 5):.0f}, V p5 {np.percentile(v, 5):.0f}, V p50 {np.percentile(v, 50):.0f}",
                      flush=True)
            out = f"/tmp/p09_view_{tag}.png"
            vis = rgb.copy()
            vis[mask] = (0.5 * vis[mask] + [0, 127, 0]).astype(np.uint8)
            cv2.imwrite(out, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
            print(f"         frame saved: {out}", flush=True)
        except Exception as e:      # diagnostics must never break the gate
            print(f"         diagnosis failed: {e}", flush=True)

    def print_trace(self, stages):
        if not self.trace:
            return
        t0 = self.trace[0][0]
        print("         trace (sim s, stage, target z, tilt, fingers L/R):", flush=True)
        last = None
        for i, (t, st, x, y, z, tilt, fl, fr_) in enumerate(self.trace):
            key = (st, round(z, 2), round(fl, 3), round(fr_, 3))
            if key != last or i == len(self.trace) - 1:
                print(f"           {t - t0:5.1f}  {st:9s} z {z:.3f}  tilt {tilt:4.1f}  L {fl:.4f} R {fr_:.4f}", flush=True)
                last = key
        for t, s_ in self.attach_events:
            if t >= t0:
                print(f"           {t - t0:5.1f}  Gazebo attach state -> {s_}", flush=True)

    def run_pick(self, est, on_stage=None):
        goal = PickObject.Goal()
        goal.object_pose = PoseStamped()
        goal.object_pose.header.frame_id = est.header.frame_id
        goal.object_pose.pose.position.x = est.pose.position.x
        goal.object_pose.pose.position.y = est.pose.position.y
        goal.object_pose.pose.position.z = 0.075
        goal.object_pose.pose.orientation.w = 1.0
        goal.object_diameter, goal.object_height = 0.05, 0.15
        stages = []

        def fb(m):
            stages.append((m.feedback.stage, self.now()))
            self.cur_stage = m.feedback.stage
            if on_stage:
                on_stage(m.feedback.stage)
        if not self.pick.wait_for_server(timeout_sec=10):
            return None, stages
        t0 = time.time()
        h = self.wait_future(self.pick.send_goal_async(goal, feedback_callback=fb), 10)
        if not h or not h.accepted:
            return None, stages
        r = self.wait_future(h.get_result_async(), 150)
        return (r.result if r else None), stages + [("wall", time.time() - t0)]


def main():
    rclpy.init()
    n = Trials()
    w0 = time.time()
    while time.time() - w0 < 120 and not (n.pick.server_is_ready() and {"robot", "target"} <= set(n.gt)
                                          and all(j in n.js for j in ARM)):
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.pick.server_is_ready():
        print("FAIL  /pick_object not available (is pick_place_server running?)")
        sys.exit(1)
    print("PASS  /pick_object, ground truth and joint states available", flush=True)
    teleport("distractor_cylinder", *PARK_DISTRACTOR, 0.0, 0.001)

    def prepare(y0, bx, dyaw, label):
        n.reset_scene()
        if not n.place_robot(y0, 0.55, dyaw):
            print(f"      {label}: robot teleport failed", flush=True)
            return None, None
        est, waited = n.estimate()
        if est is None:
            last = n.track.observation_count if n.track is not None else 0
            print(f"      {label}: no target estimate with >= 10 observations in {waited:.0f} s "
                  f"({n.n_raw} single-frame detections, {n.n_track_msgs} track messages, last count {last})",
                  flush=True)
            n.diagnose_view(label.split("(")[0].strip().replace(" ", "_"))
            return None, None
        print(f"      {label}: estimate after {waited:.1f} s ({est.observation_count} observations, "
              f"std {est.position_stddev * 1000:.1f} mm)", flush=True)
        final = n.creep(est, bx)
        if final is None:
            print(f"      {label}: no TF odom -> base_footprint during the creep", flush=True)
            return None, None
        g = n.gt_target_in_base()
        print(f"      {label}: estimate after creep base ({final[0]:.3f}, {final[1]:+.3f}), "
              f"truth ({g[0]:.3f}, {g[1]:+.3f}), estimate error "
              f"{math.hypot(final[0] - g[0], final[1] - g[1]) * 1000:.1f} mm", flush=True)
        return est, n.gt["target"]

    print("=== picks ===", flush=True)
    ok_count, knocked, times = 0, [], []
    holds = []
    for i, (y0, bx, dyaw) in enumerate(TRIALS):
        label = f"pick {i + 1:2d} (y0 {y0:+.3f}, x {bx:.3f}, yaw {dyaw:+.2f})"
        est, t_start = prepare(y0, bx, dyaw, label)
        if est is None:
            report(label, False, "setup failed")
            continue
        before_close = {}

        def on_stage(s):
            if s == "close" and "target" in n.gt:
                before_close["xy"] = n.gt["target"][:2]
        n.trace, n.cur_stage = [], "start"
        res, stages = n.run_pick(est, on_stage)
        n.spin_sim(1.0)
        if not (res and res.success):
            n.print_trace(stages)
        # highest the target got during the pick (a failed pick may drop it again in its cleanup)
        lifted = max([z for (_, _, _, _, z, _, _, _) in n.trace] + [n.gt["target"][2]]) - t_start[2]
        if res and res.success:
            lifted = n.gt["target"][2] - t_start[2]     # success: it must still be up now
        n.trace = None
        if "xy" in before_close:
            knocked.append(math.hypot(before_close["xy"][0] - t_start[0], before_close["xy"][1] - t_start[1]))
        good = bool(res and res.success and lifted > 0.05 and n.attached)
        ok_count += good
        wall = stages[-1][1] if stages and stages[-1][0] == "wall" else float("nan")
        if res and res.success:
            times.append(wall)
        print(f"      {label}: {'OK  ' if good else 'FAIL'} {ERR.get(res.error_code, '?') if res else 'no result'} "
              f"'{res.message if res else ''}', lifted {lifted * 100:.1f} cm, attached {n.attached}, {wall:.1f} s",
              flush=True)
        if good and len(holds) < N_HOLD:
            n.spin_sim(0.5)
            e0 = n.object_vs_tool0()
            worst = 0.0
            n.drive(0.2, 0.0, 10.0)
            n.spin_sim(0.5)          # let the arm stop swaying before measuring
            e1 = n.object_vs_tool0()
            n.drive(0.0, 0.5, math.pi / 2 / 0.5)
            n.spin_sim(0.5)
            e2 = n.object_vs_tool0()
            for e in (e1, e2):
                if e0 and e:
                    worst = max(worst, math.dist(e, e0))
            holds.append(worst)
            print(f"      hold test after pick {i + 1}: object moved {worst * 1000:.1f} mm relative to tool0 "
                  f"(2 m drive + 90 deg turn), still attached {n.attached}", flush=True)
    report(">= 9/10 picks succeed (success + lifted > 5 cm + attached)", ok_count >= 9, f"{ok_count}/10")
    if times:
        print(f"      pick time: mean {sum(times) / len(times):.1f} s, max {max(times):.1f} s", flush=True)
    report("arm never knocked the target before closing (< 5 mm)", bool(knocked) and max(knocked) < 0.005,
           f"max {max(knocked) * 1000:.1f} mm over {len(knocked)} picks" if knocked else "no data")
    report("hold test 3/3: object moves < 0.02 m relative to tool0", len(holds) == 3 and max(holds) < 0.02,
           ", ".join(f"{h * 1000:.1f} mm" for h in holds) or "not run")

    print("=== missed grasp (target moved 0.2 m away when the descent starts) ===", flush=True)
    missed_ok = 0
    stow = srdf_state("stow")
    for j, (y0, bx, dyaw) in enumerate(MISSED):
        label = f"missed {j + 1}"
        est, _ = prepare(y0, bx, dyaw, label)
        if est is None:
            report(label, False, "setup failed")
            continue
        n.attached_ever = False
        moved = {}

        def on_stage(s):
            if s == "descend" and not moved:
                r = n.gt["robot"]
                t = n.gt["target"]
                moved["ok"] = teleport("target_cylinder", t[0] + 0.2 * math.cos(r[3]), t[1] + 0.2 * math.sin(r[3]),
                                       0.0, 0.001)
        res, _ = n.run_pick(est, on_stage)
        n.spin_sim(1.0)
        fingers_open = min(n.js.get(f, 0.0) for f in FINGERS) >= 0.035
        stowed = max(abs(n.js[a] - q) for a, q in zip(ARM, stow)) < 0.05
        code = ERR.get(res.error_code, "?") if res else "no result"
        good = bool(res and not res.success and code == "GRASP_NOT_DETECTED" and not n.attached_ever
                    and fingers_open and stowed and moved.get("ok"))
        missed_ok += good
        print(f"      {label}: {'OK  ' if good else 'FAIL'} {code} '{res.message if res else ''}', "
              f"attached ever {n.attached_ever}, fingers open {fingers_open}, stowed {stowed}", flush=True)
    report("missed grasp 3/3: GRASP_NOT_DETECTED, no attach, gripper open, arm stowed", missed_ok == 3,
           f"{missed_ok}/3")

    n.reset_scene()
    fails = results.count(False)
    print(f"\nP09 pick trials: {len(results) - fails}/{len(results)} PASS", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
