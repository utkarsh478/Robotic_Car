#!/usr/bin/env python3
"""Phase-8 gate: perception accuracy against ground truth in mission_world.

Needs: mission_sim (objects spawned, evaluator), gt_localizer.py (map->odom from ground truth,
because the gate teleports the robot), perception.launch.xml enabled:=true.

Poses (camera range/bearing are horizontal, measured from the camera, robot facing +y):
  target at E = (0.25, 2.35): range 0.6 / 1.0 / 1.5 / 2.0 m x bearing -0.35 / 0 / +0.35  (12)
  both target (+0.25 rad) and distractor (-0.25 rad) in view at 1.0 m                   (1)
  distractor only, 10 s                                                                  (1)
  home_box (-3.0, -0.9): (0.6, 0), (0.9, +0.25), (1.2, -0.25), (1.4, 0)                 (4)
  extra target views (0.8, +0.15), (1.2, -0.20)                                          (2)
PASS: target error < 0.02 m (range <= 1.5) / < 0.04 m (2.0); box < 0.03 m; distractor-only:
zero target messages for 10 s; with both in view every raw estimate is the target; latency
(image stamp -> publish) p95 < 0.2 s; `enabled:=false` silences the detector.
Samples (rgb.png, depth_mm.png, meta.json) go to tools/samples/ when it has none yet
(or CAPTURE_SAMPLES=1); the package's pytest replays them.
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from my_robot_interfaces.msg import Detection
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SAMPLES = os.path.join(WS, "tools", "samples")
WORLD = "mission_world"
E = (0.25, 2.35)                 # target test spot (middle zone, wall at y 2.95)
BOX = (-3.0, -0.9)               # home_box floor centre (mission_world.sdf)
PARK_TARGET = (1.2, -2.4)        # out of view of every test pose
PARK_DISTRACTOR = (-0.5, -2.4)
HEADING = math.pi / 2
CAM_OFF = (0.20, -0.12)          # camera_link in base_footprint (x, y)
ARM = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint"]
FX_EXPECTED = 320.0 / math.tan(0.6)
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def robot_pose_for(obj, rng, bearing, heading=HEADING):
    a = heading + bearing
    cx, cy = obj[0] - rng * math.cos(a), obj[1] - rng * math.sin(a)
    c, s = math.cos(heading), math.sin(heading)
    return cx - (c * CAM_OFF[0] - s * CAM_OFF[1]), cy - (s * CAM_OFF[0] + c * CAM_OFF[1]), heading


def camera_xy(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * CAM_OFF[0] - s * CAM_OFF[1], y + s * CAM_OFF[0] + c * CAM_OFF[1]


def teleport(name, x, y, yaw=0.0, z=0.01):
    req = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    r = subprocess.run(["ign", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "ignition.msgs.Pose",
                        "--reptype", "ignition.msgs.Boolean", "--timeout", "3000", "--req", req],
                       capture_output=True, text=True, timeout=15)
    return "true" in r.stdout


def srdf_stow():
    txt = open(os.path.join(WS, "src", "my_robot_moveit_config", "config", "my_robot.srdf")).read()
    body = re.search(r'<group_state name="stow" group="arm">(.*?)</group_state>', txt, re.S).group(1)
    vals = dict(re.findall(r'<joint name="([^"]+)" value="([^"]+)"', body))
    return [float(vals[j]) for j in ARM]


class Eval(Node):
    def __init__(self):
        super().__init__("perception_eval", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.bridge = CvBridge()
        self.gt = {}
        for k in ("robot", "target", "distractor"):
            self.create_subscription(Odometry, f"/evaluator/{k}_odom", lambda m, k=k: self._gt(k, m), 10)
        self.msgs = {k: [] for k in ("target", "target_raw", "box", "box_raw")}
        self.latency = []
        for k in self.msgs:
            self.create_subscription(Detection, f"/perception/{k}", lambda m, k=k: self._det(k, m), 50)
        self.info = None
        self.create_subscription(CameraInfo, "/camera/camera_info", self._info, qos_profile_sensor_data)
        self.rgb, self.depth = {}, {}      # filled only while grab_frame() runs (DC-31: no extra
                                           # permanent copies of the image streams)
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.reset_cli = {k: self.create_client(Trigger, f"/{k}_detector/reset") for k in ("target", "box")}
        self.param_cli = self.create_client(SetParameters, "/target_detector/set_parameters")

    # ---------------------------------------------------------------- callbacks
    def _gt(self, k, m):
        p = m.pose.pose.position
        self.gt[k] = (p.x, p.y)

    def _det(self, k, m):
        self.msgs[k].append(m)
        if k in ("target", "box"):
            self.latency.append(self.now() - Time.from_msg(m.header.stamp).nanoseconds * 1e-9)

    def _info(self, m):
        self.info = m

    def _img(self, store, m):
        store[(m.header.stamp.sec, m.header.stamp.nanosec)] = m
        if len(store) > 20:
            del store[min(store)]

    # ---------------------------------------------------------------- helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, s):
        t0, w0 = self.now(), time.time()
        while self.now() - t0 < s and time.time() - w0 < 4 * s + 10:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, cli, req, timeout=5.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        f = cli.call_async(req)
        w0 = time.time()
        while not f.done() and time.time() - w0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
        return f.result() if f.done() else None

    def reset(self):
        for c in self.reset_cli.values():
            self.call(c, Trigger.Request())
        for k in self.msgs:
            self.msgs[k].clear()

    def set_enabled(self, value):
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name="enabled", value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                                         bool_value=value))]
        r = self.call(self.param_cli, req)
        return bool(r and r.results and r.results[0].successful)

    def move_arm(self, q, dur=3.0):
        if not self.arm.wait_for_server(timeout_sec=10):
            return False
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(seconds=dur).to_msg()
        g.trajectory.points = [pt]
        f = self.arm.send_goal_async(g)
        w0 = time.time()
        while not f.done() and time.time() - w0 < 10:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not f.done() or not f.result().accepted:
            return False
        rf = f.result().get_result_async()
        while not rf.done() and time.time() - w0 < 40:
            rclpy.spin_once(self, timeout_sec=0.02)
        return rf.done() and rf.result().status == GoalStatus.STATUS_SUCCEEDED

    def place_robot(self, x, y, yaw):
        ok = teleport("my_robot", x, y, yaw)
        self.spin_sim(1.5)
        gx, gy = self.gt.get("robot", (1e9, 1e9))
        return ok and math.hypot(gx - x, gy - y) < 0.03

    def place(self, name, xy):
        ok = teleport(name, xy[0], xy[1], 0.0, 0.001)
        self.spin_sim(0.3)
        return ok

    def collect(self, seconds):
        self.reset()
        self.spin_sim(seconds)
        return {k: list(v) for k, v in self.msgs.items()}

    def grab_frame(self, timeout=3.0):
        self.rgb, self.depth = {}, {}
        subs = [self.create_subscription(Image, "/camera/image", lambda m: self._img(self.rgb, m), qos_profile_sensor_data),
                self.create_subscription(Image, "/camera/depth_image", lambda m: self._img(self.depth, m),
                                         qos_profile_sensor_data)]
        w0 = time.time()
        while time.time() - w0 < timeout and not (set(self.rgb) & set(self.depth)):
            rclpy.spin_once(self, timeout_sec=0.02)
        for sub in subs:
            self.destroy_subscription(sub)
        return sorted(set(self.rgb) & set(self.depth))

    def save_sample(self, idx, kind, meta_extra):
        common = self.grab_frame()
        if not common or self.info is None:
            print(f"         sample {idx:02d}: no synchronised RGB+depth frame within 3 s", flush=True)
            return None
        key = common[-1]
        rgb_m, dep_m = self.rgb[key], self.depth[key]
        # Spin while waiting: a blocking lookup with a timeout would starve this node's own /tf
        # subscription (nothing else spins it), which is why the last run saved only 8 of 20 frames.
        stamp = Time.from_msg(rgb_m.header.stamp)
        w0 = time.time()
        while not self.tf.can_transform("map", rgb_m.header.frame_id, stamp) and time.time() - w0 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.02)
        try:
            tr = self.tf.lookup_transform("map", rgb_m.header.frame_id, stamp)
        except TransformException as e:
            print(f"         sample {idx:02d}: no TF map -> {rgb_m.header.frame_id} at the frame stamp ({e})", flush=True)
            return None
        q, t = tr.transform.rotation, tr.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        T = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
             [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
             [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
             [0, 0, 0, 1]]
        rgb = self.bridge.imgmsg_to_cv2(rgb_m, "rgb8")
        depth = self.bridge.imgmsg_to_cv2(dep_m, "passthrough").astype(np.float32)
        mm = np.where(np.isfinite(depth), np.clip(depth * 1000.0, 0, 65535), 0).astype(np.uint16)
        d = os.path.join(SAMPLES, f"{idx:02d}_{kind}")
        os.makedirs(d, exist_ok=True)
        cv2.imwrite(os.path.join(d, "rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        cv2.imwrite(os.path.join(d, "depth_mm.png"), mm)
        meta = {"kind": kind, "K": np.array(self.info.k).reshape(3, 3).tolist(), "T_map_optical": T,
                "frame_id": rgb_m.header.frame_id, "stamp": rgb_m.header.stamp.sec + rgb_m.header.stamp.nanosec * 1e-9,
                "gt_target": self.gt.get("target"), "gt_distractor": self.gt.get("distractor"), "gt_box": BOX,
                **meta_extra}
        json.dump(meta, open(os.path.join(d, "meta.json"), "w"), indent=1)
        return rgb


def hsv_line(rgb, hue_ranges=((0, 10), (170, 180)), s_min=120, v_min=50):
    """HSV statistics of the red mask in a sample (helps re-tune the thresholds)."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m = np.zeros(rgb.shape[:2], bool)
    for lo, hi in hue_ranges:
        m |= (hsv[..., 0] >= lo) & (hsv[..., 0] <= hi) & (hsv[..., 1] >= s_min) & (hsv[..., 2] >= v_min)
    if not m.any():
        return "no red pixels"
    h, s, v = (hsv[..., i][m] for i in range(3))
    h = np.where(h > 90, h.astype(int) - 180, h)
    return f"{m.sum()} px, H {np.percentile(h, 2):+.0f}..{np.percentile(h, 98):+.0f}, S p5 {np.percentile(s, 5):.0f}, V p5 {np.percentile(v, 5):.0f}"


def main():
    rclpy.init()
    n = Eval()
    w0 = time.time()
    while time.time() - w0 < 90 and not (n.info and {"robot", "target", "distractor"} <= set(n.gt)
                                         and n.tf.can_transform("map", "camera_optical_link", Time())):
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.info:
        print("FAIL  no /camera/camera_info")
        sys.exit(1)
    if not n.tf.can_transform("map", "camera_optical_link", Time()):
        print("FAIL  no TF map -> camera_optical_link (gt_localizer running?)")
        sys.exit(1)

    K = np.array(n.info.k).reshape(3, 3)
    report("camera_info frame = camera_optical_link", n.info.header.frame_id == "camera_optical_link",
           n.info.header.frame_id)
    report("intrinsics match hfov 1.2 / 640x480", abs(K[0, 0] - FX_EXPECTED) / FX_EXPECTED < 0.01
           and abs(K[1, 1] - FX_EXPECTED) / FX_EXPECTED < 0.01 and abs(K[0, 2] - 320) < 2 and abs(K[1, 2] - 240) < 2,
           f"fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} (expected {FX_EXPECTED:.1f}), cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}")
    report("arm to stow (out of the camera view)", n.move_arm(srdf_stow()))

    capture = os.environ.get("CAPTURE_SAMPLES") == "1" or not any(
        os.path.isfile(os.path.join(SAMPLES, d, "meta.json")) for d in (os.listdir(SAMPLES) if os.path.isdir(SAMPLES) else []))
    if capture and os.path.isdir(SAMPLES):
        shutil.rmtree(SAMPLES)
    print(f"      samples: {'capturing to ' + SAMPLES if capture else 'kept (CAPTURE_SAMPLES=1 to replace)'}", flush=True)
    idx = [0]

    def sample(kind, extra):
        if capture:
            idx[0] += 1
            rgb = n.save_sample(idx[0], kind, extra)
            if rgb is None:
                idx[0] -= 1
            if rgb is not None and kind in ("target", "both"):
                print(f"         sample {idx[0]:02d}: red mask {hsv_line(rgb)}", flush=True)

    ok = n.place("distractor_cylinder", PARK_DISTRACTOR) and n.place("target_cylinder", E)
    report("objects placed (teleport via /world/mission_world/set_pose)", ok)

    print("=== target: range x bearing ===", flush=True)
    worst = {"near": 0.0, "far": 0.0}
    min_rate = [1e9]
    poses = [(r, b) for r in (0.6, 1.0, 1.5, 2.0) for b in (-0.35, 0.0, 0.35)]
    for r, b in poses + [(0.8, 0.15), (1.2, -0.20)]:
        if not n.place_robot(*robot_pose_for(E, r, b)):
            report(f"teleport robot for range {r} bearing {b:+.2f}", False)
            continue
        got = n.collect(3.0)
        gt = np.array(n.gt["target"])
        sample("target", {"range": r, "bearing": b})
        if not got["target"]:
            min_rate[0] = 0.0
            report(f"target seen at range {r:.1f} bearing {b:+.2f}", False, "no detections in 3 s")
            continue
        last = got["target"][-1]
        err = math.hypot(last.pose.position.x - gt[0], last.pose.position.y - gt[1])
        raw = [math.hypot(m.pose.position.x - gt[0], m.pose.position.y - gt[1]) for m in got["target_raw"]]
        min_rate[0] = min(min_rate[0], len(raw) / 3.0)
        key = "near" if r <= 1.5 else "far"
        worst[key] = max(worst[key], err)
        lim = 0.02 if r <= 1.5 else 0.04
        print(f"      range {r:.1f} bearing {b:+.2f}: track error {err * 1000:5.1f} mm (limit {lim * 1000:.0f}), "
              f"{len(raw)} estimates ({len(raw) / 3.0:.1f} Hz), raw max {max(raw) * 1000:5.1f} mm, std {last.position_stddev * 1000:4.1f} mm", flush=True)
    report("target estimates at >= 5 Hz at every pose (15 Hz camera)", min_rate[0] >= 5.0,
           f"lowest {min_rate[0]:.1f} Hz")
    report("target error < 0.02 m at range <= 1.5 m", worst["near"] < 0.02, f"worst {worst['near'] * 1000:.1f} mm")
    report("target error < 0.04 m at range 2.0 m", worst["far"] < 0.04, f"worst {worst['far'] * 1000:.1f} mm")

    print("=== target and distractor both in view (1.0 m, +-0.25 rad) ===", flush=True)
    x, y, yaw = robot_pose_for(E, 1.0, 0.0)
    cx, cy = camera_xy(x, y, yaw)
    t_xy = (cx + math.cos(yaw + 0.25), cy + math.sin(yaw + 0.25))
    d_xy = (cx + math.cos(yaw - 0.25), cy + math.sin(yaw - 0.25))
    n.place("target_cylinder", t_xy)
    n.place("distractor_cylinder", d_xy)
    n.place_robot(x, y, yaw)
    got = n.collect(3.0)
    gt = n.gt["target"]
    raw = [math.hypot(m.pose.position.x - gt[0], m.pose.position.y - gt[1]) for m in got["target_raw"]]
    err = (math.hypot(got["target"][-1].pose.position.x - gt[0], got["target"][-1].pose.position.y - gt[1])
           if got["target"] else float("inf"))
    report("with the distractor in view: target error < 0.02 m", err < 0.02, f"{err * 1000:.1f} mm")
    report("with the distractor in view: every raw estimate is the target (< 0.10 m)",
           bool(raw) and max(raw) < 0.10, f"{len(raw)} estimates, max {max(raw) * 1000 if raw else 0:.1f} mm")
    sample("both", {"range": 1.0, "bearing": 0.25})

    print("=== distractor only, 10 s ===", flush=True)
    n.place("target_cylinder", PARK_TARGET)
    n.place("distractor_cylinder", E)
    n.place_robot(*robot_pose_for(E, 1.0, 0.0))
    got = n.collect(10.0)
    report("distractor only: zero target detections in 10 s", not got["target"] and not got["target_raw"],
           f"{len(got['target'])} track / {len(got['target_raw'])} raw messages")
    sample("distractor", {"range": 1.0, "bearing": 0.0})

    print("=== enabled:=false silences the detector ===", flush=True)
    n.place("distractor_cylinder", PARK_DISTRACTOR)
    n.place("target_cylinder", E)
    n.spin_sim(1.0)
    off_ok = n.set_enabled(False)
    got_off = n.collect(3.0)
    on_ok = n.set_enabled(True)
    got_on = n.collect(4.0)
    report("enabled false -> no messages; true -> messages again",
           off_ok and on_ok and not got_off["target"] and bool(got_on["target"]),
           f"off: {len(got_off['target'])} msgs, on: {len(got_on['target'])} msgs")

    print("=== home box ===", flush=True)
    n.place("target_cylinder", PARK_TARGET)
    worst_box = 0.0
    for r, b in ((0.6, 0.0), (0.9, 0.25), (1.2, -0.25), (1.4, 0.0)):
        if not n.place_robot(*robot_pose_for(BOX, r, b)):
            report(f"teleport robot for box range {r}", False)
            continue
        got = n.collect(3.0)
        sample("box", {"range": r, "bearing": b})
        if not got["box"]:
            report(f"box seen at range {r:.1f} bearing {b:+.2f}", False, "no detections in 3 s")
            continue
        last = got["box"][-1]
        err = math.hypot(last.pose.position.x - BOX[0], last.pose.position.y - BOX[1])
        worst_box = max(worst_box, err)
        yaw = 2 * math.atan2(last.pose.orientation.z, last.pose.orientation.w)
        print(f"      range {r:.1f} bearing {b:+.2f}: box error {err * 1000:5.1f} mm, yaw {math.degrees(yaw):+.1f} deg "
              f"(true 0, mod 90), {len(got['box_raw'])} frames", flush=True)
    report("box centre error < 0.03 m", worst_box < 0.03, f"worst {worst_box * 1000:.1f} mm")

    lat = np.array(n.latency)
    lat = lat[lat > -1.0]
    report("latency image stamp -> publish p95 < 0.2 s", len(lat) > 0 and np.percentile(lat, 95) < 0.2,
           f"p50 {np.percentile(lat, 50) * 1000:.0f} ms, p95 {np.percentile(lat, 95) * 1000:.0f} ms, "
           f"max {lat.max() * 1000:.0f} ms over {len(lat)} messages" if len(lat) else "no messages")
    if capture:
        print(f"      wrote {idx[0]} samples to {SAMPLES}", flush=True)

    fails = results.count(False)
    print(f"\nP08 perception eval: {len(results) - fails}/{len(results)} PASS", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
