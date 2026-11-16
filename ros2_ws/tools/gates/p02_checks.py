#!/usr/bin/env python3
"""Phase-2 gate checks (run while my_robot_gazebo.launch.xml is up).

Checks (PHASES.md P2):
  topic rates (in sim time), frame ids, lidar self-hits (before/after 180 deg),
  straight drive vs odom vs ground truth, 2*pi rotation, stop on zero command, RTF.
Ground truth (/evaluator/robot_odom) is read here ONLY: gate scripts may, autonomy may not.
Exit 0 = all PASS.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

RATE_SPEC = {  # topic: (expected Hz, tolerance Hz)
    "/scan": (10.0, 1.0),
    "/imu": (100.0, 10.0),
    "/camera/image": (15.0, 2.0),
    "/odom": (50.0, 5.0),
}
FRAME_SPEC = {"/scan": "lidar_link", "/imu": "imu_link",
              "/camera/image": "camera_optical_link", "/camera/depth_image": "camera_optical_link"}

results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Track:
    """Pose from an Odometry topic, with continuously unwrapped yaw."""

    def __init__(self):
        self.x = self.y = None
        self.yaw_u = 0.0
        self._last_yaw = None
        self.vx = self.wz = 0.0

    def update(self, msg):
        p = msg.pose.pose
        self.x, self.y = p.position.x, p.position.y
        yaw = yaw_of(p.orientation)
        if self._last_yaw is not None:
            self.yaw_u += wrap(yaw - self._last_yaw)
        self._last_yaw = yaw
        self.vx, self.wz = msg.twist.twist.linear.x, msg.twist.twist.angular.z

    def snap(self):
        return (self.x, self.y, self.yaw_u)


class Gate(Node):
    def __init__(self):
        super().__init__("gate_p02", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.stamps = {t: [] for t in RATE_SPEC}
        self.frames = {}
        self.seen = set()
        self.scan = None
        self.odom = Track()
        self.gt = Track()
        q = qos_profile_sensor_data  # best effort: compatible with reliable and best-effort publishers
        self.create_subscription(LaserScan, "/scan", lambda m: self._sensor("/scan", m), q)
        self.create_subscription(Imu, "/imu", lambda m: self._sensor("/imu", m), q)
        self.create_subscription(Image, "/camera/image", lambda m: self._sensor("/camera/image", m), q)
        self.create_subscription(Image, "/camera/depth_image", lambda m: self._sensor("/camera/depth_image", m), q)
        self.create_subscription(CameraInfo, "/camera/camera_info", lambda m: self._sensor("/camera/camera_info", m), q)
        self.create_subscription(Odometry, "/odom", self._odom, q)
        self.create_subscription(Odometry, "/evaluator/robot_odom", self._gt, q)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    # ---------- callbacks ----------
    def _sensor(self, topic, msg):
        self.seen.add(topic)
        self.frames[topic] = msg.header.frame_id
        if topic in self.stamps:
            self.stamps[topic].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if topic == "/scan":
            self.scan = msg

    def _odom(self, msg):
        self.seen.add("/odom")
        self.frames["/odom"] = f"{msg.header.frame_id}->{msg.child_frame_id}"
        self.stamps["/odom"].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.odom.update(msg)

    def _gt(self, msg):
        self.seen.add("/evaluator/robot_odom")
        self.gt.update(msg)

    # ---------- helpers ----------
    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for_sim(self, duration, cmd=None, rate_hz=20.0, wall_limit=120.0):
        """Spin until `duration` seconds of SIM time pass, publishing `cmd` at rate_hz."""
        t0, w0 = self.sim_now(), time.time()
        next_pub = t0
        while self.sim_now() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.005)
            if cmd is not None and self.sim_now() >= next_pub:
                self.cmd_pub.publish(cmd)
                next_pub += 1.0 / rate_hz
            if time.time() - w0 > wall_limit:
                raise RuntimeError("sim time stopped advancing (is Gazebo paused or crashed?)")

    def drive(self, vx, wz, duration):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = vx, wz
        self.spin_for_sim(duration, cmd)
        self.spin_for_sim(1.5, Twist())  # explicit zero, then let it settle

    def scan_check(self, label):
        s = self.scan
        if s is None:
            report(f"lidar self-hit check ({label})", False, "no scan")
            return
        valid = [r for r in s.ranges if math.isfinite(r) and s.range_min <= r <= s.range_max]
        mn = min(valid) if valid else float("inf")
        report(f"lidar: no self-hits ({label}), min valid range > 0.5 m", mn > 0.5,
               f"min {mn:.3f} m, {len(valid)}/{len(s.ranges)} valid returns")
        report(f"lidar: sees the world ({label})", len(valid) > 0,
               "no returns at all: check Sensors system / headless rendering" if not valid else "")


def main():
    rclpy.init()
    g = Gate()

    # ---- 1. everything is publishing ----
    need = {"/clock", "/scan", "/imu", "/camera/image", "/camera/depth_image",
            "/camera/camera_info", "/odom", "/evaluator/robot_odom"}
    w0 = time.time()
    while time.time() - w0 < 60.0:
        rclpy.spin_once(g, timeout_sec=0.05)
        if g.sim_now() > 0:
            g.seen.add("/clock")
        if need <= g.seen:
            break
    missing = sorted(need - g.seen)
    report("all topics publishing", not missing, f"missing {missing}" if missing else "")
    if "/clock" in missing or "/odom" in missing:
        print("Cannot continue without /clock and /odom.")
        sys.exit(1)

    subs = g.count_subscribers("/cmd_vel")
    report("diff_drive_controller subscribes to /cmd_vel (remap applied)", subs >= 1,
           f"{subs} subscriber(s)" + ("" if subs else
           "; controller is probably on /diff_drive_controller/cmd_vel_unstamped (check `ros2 topic list`)"))

    # ---- 2. rates (sim time) + RTF ----
    for k in g.stamps:
        g.stamps[k].clear()
    s0, w0 = g.sim_now(), time.time()
    g.spin_for_sim(5.0)
    rtf = (g.sim_now() - s0) / max(1e-6, time.time() - w0)
    report("real-time factor >= 0.8", rtf >= 0.8, f"RTF {rtf:.2f}")
    for topic, (hz, tol) in RATE_SPEC.items():
        st = g.stamps[topic]
        rate = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 2 and st[-1] > st[0] else 0.0
        report(f"rate {topic} = {hz:.0f} +/- {tol:.0f} Hz", abs(rate - hz) <= tol, f"{rate:.1f} Hz")

    # ---- 3. frame ids ----
    for topic, frame in FRAME_SPEC.items():
        got = g.frames.get(topic)
        report(f"frame_id {topic} = {frame}", got == frame, str(got))
    report("odom frames odom->base_footprint", g.frames.get("/odom") == "odom->base_footprint",
           str(g.frames.get("/odom")))

    # ---- 4. lidar self-hits at start ----
    g.scan_check("start, facing +x")

    # ---- 5. straight drive 0.2 m/s for 5 s ----
    o0, t0 = g.odom.snap(), g.gt.snap()
    g.drive(0.2, 0.0, 5.0)
    o1, t1 = g.odom.snap(), g.gt.snap()
    d_odom = math.hypot(o1[0] - o0[0], o1[1] - o0[1])
    d_gt = math.hypot(t1[0] - t0[0], t1[1] - t0[1])
    report("straight: odom distance 1.00 +/- 0.05 m", abs(d_odom - 1.0) <= 0.05, f"{d_odom:.3f} m")
    agree = abs(d_gt - d_odom) / max(d_odom, 1e-6)
    report("straight: ground truth agrees with odom within 3 %", agree <= 0.03,
           f"truth {d_gt:.3f} m, diff {agree * 100:.1f} %")
    report("straight: heading drift < 0.05 rad", abs(o1[2] - o0[2]) < 0.05 and abs(t1[2] - t0[2]) < 0.05,
           f"odom {o1[2] - o0[2]:+.4f}, truth {t1[2] - t0[2]:+.4f} rad")
    report("stops on zero command (|v| < 0.01 m/s)", abs(g.odom.vx) < 0.01 and abs(g.odom.wz) < 0.01,
           f"vx {g.odom.vx:+.4f}, wz {g.odom.wz:+.4f}")

    # ---- 6. rotate pi, check lidar facing backwards, rotate another pi ----
    o0, t0 = g.odom.snap(), g.gt.snap()
    g.drive(0.0, 0.5, math.pi / 0.5)
    g.scan_check("after 180 deg turn")
    g.drive(0.0, 0.5, math.pi / 0.5)
    o1, t1 = g.odom.snap(), g.gt.snap()
    dyaw_o, dyaw_t = o1[2] - o0[2], t1[2] - t0[2]
    report("rotation: odom yaw change = 2*pi +/- 0.10 rad", abs(dyaw_o - 2 * math.pi) <= 0.10,
           f"{dyaw_o:.3f} rad")
    report("rotation: ground truth agrees with odom within 0.10 rad", abs(dyaw_t - dyaw_o) <= 0.10,
           f"truth {dyaw_t:.3f} rad")
    drift = math.hypot(t1[0] - t0[0], t1[1] - t0[1])
    report("rotation in place: position drift < 0.05 m", drift < 0.05, f"{drift:.3f} m")

    n_fail = results.count(False)
    print(f"\nP02 checks: {len(results) - n_fail}/{len(results)} PASS", flush=True)
    g.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
