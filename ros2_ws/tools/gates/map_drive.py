#!/usr/bin/env python3
"""Phase-3 mapping drive: steer the robot through all three zones of mission_world so
slam_toolbox can build the map. GATE TOOLING ONLY: it steers on Gazebo ground truth
(/evaluator/robot_odom), which autonomy code must never do (CLAUDE.md rule 4). The SLAM
map itself is built only from /scan + /odom.

Route (world frame, SPEC §8.2): home -> top gap -> middle zone -> bottom gap -> right zone,
with 360 deg spins so the front-only (180 deg) lidar also sees what is behind it.
Also logs SLAM's pose (TF map->base_footprint, shifted by the spawn pose so it is in world
coordinates) against ground truth once per sim-second, to results/p03_slam_drift.csv, and
reports where they diverge. During mapping the map frame starts at the spawn pose (yaw 0).
Exit 0 = every waypoint reached (drift is reported separately as PASS/FAIL).
"""
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

SPAWN = (-3.0, -2.0)
SPIN = "spin"
ROUTE = [
    (-2.4, -2.0), SPIN,          # 0.6 m out first, so the spin also sees the home cell
    (-1.6, -0.6),
    (-1.6, 0.9), SPIN,
    (-1.6, 2.3),
    (-0.4, 2.3),                 # through the top gap of W1
    (0.4, 1.6),
    (0.95, 0.4), SPIN,
    (0.95, -1.0),                # between O1 and W2
    (0.95, -2.3),
    (2.3, -2.3),                 # through the bottom gap of W2
    (3.3, -1.6), SPIN,
    (3.4, 0.0),                  # east of O2
    (3.3, 1.6),
    (2.2, 2.3), SPIN,
    (2.2, 1.0),
    # --- return leg (P3 run 5): the home zone stayed unknown although the robot spun there
    #     at the start, so come back and DRIVE through it at the end; also pass west of O1.
    (1.95, 0.6),
    (1.95, -0.3),                # between W2 and O2 (0.40 m clear of both)
    (2.0, -2.3),
    (0.95, -2.3),                # back through the bottom gap
    (0.95, 0.4),
    (-0.45, -0.2), SPIN,         # west of O1: sees its west face
    (-0.45, 2.3),
    (-1.6, 2.3),                 # back through the top gap
    (-1.6, -0.6),
    (-2.4, -2.0),                # into the home zone heading south-west
    (-3.3, -2.4), SPIN,          # past home and the box's south face
]
V_MAX, W_MAX = 0.25, 0.8         # slow: cleaner scans
TOL = 0.15                       # waypoint reached [m]
WP_TIMEOUT = 90.0                # sim seconds per waypoint
STOP_DIST = 0.25                 # [m from lidar] obstacle straight ahead -> stop
DRIFT_OK = 0.15                  # [m] SLAM pose vs truth, PASS threshold
SNAP_AT = {1: "after_home_spin", 3: "after_wp3", 19: "end_outbound", 32: "home_spin_return", None: "final"}
SNAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "results", "p03_snapshots")
CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "results", "p03_slam_drift.csv")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    def __init__(self):
        super().__init__("map_drive", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.raw = None
        self.offset = (0.0, 0.0)
        self.scan = None
        self.create_subscription(Odometry, "/evaluator/robot_odom", self._gt, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.map_occupied = 0
        self.create_subscription(
            OccupancyGrid, "/map", self._map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.log = []            # (t, gt_x, gt_y, gt_yaw, slam_x, slam_y, slam_yaw, err_xy, err_yaw)
        self._next_log = 0.0
        self.waypoint = ""

    def slam_pose(self):
        """map->base_footprint, shifted into world coordinates. None if not available."""
        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        except Exception:  # noqa: BLE001  (tf2 raises several exception types)
            return None
        return (tr.translation.x + SPAWN[0], tr.translation.y + SPAWN[1], yaw_of(tr.rotation))

    def sample(self, force=False):
        t = self.now()
        if not force and t < self._next_log:
            return None
        self._next_log = t + 1.0
        sp = self.slam_pose()
        if sp is None or self.raw is None:
            return None
        gx, gy, gyaw = self.pose
        exy = math.hypot(sp[0] - gx, sp[1] - gy)
        eyaw = abs(wrap(sp[2] - gyaw))
        self.log.append((round(t, 2), gx, gy, gyaw, sp[0], sp[1], sp[2], exy, eyaw, self.waypoint))
        if exy > 0.5 and not getattr(self, "_warned", False):
            self._warned = True
            print(f"  !! SLAM diverged at t_sim {t:.0f} s near truth ({gx:+.2f}, {gy:+.2f}): "
                  f"SLAM says ({sp[0]:+.2f}, {sp[1]:+.2f}), yaw err {math.degrees(eyaw):.0f} deg "
                  f"[{self.waypoint}]", flush=True)
        return exy, eyaw

    def _gt(self, msg):
        p = msg.pose.pose
        self.raw = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _scan(self, msg):
        self.scan = msg

    def _map(self, msg):
        self.map_occupied = sum(1 for v in msg.data if v >= 65)
        self.map_msg = msg

    def snapshot(self, name):
        """Save the live /map as PGM+YAML, origin shifted to world coordinates (map == world)."""
        m = getattr(self, "map_msg", None)
        if m is None:
            return
        os.makedirs(SNAP_DIR, exist_ok=True)
        w, h = m.info.width, m.info.height
        rows = []
        for j in reversed(range(h)):                     # PGM row 0 = top = max y
            row = m.data[j * w:(j + 1) * w]
            # same trinary rule as map_saver: >= 65 occupied, <= 25 free, else unknown
            rows.append(bytes(0 if v >= 65 else (254 if 0 <= v <= 25 else 205) for v in row))
        base = os.path.join(SNAP_DIR, name)
        with open(base + ".pgm", "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode() + b"".join(rows))
        o = m.info.origin.position
        with open(base + ".yaml", "w") as f:
            f.write(f"image: {name}.pgm\nmode: trinary\nresolution: {m.info.resolution}\n"
                    f"origin: [{o.x + SPAWN[0]:.4f}, {o.y + SPAWN[1]:.4f}, 0.0]\n"
                    f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        print(f"      snapshot {name}: {w}x{h}, {self.map_occupied} occupied cells", flush=True)

    def wait_slam_ready(self, min_occupied=50, wall_timeout=90.0):
        """P3 run 4: scans from the first ~20 s never reached the map (home zone stayed
        unknown) because the robot moved before slam_toolbox was integrating scans.
        Wait for a real map (>= min_occupied occupied cells from the first scan) AND the
        map->base_footprint TF, then settle 3 s so the first keyframe is the home pose."""
        w0 = time.time()
        while time.time() - w0 < wall_timeout:
            self.tick()
            self.cmd(0.0, 0.0)
            if self.map_occupied >= min_occupied and self.slam_pose() is not None:
                self.settle(3.0)
                return time.time() - w0
        return None

    @property
    def pose(self):
        x, y, yaw = self.raw
        return x + self.offset[0], y + self.offset[1], yaw

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0.02)
        self.sample()

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def front_min(self, half=0.44):
        s = self.scan
        if s is None:
            return float("inf")
        best = float("inf")
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if abs(a) <= half and math.isfinite(r) and s.range_min <= r <= s.range_max:
                best = min(best, r)
        return best

    def goto(self, gx, gy):
        t0, blocked_since = self.now(), None
        while True:
            self.tick()
            x, y, yaw = self.pose
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < TOL:
                return True
            if self.now() - t0 > WP_TIMEOUT:
                print(f"  timeout at ({x:.2f}, {y:.2f}) going to ({gx}, {gy})")
                return False
            err = wrap(math.atan2(dy, dx) - yaw)
            if abs(err) > 0.35:
                self.cmd(0.0, max(-W_MAX, min(W_MAX, 1.5 * err)))
                continue
            v = min(V_MAX, 0.6 * dist)
            if self.front_min() < STOP_DIST:
                blocked_since = blocked_since or self.now()
                if self.now() - blocked_since > 15.0:
                    print(f"  blocked >15 s at ({x:.2f}, {y:.2f}): front obstacle {self.front_min():.2f} m")
                    return False
                v = 0.0
            else:
                blocked_since = None
            self.cmd(v, max(-W_MAX, min(W_MAX, 1.5 * err)))

    def spin360(self):
        turned, last = 0.0, self.pose[2]
        t0 = self.now()
        while turned < 2 * math.pi:
            self.tick()
            yaw = self.pose[2]
            turned += abs(wrap(yaw - last))
            last = yaw
            self.cmd(0.0, 0.5)
            if self.now() - t0 > 40.0:
                return False
        return True

    def settle(self, sim_s):
        t0 = self.now()
        while self.now() - t0 < sim_s:
            self.tick()
            self.cmd(0.0, 0.0)


def main():
    rclpy.init()
    d = Driver()
    w0 = time.time()
    while (d.raw is None or d.scan is None or d.now() == 0.0) and time.time() - w0 < 60:
        d.tick()
    if d.raw is None:
        print("FAIL  no /evaluator/robot_odom (is mission_sim running?)")
        sys.exit(1)

    # Ground truth should be absolute (world frame). If it reads ~(0,0) while we spawned
    # at (-3,-2), the plugin is reporting relative to the start pose: add the spawn offset.
    x, y, _ = d.raw
    if math.hypot(x - SPAWN[0], y - SPAWN[1]) < 0.1:
        print(f"ground truth is world-frame: ({x:.3f}, {y:.3f})")
    elif math.hypot(x, y) < 0.1:
        d.offset = SPAWN
        print(f"NOTE  ground truth is relative to start ({x:.3f}, {y:.3f}); adding spawn offset {SPAWN}")
    else:
        print(f"FAIL  robot ground truth ({x:.3f}, {y:.3f}) is neither spawn {SPAWN} nor origin")
        sys.exit(1)

    ready = d.wait_slam_ready()
    if ready is None:
        print(f"FAIL  slam_toolbox not integrating scans after 90 s (occupied cells in /map: {d.map_occupied})")
        sys.exit(1)
    print(f"slam ready after {ready:.1f} s wall ({d.map_occupied} occupied cells from the first scan)", flush=True)
    d.snapshot("start")

    ok_all = True
    for i, wp in enumerate(ROUTE):
        d.waypoint = f"wp {i}: {'spin' if wp == SPIN else wp}"
        if wp == SPIN:
            ok = d.spin360()
            e = d.sample(force=True)
            drift = f"  slam err {e[0]:.3f} m / {math.degrees(e[1]):.1f} deg" if e else "  slam pose n/a"
            print(f"[{i:2d}] spin 360 {'ok' if ok else 'TIMEOUT'}{drift}", flush=True)
        else:
            ok = d.goto(*wp)
            x, y, _ = d.pose
            e = d.sample(force=True)
            drift = f"  slam err {e[0]:.3f} m / {math.degrees(e[1]):.1f} deg" if e else "  slam pose n/a"
            print(f"[{i:2d}] -> ({wp[0]:+.2f}, {wp[1]:+.2f})  {'ok' if ok else 'FAILED'}  at ({x:+.2f}, {y:+.2f})"
                  f"  t_sim {d.now():.0f} s{drift}", flush=True)
        ok_all &= ok
        if i in SNAP_AT:
            d.settle(2.5)        # map_update_interval is 2 s
            d.snapshot(f"{i:02d}_{SNAP_AT[i]}")
        if not ok:
            break
    d.settle(4.0)  # let slam_toolbox publish a final map update
    d.snapshot(SNAP_AT[None])
    print("PASS  mapping drive complete" if ok_all else "FAIL  mapping drive aborted")

    os.makedirs(os.path.dirname(CSV), exist_ok=True)
    with open(CSV, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sim", "gt_x", "gt_y", "gt_yaw", "slam_x", "slam_y", "slam_yaw", "err_xy", "err_yaw", "segment"])
        w.writerows(d.log)
    if d.log:
        worst = max(d.log, key=lambda r: r[7])
        print(f"{'PASS' if worst[7] <= DRIFT_OK else 'FAIL'}  SLAM pose vs truth max {worst[7]:.3f} m "
              f"(<= {DRIFT_OK}) at t_sim {worst[0]:.0f} s [{worst[9]}]; "
              f"max yaw err {math.degrees(max(r[8] for r in d.log)):.1f} deg; log {os.path.normpath(CSV)}")
    else:
        print("FAIL  no SLAM pose samples (TF map->base_footprint never available)")
    d.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
