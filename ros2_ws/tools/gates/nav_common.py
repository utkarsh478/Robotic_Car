#!/usr/bin/env python3
"""Shared Phase-4 gate helpers: send Nav2 goals and score them against Gazebo ground truth.

GATE TOOLING ONLY (CLAUDE.md rule 4): reads /evaluator/robot_odom.

Collision check (no contact sensor needed): every true-pose sample (2 Hz, sim time), the
robot's PHYSICAL shapes are tested against the obstacle boundaries of mission_world.sdf
(sampled every 2.5 cm by check_map.boundary_samples) plus any extra obstacles a test spawns:
  chassis box |x| <= 0.25, |y| <= 0.16 ; wheel spheres (0, +-0.19) r 0.075 ;
  lidar puck (0.27, 0) r 0.035          (SPEC §3.1-3.2, DC-8)
A boundary point inside any shape = contact.
"""
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_map  # noqa: E402

WS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
WORLD = os.path.join(WS, "src", "my_robot_bringup", "worlds", "mission_world.sdf")
STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def box_points(cx, cy, sx, sy, step=0.025):
    pts = []
    hx, hy = sx / 2, sy / 2
    for k in range(int(sx / step) + 1):
        x = cx - hx + k * step
        pts += [(x, cy - hy), (x, cy + hy)]
    for k in range(int(sy / step) + 1):
        y = cy - hy + k * step
        pts += [(cx - hx, y), (cx + hx, y)]
    return pts


def robot_contains(px, py, x, y, yaw):
    """True if world point (px, py) is inside the robot's physical body at pose (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - x, py - y
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    if abs(lx) <= 0.25 and abs(ly) <= 0.16:
        return True
    if math.hypot(lx, ly - 0.19) <= 0.075 or math.hypot(lx, ly + 0.19) <= 0.075:
        return True
    return math.hypot(lx - 0.27, ly) <= 0.035


class NavMonitor(Node):
    def __init__(self, name, extra_obstacle_points=()):
        super().__init__(name, parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.gt = None          # (x, y, yaw, v, w)
        self.create_subscription(Odometry, "/evaluator/robot_odom", self._gt, qos_profile_sensor_data)
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.points = [p for _, (_, pts) in check_map.boundary_samples(WORLD).items() for p in pts]
        self.points += list(extra_obstacle_points)
        self.contacts = []      # (t, x, y, px, py)
        self.amcl_err = []      # (t, err_xy, err_yaw)
        self.create_timer(0.5, self._sample)

    # ---------- callbacks ----------
    def _gt(self, msg):
        p, t = msg.pose.pose, msg.twist.twist
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation),
                   math.hypot(t.linear.x, t.linear.y), t.angular.z)

    def _sample(self):
        if self.gt is None:
            return
        x, y, yaw = self.gt[:3]
        t = self.now()
        for px, py in self.points:
            if abs(px - x) < 0.45 and abs(py - y) < 0.45 and robot_contains(px, py, x, y, yaw):
                self.contacts.append((round(t, 1), round(x, 3), round(y, 3), round(px, 3), round(py, 3)))
                break
        a = self.amcl_pose()
        if a is not None:
            self.amcl_err.append((t, math.hypot(a[0] - x, a[1] - y), abs(wrap(a[2] - yaw))))

    # ---------- helpers ----------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_wall(self, seconds):
        w0 = time.time()
        while time.time() - w0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_sim(self, seconds, wall_limit=60.0):
        t0, w0 = self.now(), time.time()
        while self.now() - t0 < seconds and time.time() - w0 < wall_limit:
            rclpy.spin_once(self, timeout_sec=0.05)

    def amcl_pose(self):
        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        except Exception:  # noqa: BLE001
            return None
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def wait_ready(self, wall_timeout=90.0):
        """Action server up, AMCL publishing map->base_footprint, ground truth flowing."""
        w0 = time.time()
        while time.time() - w0 < wall_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.client.server_is_ready() and self.amcl_pose() is not None and self.gt is not None:
                return time.time() - w0
        return None

    def go(self, x, y, yaw, timeout=120.0):
        """Send one NavigateToPose goal in map (== world) frame and wait. Returns a result dict."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        n_contacts0 = len(self.contacts)
        fut = self.client.send_goal_async(goal)
        w0 = time.time()
        while not fut.done() and time.time() - w0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.05)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            return {"goal": [x, y, yaw], "status": "REJECTED", "time": 0.0}
        t0 = self.now()
        rfut = gh.get_result_async()
        status = None
        w0 = time.time()
        while not rfut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.now() - t0 > timeout or time.time() - w0 > 3 * timeout:
                cfut = gh.cancel_goal_async()
                w1 = time.time()
                while not (cfut.done() and rfut.done()) and time.time() - w1 < 15.0:
                    rclpy.spin_once(self, timeout_sec=0.05)
                status = "TIMEOUT->CANCELED"
                break
        if status is None:
            status = STATUS.get(rfut.result().status, f"STATUS_{rfut.result().status}")
        t_end = self.now()
        elapsed = t_end - t0
        self.spin_sim(1.0)                      # stopped within 1 s?
        gx, gy, gyaw, v, w = self.gt
        return {
            "goal": [x, y, round(yaw, 3)], "status": status, "time": round(elapsed, 1),
            "pos_err": round(math.hypot(gx - x, gy - y), 3), "yaw_err": round(abs(wrap(gyaw - yaw)), 3),
            "final_truth": [round(gx, 3), round(gy, 3), round(gyaw, 3)],
            "speed_1s_after": [round(v, 3), round(w, 3)],
            "contacts": len(self.contacts) - n_contacts0,
        }
