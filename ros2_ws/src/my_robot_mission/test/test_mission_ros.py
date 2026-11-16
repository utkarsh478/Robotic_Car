"""The real mission_fsm node against fake ROS servers (PHASES P11 task 2): Nav2 navigate_to_pose
and spin, /pick_object, /place_object, AMCL (/amcl_pose, /request_nomotion_update, /initialpose),
the global costmap service, both detector nodes (`enabled` parameter + reset service) publishing
Detection messages in map, /gripper/attached, /joint_states, /camera/camera_info and TF
map->odom->base_footprint driven by /cmd_vel. Wall clock, about 40 s per test."""
import json
import math
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")
pytest.importorskip("my_robot_interfaces")

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist  # noqa: E402
from my_robot_interfaces.action import PickObject, PlaceObject  # noqa: E402
from my_robot_interfaces.msg import Detection  # noqa: E402
from nav2_msgs.action import NavigateToPose, Spin  # noqa: E402
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Empty, Trigger  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from my_robot_mission import core  # noqa: E402
from my_robot_mission.mission_fsm import MissionNode  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
ODOM_IN_MAP = (0.2, -0.1, 0.03)            # TF map->odom: the odom frame's pose in map
TARGET = (-1.6, -2.0)
BOX = (-3.0, -0.9)


def quat(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


class FakeRobot(Node):
    """Everything around the mission, in one node."""

    def __init__(self, pick_codes=()):
        super().__init__("fake_robot")
        cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.pose = [-3.0, -2.0, 0.0]                       # map
        self.v = self.w = 0.0
        self.t_cmd = 0.0
        self.target = list(TARGET)
        self.held = False
        self.pick_codes = list(pick_codes)
        self.picks, self.places = [], []
        self.cov = None
        self.count = {"target": 0, "box": 0}
        self.tf = TransformBroadcaster(self)
        self.amcl = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", LATCHED)
        self.attached = self.create_publisher(Bool, "/gripper/attached", LATCHED)
        self.attached.publish(Bool(data=False))
        self.js = self.create_publisher(JointState, "/joint_states", 10)
        self.cam = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        self.det = {lb: self.create_publisher(Detection, f"/perception/{lb}", 10) for lb in ("target", "box")}
        self.create_subscription(Twist, "/cmd_vel", self._cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initial, 10)
        self.create_service(Empty, "/request_nomotion_update", self._nomotion)
        self.create_service(GetCostmap, "/global_costmap/get_costmap", self._costmap)
        for n in ("/global_costmap/clear_entirely_global_costmap", "/local_costmap/clear_entirely_local_costmap"):
            self.create_service(ClearEntireCostmap, n, lambda req, resp: resp)
        ActionServer(self, NavigateToPose, "navigate_to_pose", self._nav, callback_group=cb,
                     cancel_callback=lambda g: CancelResponse.ACCEPT)
        ActionServer(self, Spin, "spin", self._spin, callback_group=cb, cancel_callback=lambda g: CancelResponse.ACCEPT)
        ActionServer(self, PickObject, "/pick_object", self._pick, callback_group=cb)
        ActionServer(self, PlaceObject, "/place_object", self._place, callback_group=cb)
        self.create_timer(0.02, self._step)
        self.create_timer(1.0 / 15, self._camera)

    # --------------------------------------------------------------- world
    def _cmd(self, m):
        with self.lock:
            self.v, self.w, self.t_cmd = m.linear.x, m.angular.z, time.time()

    def _step(self):
        with self.lock:
            if time.time() - self.t_cmd > 0.5:                # diff_drive cmd_vel_timeout
                self.v = self.w = 0.0
            x, y, a = self.pose
            self.pose = [x + self.v * math.cos(a) * 0.02, y + self.v * math.sin(a) * 0.02, core.wrap(a + self.w * 0.02)]
            ox, oy, oa = ODOM_IN_MAP                      # odom pose = inverse(map->odom) * map pose
            c, s = math.cos(oa), math.sin(oa)
            dx, dy = self.pose[0] - ox, self.pose[1] - oy
            odom = (c * dx + s * dy, -s * dx + c * dy, self.pose[2] - oa)
        now = self.get_clock().now().to_msg()
        out = []
        for parent, child, (x, y, a) in (("map", "odom", ODOM_IN_MAP), ("odom", "base_footprint", odom)):
            t = TransformStamped()
            t.header.stamp, t.header.frame_id, t.child_frame_id = now, parent, child
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation.z, t.transform.rotation.w = quat(a)
            out.append(t)
        self.tf.sendTransform(out)
        js = JointState(name=["left_finger_joint", "right_finger_joint"])
        js.header.stamp = now
        js.position = [0.023, 0.023] if self.held else [0.04, 0.04]
        self.js.publish(js)

    def visible(self, xy):
        bx, by = core.to_base(xy[0], xy[1], self.pose)
        cx, cy = bx - 0.20, by + 0.12
        return cx > 0.15 and math.hypot(cx, cy) < 2.7 and abs(math.atan2(cy, cx)) < 0.55

    def _camera(self):
        self.cam.publish(CameraInfo())
        for lb, xy, z in (("target", self.target, 0.075), ("box", BOX, 0.01)):
            node = DETECTORS.get(lb)
            if node is None or not node.get_parameter("enabled").value:
                continue
            if node.reset_flag:
                self.count[lb], node.reset_flag = 0, False
            with self.lock:
                seen = not (lb == "target" and self.held) and self.visible(xy)
            if seen:
                self.count[lb] += 1
            if self.count[lb] == 0:
                continue
            d = Detection(label=lb, confidence=1.0, position_stddev=0.004, observation_count=self.count[lb])
            d.header.frame_id = "map"
            d.header.stamp = self.get_clock().now().to_msg()
            d.pose.position.x, d.pose.position.y, d.pose.position.z = float(xy[0]), float(xy[1]), z
            d.pose.orientation.w = 1.0
            self.det[lb].publish(d)

    # --------------------------------------------------------------- AMCL
    def _initial(self, m):
        c = m.pose.covariance
        self.cov = [c[0], c[7], c[35]]
        self._publish_amcl()

    def _nomotion(self, req, resp):
        if self.cov is not None:
            self.cov = [v * 0.5 for v in self.cov]
            self._publish_amcl()
        return resp

    def _publish_amcl(self):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        c = [0.0] * 36
        c[0], c[7], c[35] = self.cov
        m.pose.covariance = c
        self.amcl.publish(m)

    # --------------------------------------------------------------- servers
    def _costmap(self, req, resp):
        md = resp.map.metadata
        md.resolution, md.size_x, md.size_y = 0.05, 160, 120
        md.origin.position.x, md.origin.position.y = -4.0, -3.0
        resp.map.data = [0] * (160 * 120)
        return resp

    def _nav(self, gh):
        p = gh.request.pose.pose
        time.sleep(0.3)
        if gh.is_cancel_requested:
            gh.canceled()
            return NavigateToPose.Result()
        with self.lock:
            self.pose = [p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)]
        gh.succeed()
        return NavigateToPose.Result()

    def _spin(self, gh):
        ang = gh.request.target_yaw
        steps = max(1, int(abs(ang) / 0.1))
        for _ in range(steps):
            if gh.is_cancel_requested:
                gh.canceled()
                return Spin.Result()
            with self.lock:
                self.pose[2] = core.wrap(self.pose[2] + ang / steps)
            time.sleep(0.02)
        gh.succeed()
        return Spin.Result()

    def _robot_frame(self, ps):
        """odom request -> map, and the point in the base frame"""
        ox, oy, oa = ODOM_IN_MAP
        x, y = ps.pose.position.x, ps.pose.position.y
        c, s = math.cos(oa), math.sin(oa)
        mx, my = c * x - s * y + ox, s * x + c * y + oy
        with self.lock:
            return (mx, my), core.to_base(mx, my, self.pose)

    def _pick(self, gh):
        (mx, my), (bx, by) = self._robot_frame(gh.request.object_pose)
        self.picks.append((mx, my, bx, by, gh.request.object_pose.header.frame_id))
        time.sleep(0.3)
        r = PickObject.Result()
        code = self.pick_codes.pop(0) if self.pick_codes else 0
        if code == 0 and (math.hypot(mx - self.target[0], my - self.target[1]) > 0.02 or not 0.36 <= bx <= 0.44):
            code = PickObject.Result.UNREACHABLE
        r.success, r.error_code, r.message = code == 0, code, "fake pick"
        if code == 0:
            self.held = True
            self.attached.publish(Bool(data=True))
            gh.succeed()
        else:
            gh.abort()
        return r

    def _place(self, gh):
        (mx, my), (bx, by) = self._robot_frame(gh.request.place_pose)
        self.places.append((mx, my, gh.request.place_pose.pose.position.z, bx, by))
        time.sleep(0.3)
        r = PlaceObject.Result()
        ok = math.hypot(mx - BOX[0], my - BOX[1]) < 0.02 and 0.45 <= bx <= 0.51
        r.success, r.error_code, r.message = ok, 0 if ok else PlaceObject.Result.UNREACHABLE, "fake place"
        if ok:
            self.held = False
            self.target = [mx, my]
            self.attached.publish(Bool(data=False))
            gh.succeed()
        else:
            gh.abort()
        return r


class FakeDetector(Node):
    def __init__(self, label):
        super().__init__(f"{label}_detector")
        self.declare_parameter("enabled", False)
        self.reset_flag = False
        self.create_service(Trigger, "~/reset", self._reset)

    def _reset(self, req, resp):
        self.reset_flag = True
        resp.success = True
        return resp


DETECTORS = {}


def run_mission(tmp_path, pick_codes=(), timeout=120.0):
    # own DDS domain: these fakes publish /cmd_vel etc. and must never reach a running simulation
    rclpy.init(domain_id=int(os.environ.get("MISSION_TEST_DOMAIN_ID", "97")))
    try:
        DETECTORS.clear()
        DETECTORS.update(target=FakeDetector("target"), box=FakeDetector("box"))
        robot = FakeRobot(pick_codes)
        params = [Parameter("auto_start", value=True), Parameter("trial_id", value="ros_test"),
                  Parameter("results_dir", value=str(tmp_path)),
                  # one waypoint, so the test does not drive the whole room
                  Parameter("search_waypoints", value=[-2.4, -2.0])]
        mission = MissionNode(parameter_overrides=params)
        ex = MultiThreadedExecutor(num_threads=6)
        for n in (robot, mission, *DETECTORS.values()):
            ex.add_node(n)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        t0 = time.time()
        while time.time() - t0 < timeout and mission.mission.state not in core.TERMINAL:
            time.sleep(0.1)
        state = mission.mission.state
        result = mission.mission.result
        ex.shutdown()
        return state, result, robot, mission
    finally:
        rclpy.try_shutdown()


def test_full_mission_against_fake_servers(tmp_path):
    state, result, robot, mission = run_mission(tmp_path)
    assert state == "MISSION_COMPLETE", mission.mission.reason
    assert result["success"] and result["trial_id"] == "ros_test"
    # pick: in odom, at the true target, from base-x 0.40
    mx, my, bx, by, frame = robot.picks[-1]
    assert frame == "odom" and math.hypot(mx - TARGET[0], my - TARGET[1]) < 0.01
    assert 0.39 <= bx <= 0.42 and abs(by) < 0.015
    px, py, pz, pbx, pby = robot.places[-1]
    assert math.hypot(px - BOX[0], py - BOX[1]) < 0.01 and pz == pytest.approx(0.105)
    assert 0.47 <= pbx <= 0.50
    # the target detector was left enabled for the release check, then everything switched off
    assert not DETECTORS["target"].get_parameter("enabled").value
    # CSV: header + one row per state exit
    with open(os.path.join(tmp_path, "mission_ros_test.csv")) as f:
        rows = f.read().strip().splitlines()
    assert rows[0] == "trial_id,state,t_enter,t_exit,outcome,retries,error_code,note"
    assert [r.split(",")[1] for r in rows[1:]] == ["INIT", "WAIT_FOR_START", "LOCALIZE", "SEARCH", "APPROACH_TARGET",
                                                   "REFINE_TARGET", "FINAL_ALIGN_TARGET", "PICK", "NAVIGATE_HOME",
                                                   "FIND_BOX", "APPROACH_BOX", "FINAL_ALIGN_BOX", "PLACE",
                                                   "STOW_AND_FINISH"]


def test_pick_failures_end_in_failed_with_the_reason(tmp_path):
    g = PickObject.Result.GRASP_NOT_DETECTED
    state, result, robot, mission = run_mission(tmp_path, pick_codes=[g, g])
    assert state == "FAILED" and result["reason"] == "PICK_GRASP_NOT_DETECTED"
    assert len(robot.picks) == 2 and not result["success"]
    assert json.loads(json.dumps(result))["states"][-1].startswith("PICK:failure")
