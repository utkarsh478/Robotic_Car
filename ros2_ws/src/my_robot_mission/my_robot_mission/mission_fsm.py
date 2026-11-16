#!/usr/bin/env python3
"""mission_fsm: the ROS side of the mission (SPEC §12). The state machine itself is core.Mission.

Every call here is non-blocking: actions and services are sent with *_async and polled by the
10 Hz timer through small Op objects, so no callback ever waits on another (SPEC §12).

Interfaces
  actions   navigate_to_pose, spin (Nav2), /pick_object, /place_object
  services  /mission/start (Trigger); uses /{target,box}_detector/{set_parameters,reset},
            /request_nomotion_update (AMCL), /global_costmap/get_costmap,
            /{global,local}_costmap/clear_entirely_{global,local}_costmap
  topics    pub /cmd_vel (Twist, DC-2), /initialpose, /mission/state, /mission/result (latched)
            sub /perception/target, /perception/box, /amcl_pose, /gripper/attached,
                /joint_states, /camera/camera_info
  CSV       <results_dir>/mission_<trial_id>.csv, one row per state exit
Reads no ground truth and no simulator topic (CLAUDE.md rule 4; tools/gates/check_isolation.sh).
"""
import csv
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from my_robot_interfaces.action import PickObject, PlaceObject
from my_robot_interfaces.msg import Detection
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_mission import core

STATUS = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
          GoalStatus.STATUS_CANCELED: "CANCELED"}
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
CSV_FIELDS = ["trial_id", "state", "t_enter", "t_exit", "outcome", "retries", "error_code", "note"]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Op:
    """One asynchronous action goal or service call, polled by the state machine."""

    def __init__(self, kind):
        self.kind = kind
        self.done = False
        self.ok = False
        self.code = None
        self.status = "PENDING"
        self.message = ""
        self.result = None
        self._goal = None
        self._cancel_requested = False

    def finish(self, ok, status, code=None, message="", result=None):
        if self.done:
            return
        self.ok, self.status, self.code, self.message, self.result = bool(ok), status, code, message, result
        self.done = True

    def cancel(self):
        self._cancel_requested = True
        if self._goal is not None and not self.done:
            self._goal.cancel_goal_async()
        self.finish(False, "CANCELED")


class ActionOp(Op):
    def __init__(self, kind, client, goal, judge, feedback_cb=None):
        super().__init__(kind)
        self.judge = judge
        if not client.server_is_ready():
            self.finish(False, "NO_SERVER")
            return
        fut = client.send_goal_async(goal, feedback_callback=feedback_cb)
        fut.add_done_callback(self._accepted)

    def _accepted(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.finish(False, "REJECTED")
            return
        self._goal = gh
        if self._cancel_requested:
            gh.cancel_goal_async()
            return
        gh.get_result_async().add_done_callback(self._result)

    def _result(self, fut):
        res = fut.result()
        status = STATUS.get(res.status, f"STATUS_{res.status}")
        ok, code, msg = self.judge(status, res.result)
        self.finish(ok, status, code, msg, res.result)


class ServiceOp(Op):
    def __init__(self, kind, client, request, convert):
        super().__init__(kind)
        if not client.service_is_ready():
            self.finish(False, "NO_SERVICE")
            return
        self.convert = convert
        client.call_async(request).add_done_callback(self._done)

    def _done(self, fut):
        try:
            res = fut.result()
            self.finish(True, "SUCCEEDED", result=self.convert(res))
        except Exception as e:  # noqa: BLE001
            self.finish(False, "ERROR", message=str(e))


def judge_nav(status, _res):
    return status == "SUCCEEDED", None, status


def judge_manip(status, res):
    if res is None:
        return False, None, status
    return bool(res.success) and status == "SUCCEEDED", int(res.error_code), res.message


class MissionNode(Node):
    def __init__(self, **kw):
        super().__init__("mission_fsm", **kw)
        dyn = ParameterDescriptor(dynamic_typing=True)     # launch may pass "7" -> int; keep it a name
        for k, v in core.DEFAULTS.items():
            self.declare_parameter(k, v, dyn if k == "trial_id" else ParameterDescriptor())
        self.declare_parameter("results_dir", "auto", dyn)
        P = {k: self.get_parameter(k).value for k in core.DEFAULTS}
        P["trial_id"] = str(P["trial_id"])
        if P["trial_id"] in ("", "auto"):
            P["trial_id"] = time.strftime("%Y%m%d_%H%M%S")
        self.trial_id = P["trial_id"]

        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "spin")
        self.pick_client = ActionClient(self, PickObject, "/pick_object")
        self.place_client = ActionClient(self, PlaceObject, "/place_object")
        self.det_param = {lb: self.create_client(SetParameters, f"/{lb}_detector/set_parameters")
                          for lb in ("target", "box")}
        self.det_reset = {lb: self.create_client(Trigger, f"/{lb}_detector/reset") for lb in ("target", "box")}
        self.nomo = self.create_client(Empty, "/request_nomotion_update")
        self.get_costmap = self.create_client(GetCostmap, "/global_costmap/get_costmap")
        self.clear = [self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"),
                      self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")]

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.state_pub = self.create_publisher(String, "/mission/state", LATCHED)
        self.result_pub = self.create_publisher(String, "/mission/result", LATCHED)

        self.tracks = {"target": None, "box": None}
        self.create_subscription(Detection, "/perception/target", lambda m: self._track("target", m), 10)
        self.create_subscription(Detection, "/perception/box", lambda m: self._track("box", m), 10)
        self.cov = None
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl, LATCHED)
        self.attached = None
        self.create_subscription(Bool, "/gripper/attached", self._attached, LATCHED)
        self.fingers = {}
        self.create_subscription(JointState, "/joint_states", self._joints, 20)
        self.camera_ok = False
        self.create_subscription(CameraInfo, "/camera/camera_info", self._camera, qos_profile_sensor_data)

        self.ops = []
        self.csv_path = self._csv_open(self.get_parameter("results_dir").value)
        self.create_service(Trigger, "/mission/start", self._start)
        self.mission = core.Mission(self, P)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"mission_fsm up: trial {self.trial_id}, auto_start {P['auto_start']}, "
                               f"CSV {self.csv_path}")

    # ------------------------------------------------------------- callbacks
    def _track(self, label, m):
        if m.header.frame_id != "map":
            self.get_logger().warn(f"{label} detection in frame '{m.header.frame_id}', expected map",
                                   throttle_duration_sec=10.0)
            return
        p = m.pose.position
        self.tracks[label] = core.Track(p.x, p.y, p.z, yaw_of(m.pose.orientation), int(m.observation_count),
                                        float(m.position_stddev))

    def _amcl(self, m):
        c = m.pose.covariance
        self.cov = (c[0], c[7], c[35], self.sim_now())

    def _attached(self, m):
        self.attached = bool(m.data)

    def _joints(self, m):
        for n, q in zip(m.name, m.position):
            if n in ("left_finger_joint", "right_finger_joint"):
                self.fingers[n] = q

    def _camera(self, _m):
        self.camera_ok = True

    def _start(self, _req, resp):
        resp.success, resp.message = self.mission.request_start()
        return resp

    def _tick(self):
        now = self.sim_now()
        if now <= 0.0:                              # sim time: no /clock yet
            return
        self.mission.tick(now)

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- IO protocol (core.Mission)
    def ready(self):
        missing = []
        for name, c in (("navigate_to_pose", self.nav_client), ("spin", self.spin_client),
                        ("/pick_object", self.pick_client), ("/place_object", self.place_client)):
            if not c.server_is_ready():
                missing.append(name)
        for lb in ("target", "box"):
            if not self.det_param[lb].service_is_ready():
                missing.append(f"/{lb}_detector")
        if not self.camera_ok:
            missing.append("/camera/camera_info")
        if self.robot_pose("map") is None:
            missing.append("TF map->base_footprint")
        return missing

    def robot_pose(self, frame):
        try:
            tr = self.tf.lookup_transform(frame, "base_footprint", Time()).transform
        except TransformException:
            return None
        return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

    def map_to_odom(self, x, y, yaw):
        try:
            tr = self.tf.lookup_transform("odom", "map", Time()).transform
        except TransformException:
            return None
        a = yaw_of(tr.rotation)
        c, s = math.cos(a), math.sin(a)
        return (tr.translation.x + c * x - s * y, tr.translation.y + s * x + c * y, core.wrap(yaw + a))

    def track(self, label):
        return self.tracks[label]

    def detector(self, label, enabled):
        req = SetParameters.Request()
        req.parameters = [ParamMsg(name="enabled", value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                                        bool_value=bool(enabled)))]
        if self.det_param[label].service_is_ready():
            self.det_param[label].call_async(req)
        else:
            self.get_logger().warn(f"/{label}_detector/set_parameters not available")

    def reset_tracks(self, label):
        self.tracks[label] = None
        if self.det_reset[label].service_is_ready():
            self.det_reset[label].call_async(Trigger.Request())

    def _pose_stamped(self, frame, x, y, z=0.0, yaw=0.0):
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return ps

    def _keep(self, op):
        self.ops = [o for o in self.ops if not o.done] + [op]
        return op

    def nav(self, x, y, yaw):
        g = NavigateToPose.Goal()
        g.pose = self._pose_stamped("map", x, y, 0.0, yaw)
        return self._keep(ActionOp("nav", self.nav_client, g, judge_nav))

    def spin(self, angle):
        g = Spin.Goal()
        g.target_yaw = float(angle)
        g.time_allowance = Duration(seconds=20.0).to_msg()
        return self._keep(ActionOp("spin", self.spin_client, g, judge_nav))

    def pick(self, x, y, z):
        g = PickObject.Goal()
        g.object_pose = self._pose_stamped("odom", x, y, z)
        g.object_diameter = float(self.mission.p("target.diameter"))
        g.object_height = float(self.mission.p("target.height"))
        return self._keep(ActionOp("pick", self.pick_client, g, judge_manip))

    def place(self, x, y, z, yaw):
        g = PlaceObject.Goal()
        g.place_pose = self._pose_stamped("odom", x, y, z, yaw)
        return self._keep(ActionOp("place", self.place_client, g, judge_manip))

    def costmap(self):
        def convert(res):
            m = res.map
            md = m.metadata
            return core.Grid(md.resolution, md.origin.position.x, md.origin.position.y,
                             md.size_x, md.size_y, bytes(m.data))
        return self._keep(ServiceOp("costmap", self.get_costmap, GetCostmap.Request(), convert))

    def clear_costmaps(self):
        for c in self.clear:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())

    def cmd(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)

    def cancel_all(self):
        for o in self.ops:
            if not o.done:
                o.cancel()
        self.ops = []

    def initial_pose(self, x, y, yaw, cov):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose = self._pose_stamped("map", x, y, 0.0, yaw).pose
        c = [0.0] * 36
        c[0], c[7], c[35] = float(cov[0]), float(cov[1]), float(cov[2])
        m.pose.covariance = c
        self.init_pub.publish(m)

    def nomotion(self):
        if self.nomo.service_is_ready():
            self.nomo.call_async(Empty.Request())
        else:
            self.get_logger().warn("AMCL /request_nomotion_update not available: LOCALIZE will spin instead",
                                   throttle_duration_sec=30.0)

    def amcl_cov(self):
        return self.cov

    def grasp(self):
        q = None
        if len(self.fingers) == 2:
            q = (self.fingers["left_finger_joint"], self.fingers["right_finger_joint"])
        return self.attached, q

    def publish_state(self, name):
        self.state_pub.publish(String(data=name))

    def publish_result(self, result):
        self.result_pub.publish(String(data=json.dumps(result)))
        self.get_logger().info(f"mission result: {json.dumps(result)}")

    def _csv_open(self, results_dir):
        results_dir = str(results_dir)
        if results_dir in ("", "auto"):
            results_dir = os.path.join(os.getcwd(), "results")
        os.makedirs(results_dir, exist_ok=True)
        path = os.path.join(results_dir, f"mission_{self.trial_id}.csv")
        with open(path, "w", newline="") as f:
            csv.DictWriter(f, CSV_FIELDS).writeheader()
        return path

    def csv(self, row):
        row = dict(row, trial_id=self.trial_id)
        with open(self.csv_path, "a", newline="") as f:
            csv.DictWriter(f, CSV_FIELDS).writerow(row)

    def log(self, level, text):
        # one call site per severity: rclpy refuses a call site whose severity changes
        if level == "error":
            self.get_logger().error(text)
        elif level == "warn":
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd(0.0, 0.0)
        except Exception:  # noqa: BLE001 - context may already be shut down by the launch's SIGINT
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
