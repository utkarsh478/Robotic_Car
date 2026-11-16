"""Small rclpy client for move_group (Phase 7 gates): MoveGroup action, state validity,
planning-scene objects, joint states. Talks only to MoveIt's public interfaces."""
import os
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MotionPlanRequest,
                             MoveItErrorCodes, PlanningOptions, PlanningScene, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
ARM = ["arm_yaw_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint"]
FINGERS = ["left_finger_joint", "right_finger_joint"]
ARM_BASE = (0.05, 0.0, 0.11)              # arm_base_link in base_link
ARM_BASE_ABOVE_GROUND = 0.075 + 0.11
# rosidl puts message constants on the metaclass as properties
ERR = {getattr(MoveItErrorCodes, k): k for k in dir(type(MoveItErrorCodes)) if k.isupper()}


def err_name(code):
    return ERR.get(code, str(code))


def srdf_states(path=None):
    path = path or os.path.join(WS, "src", "my_robot_moveit_config", "config", "my_robot.srdf")
    txt = open(path).read()
    out = {}
    for name, group, body in re.findall(r'<group_state name="([^"]+)" group="([^"]+)">(.*?)</group_state>', txt, re.S):
        out[name] = (group, {j: float(v) for j, v in re.findall(r'<joint name="([^"]+)" value="([^"]+)"', body)})
    return out


def ik_cli_path():
    pre = subprocess.run(["ros2", "pkg", "prefix", "my_robot_manipulation"], capture_output=True, text=True).stdout.strip()
    return os.path.join(pre, "lib", "my_robot_manipulation", "ik_cli")


def ik(cli, x_base, y_base, h_ground, pitch=None):
    """tool0 target in base_link x/y and height above ground -> (ok, joints, text)."""
    args = [cli, "ik", f"{x_base - ARM_BASE[0]:.6f}", f"{y_base - ARM_BASE[1]:.6f}",
            f"{h_ground - ARM_BASE_ABOVE_GROUND:.6f}"]
    if pitch is not None:
        args.append(f"{pitch:.6f}")
    out = subprocess.run(args, capture_output=True, text=True).stdout.split()
    if out and out[0] == "OK":
        return True, [float(v) for v in out[1:5]], " ".join(out[5:])
    return False, None, " ".join(out)


class MoveItClient(Node):
    def __init__(self, name):
        super().__init__(name, parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.js = {}
        self.js_stamp = 0.0
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.validity = self.create_client(GetStateValidity, "/check_state_validity")
        self.apply_scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def _js(self, m):
        for n, p in zip(m.name, m.position):
            self.js[n] = p
        self.js_stamp = time.time()

    def spin_for(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.02)

    def sim_sleep(self, seconds):
        clk = self.get_clock()
        t0, w0 = clk.now().nanoseconds, time.time()
        while (clk.now().nanoseconds - t0) * 1e-9 < seconds and time.time() - w0 < 30:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_ready(self, timeout=120.0):
        w0 = time.time()
        while time.time() - w0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.move.server_is_ready() and self.validity.service_is_ready()
                    and self.apply_scene.service_is_ready() and all(j in self.js for j in ARM + FINGERS)):
                return time.time() - w0
        return None

    def _call(self, client, req, timeout=10.0):
        f = client.call_async(req)
        w0 = time.time()
        while not f.done() and time.time() - w0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
        return f.result() if f.done() else None

    def robot_state(self, joints):
        rs = RobotState()
        rs.is_diff = True
        rs.joint_state.name = list(joints)
        rs.joint_state.position = [float(v) for v in joints.values()]
        return rs

    def state_valid(self, joints, group="arm"):
        """-> (valid, [contact pairs]) for the CURRENT scene with `joints` overriding the state."""
        req = GetStateValidity.Request()
        req.robot_state = self.robot_state(joints)
        req.group_name = group
        r = self._call(self.validity, req)
        if r is None:
            return None, ["service timeout"]
        return r.valid, sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in r.contacts})

    def add_box(self, name, frame, size, xyz):
        co = CollisionObject()
        co.header.frame_id = frame
        co.id = name
        co.pose.orientation.w = 1.0          # Humble: object pose (all-zero quaternion is invalid)
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
        p = Pose()
        p.position.x, p.position.y, p.position.z = (float(v) for v in xyz)
        p.orientation.w = 1.0
        co.primitives = [prim]
        co.primitive_poses = [p]
        co.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [co]
        r = self._call(self.apply_scene, ApplyPlanningScene.Request(scene=scene))
        return bool(r and r.success)

    def go(self, group, joints, plan_only=False, start=None, vel=0.3, acc=0.3, time_s=2.0, attempts=2):
        """Plan (and execute) to a joint goal. -> dict(code, name, planning_time, wall, n_points)."""
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = attempts
        req.allowed_planning_time = time_s
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.pipeline_id = "ompl"
        if start is not None:
            req.start_state = self.robot_state(start)
        else:
            req.start_state.is_diff = True
        c = Constraints()
        for j, v in joints.items():
            c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=0.001,
                                                       tolerance_below=0.001, weight=1.0))
        req.goal_constraints = [c]
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=plan_only)
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        w0 = time.time()
        f = self.move.send_goal_async(goal)
        while not f.done() and time.time() - w0 < 10:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not f.done() or not f.result().accepted:
            return {"code": -999, "name": "GOAL_NOT_ACCEPTED", "planning_time": None, "wall": time.time() - w0}
        rf = f.result().get_result_async()
        while not rf.done() and time.time() - w0 < 90:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not rf.done():
            return {"code": -998, "name": "RESULT_TIMEOUT", "planning_time": None, "wall": time.time() - w0}
        res = rf.result().result
        return {"code": res.error_code.val, "name": err_name(res.error_code.val), "planning_time": res.planning_time,
                "wall": time.time() - w0, "n_points": len(res.planned_trajectory.joint_trajectory.points)}

    def joint_error(self, joints):
        return max(abs(self.js[j] - v) for j, v in joints.items())


def require(ok, msg):
    if not ok:
        print(f"FAIL  {msg}", flush=True)
        sys.exit(1)
