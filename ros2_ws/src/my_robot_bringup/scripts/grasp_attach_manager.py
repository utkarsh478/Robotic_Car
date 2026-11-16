#!/usr/bin/env python3
"""Owns the simulated grasp attach (Gazebo DetachableJoint on the robot, SPEC §3.4, DC-19).

Why this exists: Gazebo Fortress 6.18's DetachableJoint creates the fixed joint
(gripper <-> target_cylinder) the moment both models exist; the "start detached" option
(initial_attach) only appears after 6.18. Left alone, the target would be welded to the
gripper from t = 0, wherever it is in the world.

Behaviour:
  * desired state starts DETACHED.
  * Gazebo reports the real state on /gripper/attach_state ("attached"/"detached"), but only
    when it changes, so at startup the state is unknown.
  * Every 0.25 s: while the reported state differs from the desired one (or is unknown),
    publish the corrective command on /gripper/attach or /gripper/detach. Gazebo ignores a
    detach when already detached and an attach when already attached, so repeating is safe.
  * Service /gripper/set_attached (std_srvs/SetBool): data=true attaches, false detaches.
    Returns once Gazebo confirms the new state (timeout 3 s). Phase 9 calls it only after
    sensor evidence of a grasp (SPEC §11.5): never trust the command alone.
  * Publishes /gripper/attached (std_msgs/Bool, latched) = Gazebo-confirmed state.
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool


class GraspAttachManager(Node):
    def __init__(self):
        super().__init__("grasp_attach_manager")
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.desired = False
        self.reported = None            # None = unknown, True = attached, False = detached
        self.commands_sent = 0
        self.pub_attach = self.create_publisher(Empty, "/gripper/attach", 10)
        self.pub_detach = self.create_publisher(Empty, "/gripper/detach", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(Bool, "/gripper/attached", latched)
        self.create_subscription(String, "/gripper/attach_state", self._on_state, 10, callback_group=self.cb)
        self.create_service(SetBool, "/gripper/set_attached", self._on_set, callback_group=self.cb)
        self.create_timer(0.25, self._enforce, callback_group=self.cb)
        self.get_logger().info("grasp attach manager up: desired DETACHED")

    def _on_state(self, msg):
        attached = msg.data.strip().lower() == "attached"
        with self.lock:
            changed = attached != self.reported
            self.reported = attached
        if changed:
            self.get_logger().info(f"Gazebo reports: {'ATTACHED' if attached else 'DETACHED'}"
                                   f" (desired {'ATTACHED' if self.desired else 'DETACHED'})")
            self.pub_state.publish(Bool(data=attached))

    def _enforce(self):
        with self.lock:
            mismatch = self.reported is None or self.reported != self.desired
            desired = self.desired
        if mismatch:
            (self.pub_attach if desired else self.pub_detach).publish(Empty())
            self.commands_sent += 1

    def _on_set(self, req, res):
        with self.lock:
            self.desired = bool(req.data)
        want = "ATTACHED" if req.data else "DETACHED"
        t0 = time.time()
        while time.time() - t0 < 3.0:
            with self.lock:
                if self.reported == self.desired:
                    res.success, res.message = True, f"{want} (confirmed by Gazebo)"
                    return res
            time.sleep(0.05)
        res.success, res.message = False, f"{want} requested but Gazebo did not confirm within 3 s"
        return res


def main():
    rclpy.init()
    node = GraspAttachManager()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
