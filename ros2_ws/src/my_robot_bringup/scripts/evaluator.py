#!/usr/bin/env python3
"""Ground-truth evaluator (SPEC §13). Phase-3 version: collects true poses only.

Subscribes ONLY to /evaluator/* (bridged Gazebo ground truth) and publishes
/evaluator/summary (std_msgs/String, JSON, 1 Hz):
  {"t": sim_s, "robot": [x, y, yaw] | null, "target": [x, y, z] | null,
   "distractor": [x, y, z] | null}
Later phases add trial scoring (placement success, detection/nav error, collisions)
and write results/trial_<timestamp>.json.

CLAUDE.md rule 4: no autonomy node may subscribe to anything this node reads or publishes.
"""
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Evaluator(Node):
    def __init__(self):
        super().__init__("evaluator")
        self.poses = {"robot": None, "target": None, "distractor": None}
        for key in self.poses:
            self.create_subscription(
                Odometry, f"/evaluator/{key}_odom",
                lambda msg, k=key: self._on_pose(k, msg), qos_profile_sensor_data)
        self.pub = self.create_publisher(String, "/evaluator/summary", 10)
        self.create_timer(1.0, self._publish)

    def _on_pose(self, key, msg):
        p = msg.pose.pose
        if key == "robot":
            self.poses[key] = [p.position.x, p.position.y, yaw_of(p.orientation)]
        else:
            self.poses[key] = [p.position.x, p.position.y, p.position.z]

    def _publish(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        out = {"t": round(t, 3)}
        for k, v in self.poses.items():
            out[k] = None if v is None else [round(c, 4) for c in v]
        self.pub.publish(String(data=json.dumps(out)))


def main():
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
