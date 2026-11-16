#!/usr/bin/env python3
"""GATE-ONLY stand-in for AMCL: publishes map->odom from ground truth so that a gate can
teleport the robot (which odometry cannot know about) and still have a correct map frame.

  map->odom = T_world_base(ground truth) * inverse(T_odom_base(diff_drive))     (map == world, DC-9)

Planar (x, y, yaw). Reads /evaluator/robot_odom, so it must never be part of the autonomy stack;
it lives in tools/gates and P11's isolation check keeps it out of src/.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class GtLocalizer(Node):
    def __init__(self):
        super().__init__("gt_localizer", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.gt = None
        self.create_subscription(Odometry, "/evaluator/robot_odom", self._gt, 10)
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self.br = TransformBroadcaster(self)
        self.create_timer(1.0 / 30.0, self._tick)

    def _gt(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.gt = (p.x, p.y, yaw_of(q))

    def _tick(self):
        if self.gt is None:
            return
        try:
            t = self.tf.lookup_transform("odom", "base_footprint", Time())
        except TransformException:
            return
        ox, oy, oyaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        gx, gy, gyaw = self.gt
        yaw = gyaw - oyaw                     # map->odom rotation
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = gx - (c * ox - s * oy), gy - (s * ox + c * oy)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = "map", "odom"
        msg.transform.translation.x, msg.transform.translation.y = tx, ty
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.br.sendTransform(msg)


def main():
    rclpy.init()
    n = GtLocalizer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
