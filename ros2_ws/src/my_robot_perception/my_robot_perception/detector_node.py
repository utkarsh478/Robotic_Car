"""Shared ROS 2 node for the colour + depth detectors (SPEC §9).

RGB + depth (ApproximateTimeSynchronizer, slop 0.05 s) + camera_info -> pipeline.detect_*()
-> tf2 to `map` at the image stamp (0.1 s timeout; frame dropped if unavailable) -> tracker.

Publishes
  /perception/<label>              Detection, the track with most observations, <= publish_rate
  /perception/<label>_raw          Detection, every single-frame estimate (observation_count 1)
  /perception/markers              MarkerArray (namespace = label)
  /perception/<label>/debug_image  mask overlay (only when someone subscribes)
Service  ~/reset (std_srvs/Trigger): drop all tracks.
Param    enabled (bool): when false nothing is processed or published (the mission toggles it).
"""
import math

import message_filters
import numpy as np
import rclpy
import rclpy.executors
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
from my_robot_interfaces.msg import Detection
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from . import geometry as g
from . import vision as vis


def _pairs(flat):
    flat = list(flat)
    return [[flat[i], flat[i + 1]] for i in range(0, len(flat) - 1, 2)]


class DetectorNode(Node):
    def __init__(self, name, label, defaults, detect_fn, color, subscribe=True):
        super().__init__(name)
        self.label, self.detect_fn, self.color = label, detect_fn, color
        self.declare_parameter("enabled", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("depth_topic", "/camera/depth_image")
        self.declare_parameter("info_topic", "/camera/camera_info")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("tf_timeout", 0.1)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("track_gate", 0.15)
        self.declare_parameter("track_timeout", 10.0)
        self.declare_parameter("track_window", 30)
        self.defaults = dict(defaults)
        for k, v in defaults.items():
            if k == "hue_ranges":
                self.declare_parameter(k, [int(x) for pair in v for x in pair])
            else:
                self.declare_parameter(k, v)

        self.bridge = CvBridge()
        self.K = None
        # Image processing, reset and stats share one mutually exclusive group (no races on the
        # tracker). TF arrives in the listener's own reentrant group, so with the
        # MultiThreadedExecutor in run() /tf keeps flowing while a frame waits for its transform.
        # NOT TransformListener(spin_thread=True): that spins this same node in a second executor
        # thread, which crashes rclpy ("wait set index too big") and starves the lookup (DC-32).
        self.cb = MutuallyExclusiveCallbackGroup()
        self.tf = None
        self.tracker = g.Tracker(self.p("track_gate"), self.p("track_timeout"), self.p("track_window"))
        self.last_pub = -1e9
        self.stats = {"frames": 0, "tf_drop": 0, "detections": 0}
        self.last_tf_error = ""

        self.pub = self.create_publisher(Detection, f"/perception/{label}", 10)
        self.pub_raw = self.create_publisher(Detection, f"/perception/{label}_raw", 10)
        self.pub_mk = self.create_publisher(MarkerArray, "/perception/markers", 10)
        self.pub_dbg = self.create_publisher(Image, f"/perception/{label}/debug_image", 2)
        self.create_service(Trigger, "~/reset", self._reset, callback_group=self.cb)
        # Image streams are ~2 MB per RGB+depth pair at 15 Hz. Every extra subscriber is another
        # copy through DDS, and with best-effort transport large messages get dropped, so in
        # perception_node only ONE detector subscribes and hands each frame to its peers.
        self.peers = []
        self.rx = 0
        if subscribe:
            self.tf = Buffer()
            self._tfl = TransformListener(self.tf, self)
            self.create_subscription(CameraInfo, self.p("info_topic"), self._info, qos_profile_sensor_data)
            subs = [message_filters.Subscriber(self, Image, self.p(t), qos_profile=qos_profile_sensor_data,
                                               callback_group=self.cb)
                    for t in ("image_topic", "depth_topic")]
            self.sync = message_filters.ApproximateTimeSynchronizer(subs, 5, self.p("sync_slop"))
            self.sync.registerCallback(self._images)
        self.create_timer(10.0, self._log_stats, callback_group=self.cb)
        self.get_logger().info(f"{label} detector up (enabled={self.p('enabled')})")

    def p(self, name):
        return self.get_parameter(name).value

    def pipeline_params(self):
        out = {}
        for k in self.defaults:
            v = self.p(k)
            out[k] = _pairs(v) if k == "hue_ranges" else v
        return out

    def _info(self, msg):
        self.K = np.array(msg.k, float).reshape(3, 3)

    def _reset(self, req, resp):
        self.tracker.reset()
        self.last_pub = -1e9
        resp.success = True
        resp.message = f"{self.label} tracks cleared"
        return resp

    def _log_stats(self):
        s = self.stats
        rx = f"{self.rx} synced image pairs received, " if self.peers or hasattr(self, "sync") else ""
        self.get_logger().info(f"{self.label}: {rx}{s['frames']} frames processed, {s['detections']} detections, "
                               f"{s['tf_drop']} dropped (no TF) in the last 10 s"
                               + (f"; last TF error: {self.last_tf_error}" if s["tf_drop"] and self.last_tf_error else ""))
        self.stats = {k: 0 for k in s}
        self.rx = 0

    def _images(self, rgb_msg, depth_msg):
        """Synchronised pair received: one TF lookup and one decode, then this detector and its
        peers each run their pipeline on it."""
        self.rx += 1
        nodes = [n for n in [self] + self.peers if n.p("enabled")]
        if self.K is None or not nodes:
            return
        for n in nodes:
            n.stats["frames"] += 1
        stamp = Time.from_msg(rgb_msg.header.stamp)
        try:
            tr = self.tf.lookup_transform(self.p("map_frame"), rgb_msg.header.frame_id, stamp,
                                          timeout=Duration(seconds=self.p("tf_timeout")))
        except TransformException as e:
            for n in nodes:
                n.stats["tf_drop"] += 1
            self.last_tf_error = str(e)
            return
        q, t = tr.transform.rotation, tr.transform.translation
        T = g.quat_to_matrix(q.x, q.y, q.z, q.w, (t.x, t.y, t.z))
        frame = {}
        for n in nodes:
            n.process(rgb_msg, depth_msg, self.K, T, frame)

    def _decode(self, rgb_msg, depth_msg, frame):
        if "rgb" not in frame:
            frame["rgb"] = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            frame["depth"] = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
        return frame["rgb"], frame["depth"]

    def process(self, rgb_msg, depth_msg, K, T, frame):
        stamp = Time.from_msg(rgb_msg.header.stamp)
        rgb, depth = self._decode(rgb_msg, depth_msg, frame)
        res, info = self.detect_fn(rgb, depth, K, T, self.pipeline_params())
        ts = stamp.nanoseconds * 1e-9
        self.tracker.prune(ts)
        if res is not None:
            xy, yaw = (res if isinstance(res, tuple) else (res, None))
            self.stats["detections"] += 1
            self.tracker.update(xy, ts, extra=yaw)
            self.pub_raw.publish(self._detection(rgb_msg.header.stamp, xy, yaw, 1, float("nan"),
                                                 float(info.get("area", 0)) / 1e4))
        best = self.tracker.best()
        if best is not None and ts - self.last_pub >= 1.0 / self.p("publish_rate"):
            self.last_pub = ts
            std = g.Tracker.stddev(best)
            conf = min(1.0, best["count"] / 10.0) * (1.0 if (np.isfinite(std) and std < 0.03) else 0.5)
            self.pub.publish(self._detection(rgb_msg.header.stamp, best["mean"], best["extra"], best["count"], std, conf))
            self._markers(rgb_msg.header.stamp, best["mean"], best["count"])
        if self.pub_dbg.get_subscription_count() > 0:
            dbg = vis.overlay(rgb, info["mask"], (0, 255, 255) if res is None else (0, 255, 0))
            m = self.bridge.cv2_to_imgmsg(dbg, "rgb8")
            m.header = rgb_msg.header
            self.pub_dbg.publish(m)

    def _detection(self, stamp, xy, yaw, count, std, conf):
        d = Detection()
        d.header.stamp = stamp
        d.header.frame_id = self.p("map_frame")
        d.label = self.label
        d.pose.position.x, d.pose.position.y = float(xy[0]), float(xy[1])
        d.pose.position.z = float(self.p("mid_z") if "mid_z" in self.defaults else self.p("floor_z"))
        yaw = 0.0 if yaw is None else float(yaw)
        d.pose.orientation = Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
        d.confidence = float(conf)
        d.position_stddev = float(std)
        d.observation_count = int(count)
        return d

    def _markers(self, stamp, xy, count):
        ma = MarkerArray()
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp, self.p("map_frame")
        m.ns, m.id, m.action = self.label, 0, Marker.ADD
        if self.label == "box":
            m.type = Marker.CUBE
            m.scale.x = m.scale.y = 0.30
            m.scale.z = 0.15
            m.pose.position.z = 0.075
        else:
            m.type = Marker.CYLINDER
            m.scale.x = m.scale.y = 0.05
            m.scale.z = 0.15
            m.pose.position.z = 0.075
        m.pose.position.x, m.pose.position.y = float(xy[0]), float(xy[1])
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = self.color
        m.color.a = 0.6
        m.lifetime = Duration(seconds=1.0).to_msg()
        t = Marker()
        t.header, t.ns, t.id, t.type = m.header, self.label, 1, Marker.TEXT_VIEW_FACING
        t.pose.position.x, t.pose.position.y, t.pose.position.z = float(xy[0]), float(xy[1]), 0.35
        t.pose.orientation.w = 1.0
        t.scale.z = 0.08
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text = f"{self.label} n={count}"
        t.lifetime = m.lifetime
        ma.markers = [m, t]
        self.pub_mk.publish(ma)


def run(*node_factories):
    """Spin one or more detector nodes in one process. The first one subscribes to the camera
    and feeds the others (one copy of the image streams instead of one per detector)."""
    rclpy.init()
    nodes = [f(i == 0) for i, f in enumerate(node_factories)]
    nodes[0].peers = nodes[1:]
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0

