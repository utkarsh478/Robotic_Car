#!/usr/bin/env python3
"""Phase-1 TF gate: every base/sensor frame resolves from base_footprint,
and base_footprint -> camera_optical_link is (0.20, -0.12, 0.50) +/- 0.001.
Run while display.launch.xml is up. Exit 0 = PASS."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

FRAMES = ["base_link", "left_wheel_link", "right_wheel_link", "front_caster_link",
          "rear_caster_link", "lidar_link", "imu_link", "camera_mast_link",
          "camera_link", "camera_optical_link"]
EXPECT_CAM = (0.20, -0.12, 0.50)


def main():
    rclpy.init()
    node = Node("tf_check_p01")
    buf = Buffer()
    _listener = TransformListener(buf, node)  # keep a reference or it is garbage-collected

    deadline = time.time() + 10.0
    missing = FRAMES
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        missing = [f for f in FRAMES if not buf.can_transform("base_footprint", f, Time())]
        if not missing:
            break

    ok = True
    if missing:
        print(f"FAIL  frames not reachable from base_footprint: {missing}")
        ok = False
    else:
        print(f"PASS  all {len(FRAMES)} frames reachable from base_footprint")

    if "camera_optical_link" not in missing:
        t = buf.lookup_transform("base_footprint", "camera_optical_link", Time()).transform.translation
        got = (t.x, t.y, t.z)
        err = max(abs(a - b) for a, b in zip(got, EXPECT_CAM))
        tag = "PASS" if err <= 1e-3 else "FAIL"
        ok &= err <= 1e-3
        print(f"{tag}  base_footprint->camera_optical_link = ({got[0]:.4f}, {got[1]:.4f}, {got[2]:.4f}), "
              f"max err {err * 1000:.2f} mm")

    # The tree must have exactly one root (no disconnected sub-trees).
    yaml = buf.all_frames_as_yaml()
    parents = {}
    cur = None
    for line in yaml.splitlines():
        line = line.rstrip()
        if line and not line.startswith(" ") and line.endswith(":"):
            cur = line[:-1].strip()
        elif cur and "parent:" in line:
            parents[cur] = line.split("parent:")[1].strip().strip("'\"")
    roots = sorted(set(parents.values()) - set(parents))
    tag = "PASS" if roots == ["base_footprint"] else "FAIL"
    ok &= roots == ["base_footprint"]
    print(f"{tag}  TF roots = {roots} (expect ['base_footprint'])")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
