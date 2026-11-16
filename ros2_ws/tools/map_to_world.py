#!/usr/bin/env python3
"""Shift a slam_toolbox map so its map frame equals the Gazebo world frame.

slam_toolbox starts the map frame at the robot's first pose. The robot spawns at home
(-3.0, -2.0, yaw 0), so map = world - (-3, -2). Because the yaw is 0, adding the spawn
offset to the map origin makes map == world. Nav2/AMCL then use SPEC world coordinates
directly (home, waypoints, targets).

usage: map_to_world.py RAW.yaml OUT.yaml [--dx -3.0] [--dy -2.0]
Writes OUT.yaml and copies the image next to it as OUT.pgm.
"""
import argparse
import os
import shutil

import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raw_yaml")
    ap.add_argument("out_yaml")
    ap.add_argument("--dx", type=float, default=-3.0)
    ap.add_argument("--dy", type=float, default=-2.0)
    a = ap.parse_args()

    with open(a.raw_yaml) as f:
        m = yaml.safe_load(f)
    raw_img = m["image"] if os.path.isabs(m["image"]) else os.path.join(os.path.dirname(a.raw_yaml), m["image"])
    out_img = os.path.splitext(a.out_yaml)[0] + ".pgm"
    shutil.copyfile(raw_img, out_img)

    ox, oy, oyaw = m["origin"]
    if abs(oyaw) > 1e-9:
        raise SystemExit(f"origin yaw {oyaw} != 0: a pure translation is not enough")
    m["origin"] = [round(ox + a.dx, 6), round(oy + a.dy, 6), 0.0]
    m["image"] = os.path.basename(out_img)
    with open(a.out_yaml, "w") as f:
        yaml.safe_dump(m, f, default_flow_style=None, sort_keys=False)
    print(f"origin ({ox:.3f}, {oy:.3f}) -> ({m['origin'][0]:.3f}, {m['origin'][1]:.3f}); wrote {a.out_yaml}")


if __name__ == "__main__":
    main()
