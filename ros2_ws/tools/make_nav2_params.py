#!/usr/bin/env python3
"""Generate src/my_robot_navigation/config/nav2_params.yaml from the INSTALLED Nav2 default
(CLAUDE.md rule 3: never write Nav2 params from scratch).

  default = $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml
  output  = default + the OVERRIDES below (SPEC §10 + DESIGN_CHANGES)

Every override names its full path. If a node/section on that path does not exist in the
installed default the script stops (wrong node name = silently ignored params otherwise).
A leaf key that the default does not list is added and reported as ADD.
usage: make_nav2_params.py [--default FILE] [--out FILE] [--check]
  --check : exit 1 if the committed output differs from what would be generated now.
"""
import argparse
import copy
import os
import sys

import yaml

WS = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
OUT = os.path.join(WS, "src", "my_robot_navigation", "config", "nav2_params.yaml")

# Robot footprint (DESIGN_CHANGES DC-8 + open item): lidar body reaches x 0.305, wheel
# collision spheres reach |y| 0.265, chassis rear at x -0.25 (+2 cm margin).
FOOTPRINT = "[[0.31, 0.27], [0.31, -0.27], [-0.27, -0.27], [-0.27, 0.27]]"

SCAN_SOURCE = {
    "topic": "/scan",
    "max_obstacle_height": 2.0,
    "clearing": True,
    "marking": True,
    "data_type": "LaserScan",
    "raytrace_max_range": 6.0,
    "raytrace_min_range": 0.0,
    "obstacle_max_range": 5.0,
    "obstacle_min_range": 0.0,
}
OBSTACLE_LAYER = {
    "plugin": "nav2_costmap_2d::ObstacleLayer",
    "enabled": True,
    "observation_sources": "scan",
    "scan": SCAN_SOURCE,
}
INFLATION = {
    "plugin": "nav2_costmap_2d::InflationLayer",
    "cost_scaling_factor": 3.0,
    "inflation_radius": 0.45,
}
RPP = {  # SPEC §10: Regulated Pure Pursuit, no reversing (rear is lidar-blind)
    "plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "desired_linear_vel": 0.3,
    "lookahead_dist": 0.6,
    "min_lookahead_dist": 0.3,
    "max_lookahead_dist": 0.9,
    "lookahead_time": 1.5,
    "rotate_to_heading_angular_vel": 1.0,
    "transform_tolerance": 0.1,
    "use_velocity_scaled_lookahead_dist": False,
    "min_approach_linear_velocity": 0.05,
    "approach_velocity_scaling_dist": 0.6,
    "use_collision_detection": True,
    "max_allowed_time_to_collision_up_to_carrot": 1.0,
    "use_regulated_linear_velocity_scaling": True,
    "use_cost_regulated_linear_velocity_scaling": False,
    "regulated_linear_scaling_min_radius": 0.9,
    "regulated_linear_scaling_min_speed": 0.25,
    "use_rotate_to_heading": True,
    "allow_reversing": False,
    "rotate_to_heading_min_angle": 0.785,
    "max_angular_accel": 2.0,
    "max_robot_pose_search_dist": 10.0,
}

# (path, value)   value None = delete the key
OVERRIDES = [
    # ---------------- AMCL ----------------
    (["amcl", "ros__parameters", "base_frame_id"], "base_footprint"),
    (["amcl", "ros__parameters", "global_frame_id"], "map"),
    (["amcl", "ros__parameters", "odom_frame_id"], "odom"),
    (["amcl", "ros__parameters", "scan_topic"], "scan"),
    (["amcl", "ros__parameters", "laser_model_type"], "likelihood_field"),
    (["amcl", "ros__parameters", "laser_max_range"], 12.0),
    (["amcl", "ros__parameters", "max_beams"], 120),   # every 3rd of 361 rays (1.5 deg)
    (["amcl", "ros__parameters", "robot_model_type"], "nav2_amcl::DifferentialMotionModel"),
    # DC-18: defaults assume noisy real odometry and a loose laser model. gate_p04 run 1:
    # AMCL vs truth mean 0.098 m, max 0.253 m / 0.149 rad although the map is within
    # 4.5 cm (P3) and sim odometry is near-exact (P2). Tighten to match the simulation.
    (["amcl", "ros__parameters", "alpha1"], 0.05),   # rot noise from rot
    (["amcl", "ros__parameters", "alpha2"], 0.05),   # rot noise from trans
    (["amcl", "ros__parameters", "alpha3"], 0.05),   # trans noise from trans
    (["amcl", "ros__parameters", "alpha4"], 0.05),   # trans noise from rot
    (["amcl", "ros__parameters", "sigma_hit"], 0.08),   # lidar noise 0.01 m + map cell 0.05 m
    (["amcl", "ros__parameters", "z_hit"], 0.9),
    (["amcl", "ros__parameters", "z_rand"], 0.05),
    (["amcl", "ros__parameters", "z_max"], 0.05),
    (["amcl", "ros__parameters", "update_min_d"], 0.10),
    (["amcl", "ros__parameters", "update_min_a"], 0.10),
    # map frame == Gazebo world frame (DC-9): start AMCL at the home pose
    (["amcl", "ros__parameters", "set_initial_pose"], True),
    (["amcl", "ros__parameters", "initial_pose"], {"x": -3.0, "y": -2.0, "z": 0.0, "yaw": 0.0}),
    # ---------------- BT navigator ----------------
    (["bt_navigator", "ros__parameters", "robot_base_frame"], "base_footprint"),
    (["bt_navigator", "ros__parameters", "odom_topic"], "/odom"),
    # ---------------- controller ----------------
    (["controller_server", "ros__parameters", "controller_frequency"], 20.0),
    (["controller_server", "ros__parameters", "general_goal_checker", "xy_goal_tolerance"], 0.10),
    (["controller_server", "ros__parameters", "general_goal_checker", "yaw_goal_tolerance"], 0.10),
    (["controller_server", "ros__parameters", "FollowPath"], RPP),
    # ---------------- local costmap ----------------
    (["local_costmap", "local_costmap", "ros__parameters", "robot_base_frame"], "base_footprint"),
    (["local_costmap", "local_costmap", "ros__parameters", "global_frame"], "odom"),
    (["local_costmap", "local_costmap", "ros__parameters", "rolling_window"], True),
    (["local_costmap", "local_costmap", "ros__parameters", "width"], 3),
    (["local_costmap", "local_costmap", "ros__parameters", "height"], 3),
    (["local_costmap", "local_costmap", "ros__parameters", "resolution"], 0.05),
    (["local_costmap", "local_costmap", "ros__parameters", "robot_radius"], None),
    (["local_costmap", "local_costmap", "ros__parameters", "footprint"], FOOTPRINT),
    (["local_costmap", "local_costmap", "ros__parameters", "plugins"], ["obstacle_layer", "inflation_layer"]),
    (["local_costmap", "local_costmap", "ros__parameters", "voxel_layer"], None),
    (["local_costmap", "local_costmap", "ros__parameters", "obstacle_layer"], OBSTACLE_LAYER),
    (["local_costmap", "local_costmap", "ros__parameters", "inflation_layer"], INFLATION),
    # ---------------- global costmap ----------------
    (["global_costmap", "global_costmap", "ros__parameters", "robot_base_frame"], "base_footprint"),
    (["global_costmap", "global_costmap", "ros__parameters", "global_frame"], "map"),
    (["global_costmap", "global_costmap", "ros__parameters", "resolution"], 0.05),
    (["global_costmap", "global_costmap", "ros__parameters", "track_unknown_space"], True),
    (["global_costmap", "global_costmap", "ros__parameters", "robot_radius"], None),
    (["global_costmap", "global_costmap", "ros__parameters", "footprint"], FOOTPRINT),
    (["global_costmap", "global_costmap", "ros__parameters", "plugins"],
     ["static_layer", "obstacle_layer", "inflation_layer"]),
    (["global_costmap", "global_costmap", "ros__parameters", "obstacle_layer"], OBSTACLE_LAYER),
    (["global_costmap", "global_costmap", "ros__parameters", "inflation_layer"], INFLATION),
    # ---------------- planner ----------------
    (["planner_server", "ros__parameters", "GridBased", "use_astar"], True),
    (["planner_server", "ros__parameters", "GridBased", "allow_unknown"], True),
    (["planner_server", "ros__parameters", "GridBased", "tolerance"], 0.3),
    # ---------------- behaviors / smoothing ----------------
    (["behavior_server", "ros__parameters", "robot_base_frame"], "base_footprint"),
    (["velocity_smoother", "ros__parameters", "max_velocity"], [0.4, 0.0, 1.0]),
    (["velocity_smoother", "ros__parameters", "min_velocity"], [-0.4, 0.0, -1.0]),
    (["velocity_smoother", "ros__parameters", "max_accel"], [0.8, 0.0, 2.0]),
    (["velocity_smoother", "ros__parameters", "max_decel"], [-0.8, 0.0, -2.0]),
]


def find_default():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("nav2_bringup"), "params", "nav2_params.yaml")
    except Exception:  # noqa: BLE001
        sys.exit("cannot locate nav2_bringup: source /opt/ros/<distro>/setup.bash (or pass --default)")


def apply(doc, log):
    errors = []
    for path, value in OVERRIDES:
        node = doc
        for k in path[:-1]:
            if not isinstance(node, dict) or k not in node:
                errors.append(f"path not in installed default: {'.'.join(path[:-1])}")
                node = None
                break
            node = node[k]
        if node is None:
            continue
        key = path[-1]
        dotted = ".".join(path)
        if value is None:
            if key in node:
                del node[key]
                log.append(f"DEL    {dotted}")
            else:
                log.append(f"(skip) {dotted} not present, nothing to delete")
        elif key in node:
            if node[key] != value:
                old = node[key] if not isinstance(node[key], dict) else "{...}"
                log.append(f"CHANGE {dotted}: {old!r} -> {value if not isinstance(value, dict) else '{...}'!r}")
            node[key] = copy.deepcopy(value)
        else:
            log.append(f"ADD    {dotted} = {value if not isinstance(value, dict) else '{...}'!r}")
            node[key] = copy.deepcopy(value)

    # every *robot_base_frame / base_frame_id still pointing at base_link -> base_footprint
    def walk(d, prefix):
        for k, v in d.items():
            if isinstance(v, dict):
                walk(v, prefix + [k])
            elif k in ("robot_base_frame", "base_frame_id", "base_frame") and v == "base_link":
                d[k] = "base_footprint"
                log.append(f"CHANGE {'.'.join(prefix + [k])}: 'base_link' -> 'base_footprint' (sweep)")
    walk(doc, [])
    return errors


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--default")
    ap.add_argument("--out", default=OUT)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    src = a.default or find_default()
    doc = yaml.safe_load(open(src))
    log = []
    errors = apply(doc, log)
    if errors:
        for e in errors:
            print(f"ERROR  {e}")
        sys.exit(1)
    header = (f"# GENERATED by tools/make_nav2_params.py - do not edit by hand; change OVERRIDES there.\n"
              f"# Source: {src}\n# Changes vs the installed default:\n"
              + "".join(f"#   {line}\n" for line in log))
    text = header + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None, width=110)
    if a.check:
        same = os.path.exists(a.out) and open(a.out).read() == text
        print("PASS  nav2_params.yaml up to date" if same else "FAIL  nav2_params.yaml differs from generator output")
        sys.exit(0 if same else 1)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        f.write(text)
    for line in log:
        print(f"      {line}")
    print(f"PASS  wrote {os.path.relpath(a.out, WS)} ({len(log)} changes vs installed default)")


if __name__ == "__main__":
    main()
