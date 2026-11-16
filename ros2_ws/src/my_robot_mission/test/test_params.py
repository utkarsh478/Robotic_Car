"""config/mission_params.yaml must set exactly the parameters core.DEFAULTS declares, with the same
types: a ROS parameter keeps the type of its declared default, so 5 vs 5.0 is a launch-time error."""
import os

import yaml

from my_robot_mission import core

HERE = os.path.dirname(os.path.abspath(__file__))
NODE_ONLY = {"use_sim_time", "results_dir"}


def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, dict):
            out.update(flatten(v, f"{prefix}{k}."))
        else:
            out[f"{prefix}{k}"] = v
    return out


def kind(v):
    if isinstance(v, list):
        return ("list", tuple(sorted({type(x).__name__ for x in v})))
    return type(v).__name__


def test_yaml_matches_defaults():
    with open(os.path.join(HERE, "..", "config", "mission_params.yaml")) as f:
        y = flatten(yaml.safe_load(f)["mission_fsm"]["ros__parameters"])
    keys = set(y) - NODE_ONLY
    assert keys == set(core.DEFAULTS), (sorted(keys - set(core.DEFAULTS)), sorted(set(core.DEFAULTS) - keys))
    for k in keys:
        assert kind(y[k]) == kind(core.DEFAULTS[k]), k
        assert y[k] == core.DEFAULTS[k], f"{k}: yaml {y[k]} != code {core.DEFAULTS[k]}"


def test_waypoints_are_pairs_and_far_from_targets():
    w = core.DEFAULTS["search_waypoints"]
    assert len(w) % 2 == 0
    for x, y in zip(w[::2], w[1::2]):
        for tx, ty in ((3.3, 2.4), (0.9, 2.5), (3.4, -1.2)):          # T1-T3, SPEC §12: >= 0.6 m
            assert (x - tx) ** 2 + (y - ty) ** 2 >= 0.6 ** 2
