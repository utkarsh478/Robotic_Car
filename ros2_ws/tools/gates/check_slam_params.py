#!/usr/bin/env python3
"""CLAUDE.md rule 3: our slam_toolbox params must only use keys that exist in the installed
default. Prints every value that differs from the default. Exit 1 if we use an unknown key."""
import os
import sys

import yaml
from ament_index_python.packages import get_package_share_directory

WS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
OURS = os.path.join(WS, "src", "my_robot_navigation", "config", "slam_toolbox_params.yaml")
DEFAULT = os.path.join(get_package_share_directory("slam_toolbox"), "config", "mapper_params_online_async.yaml")

ours = yaml.safe_load(open(OURS))["slam_toolbox"]["ros__parameters"]
default = yaml.safe_load(open(DEFAULT))["slam_toolbox"]["ros__parameters"]

unknown = sorted(set(ours) - set(default))
missing = sorted(set(default) - set(ours))
changed = sorted(k for k in set(ours) & set(default) if ours[k] != default[k])

print(f"installed default: {DEFAULT}")
for k in changed:
    print(f"      changed  {k}: {default[k]!r} -> {ours[k]!r}")
for k in missing:
    print(f"WARN  not set in ours (installed default applies): {k} = {default[k]!r}")
if unknown:
    print(f"FAIL  keys not in the installed default: {unknown}")
    sys.exit(1)
print(f"PASS  slam_toolbox params: all {len(ours)} keys exist in the installed default ({len(changed)} changed)")
