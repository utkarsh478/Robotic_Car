#!/usr/bin/env python3
"""Generate the self-collision matrix (disable_collisions) for config/my_robot.srdf.  SPEC §11.2

Same classification as MoveIt Setup Assistant:
  Adjacent - links of one rigid body (fixed joints) or of two bodies joined by one moving joint
  Default  - colliding in the zero configuration
  Always   - colliding in >= 95 % of random configurations
  Never    - never colliding in N random configurations (default 10 000) AND never closer than
             NEVER_MARGIN (3 cm). The margin is stricter than Setup Assistant: a pair that only
             collides in rare configurations still shows near misses, so it stays enabled.
Every other pair stays ENABLED, i.e. MoveIt checks it.

Collision test: robot_collision.py (sampled surfaces + exact signed distance, padding 3 mm, so
"touching" counts as colliding; that only ever keeps more pairs enabled = safer).

Usage (from the workspace root):
  python3 src/my_robot_moveit_config/scripts/gen_acm.py            # rewrite the SRDF block
  python3 src/my_robot_moveit_config/scripts/gen_acm.py --verify   # fresh samples, different seed:
        FAIL if any pair the SRDF disables as Never/Default/Always is seen colliding
The URDF is produced with `xacro <my_robot.urdf.xacro> with_arm:=true`, exactly like Gazebo/MoveIt.
"""
import argparse
import os
import re
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_collision import Robot, bounding_overlap, pair_clearance  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRDF = os.path.join(PKG, "config", "my_robot.srdf")
XACRO = os.path.join(os.path.dirname(PKG), "my_robot_description", "urdf", "my_robot.urdf.xacro")
BEGIN, END = "<!-- BEGIN gen_acm.py (generated, do not edit by hand) -->", "<!-- END gen_acm.py -->"
PAD = 0.003
NEVER_MARGIN = 0.03


def load_robot(urdf_path):
    if urdf_path:
        return Robot(open(urdf_path).read())
    xml = subprocess.run(["xacro", XACRO, "with_arm:=true"], check=True, capture_output=True, text=True).stdout
    return Robot(xml)


def sample_configs(robot, n, rng):
    q = {}
    for j in robot.moving:
        if j["type"] == "continuous":      # wheels: spheres, rotation does not matter
            q[j["name"]] = np.zeros(n)
        else:
            q[j["name"]] = rng.uniform(j["lower"], j["upper"], n)
    return q


def collision_counts(robot, pairs, q, chunk=400):
    """pair -> (number of colliding samples, minimum clearance seen)."""
    n = len(next(iter(q.values())))
    counts = {p: 0 for p in pairs}
    closest = {p: float("inf") for p in pairs}
    cache = {}
    for s in range(0, n, chunk):
        qc = {k: v[s:s + chunk] for k, v in q.items()}
        T = robot.fk(qc)
        for a, b in pairs:
            for sa in robot.shapes[a]:
                for sb in robot.shapes[b]:
                    near = bounding_overlap(sa, T[a], sb, T[b], NEVER_MARGIN + PAD)
                    if not near.any():
                        continue
                    idx = np.nonzero(near)[0]
                    d = pair_clearance(sa, T[a][idx], sb, T[b][idx], cache)
                    counts[(a, b)] += int((d < PAD).sum())
                    closest[(a, b)] = min(closest[(a, b)], float(d.min()))
    return counts, closest


def classify(robot, trials, seed, min_fraction=0.95):
    links = sorted(robot.shapes)
    all_pairs = [(a, b) for i, a in enumerate(links) for b in links[i + 1:]]
    adjacent = robot.adjacent_pairs()
    reason = {p: "Adjacent" for p in all_pairs if p in adjacent}
    rest = [p for p in all_pairs if p not in reason]
    zero = {j["name"]: np.zeros(1) for j in robot.moving}
    for p, c in collision_counts(robot, rest, zero)[0].items():
        if c:
            reason[p] = "Default"
    rest = [p for p in rest if p not in reason]
    counts, closest = collision_counts(robot, rest, sample_configs(robot, trials, np.random.default_rng(seed)))
    for p, c in counts.items():
        if c >= min_fraction * trials:
            reason[p] = "Always"
        elif c == 0 and closest[p] > NEVER_MARGIN:
            reason[p] = "Never"
    enabled = {p: (counts[p], closest[p]) for p in rest if p not in reason}
    return all_pairs, reason, enabled


def srdf_block(reason, trials, seed):
    lines = [BEGIN, f"  <!-- {trials} random configurations, seed {seed}, padding {PAD} m, "
                    f"Never margin {NEVER_MARGIN} m -->"]
    for (a, b), r in sorted(reason.items(), key=lambda kv: (kv[1], kv[0])):
        lines.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="{r}"/>')
    lines.append("  " + END)
    return "\n".join(lines)


def srdf_disabled():
    txt = open(SRDF).read()
    return {tuple(sorted(m[:2])): m[2] for m in
            re.findall(r'<disable_collisions\s+link1="([^"]+)"\s+link2="([^"]+)"\s+reason="([^"]+)"', txt)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=10000)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--urdf", help="use this URDF file instead of running xacro")
    ap.add_argument("--verify", action="store_true")
    a = ap.parse_args()
    robot = load_robot(a.urdf)
    t0 = time.time()

    if a.verify:
        disabled = srdf_disabled()
        adjacent = robot.adjacent_pairs()
        bad_adj = [p for p, r in disabled.items() if r == "Adjacent" and p not in adjacent]
        check = [p for p, r in disabled.items() if r != "Adjacent"]
        counts, _ = collision_counts(robot, check, sample_configs(robot, a.trials, np.random.default_rng(a.seed + 1000)))
        bad = {p: c for p, c in counts.items() if c and disabled[p] == "Never"}
        links = sorted(robot.shapes)
        n_all = len(links) * (len(links) - 1) // 2
        print(f"ACM verify: {len(disabled)} disabled / {n_all} pairs, {n_all - len(disabled)} checked by MoveIt; "
              f"{a.trials} fresh samples (seed {a.seed + 1000}) in {time.time() - t0:.0f} s")
        for p in bad_adj:
            print(f"FAIL  {p} marked Adjacent but is not adjacent in the URDF")
        for p, c in sorted(bad.items()):
            print(f"FAIL  {p} disabled as Never but collided in {c}/{a.trials} samples")
        ok = not bad and not bad_adj
        print("PASS  no disabled pair collides in fresh samples" if ok else "FAIL  ACM unsafe: rerun gen_acm.py")
        sys.exit(0 if ok else 1)

    all_pairs, reason, enabled = classify(robot, a.trials, a.seed)
    print(f"{len(all_pairs)} link pairs with geometry, {a.trials} samples, {time.time() - t0:.0f} s")
    for r in ("Adjacent", "Default", "Always", "Never"):
        print(f"  {r:8s} {sum(1 for v in reason.values() if v == r)}")
    print(f"  ENABLED  {len(enabled)} (checked by MoveIt):")
    for p, (c, d) in sorted(enabled.items(), key=lambda kv: (-kv[1][0], kv[1][1])):
        print(f"    {p[0]:18s} {p[1]:18s} collides in {100.0 * c / a.trials:5.2f} % of samples, closest {d:+.3f} m")
    txt = open(SRDF).read()
    if BEGIN not in txt or END not in txt:
        sys.exit(f"markers not found in {SRDF}")
    start = txt.index(BEGIN)
    stop = txt.index(END) + len(END)
    open(SRDF, "w").write(txt[:start] + srdf_block(reason, a.trials, a.seed) + txt[stop:])
    print(f"wrote {len(reason)} disable_collisions entries to {SRDF}")


if __name__ == "__main__":
    main()
