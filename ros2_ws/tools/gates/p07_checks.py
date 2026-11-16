#!/usr/bin/env python3
"""Phase-7 gate, part 1: move_group plans AND executes on the Gazebo arm.

  1. named states: stow -> zero -> stow -> transport (plan + execute), tracking < 0.02 rad
  2. gripper group: closed -> open via MoveIt, tracking < 0.005 m
  3. collision rejection: IK goal with tool0 INSIDE the chassis (base_link (0.22, 0, 0.05)):
     /check_state_validity must report a contact with base_link, and plan+execute must fail
     with the arm not moving (< 0.01 rad)
  4. mission grasp poses: pre-grasp (base x 0.40, 0.225 m above ground) -> grasp (0.125 m,
     DC-20) -> stow, all planned by MoveIt from analytic-IK joint goals, tracking < 0.02 rad
A floor box is added to the planning scene first (top 2 mm below the ground, so the wheels and
casters resting on the ground do not count as touching it).
"""
import sys

import rclpy

from moveit_common import ARM, FINGERS, MoveItClient, ik, ik_cli_path, require, srdf_states

results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}", flush=True)


def go_and_check(n, label, group, joints, tol):
    r = n.go(group, joints)
    n.sim_sleep(0.5)
    err = n.joint_error(joints)
    ok = r["code"] == 1 and err < tol
    pt = f"{r['planning_time']:.3f} s" if r["planning_time"] is not None else "-"
    report(f"{label}", ok, f"{r['name']}, plan {pt}, wall {r['wall']:.1f} s, error {err:.4f}")
    return ok


def main():
    rclpy.init()
    n = MoveItClient("p07_checks")
    t = n.wait_ready()
    require(t is not None, "move_group (/move_action, /check_state_validity, /apply_planning_scene) "
                           "or /joint_states not available after 120 s")
    print(f"PASS  move_group interfaces ready ({t:.0f} s)", flush=True)
    report("floor added to planning scene",
           n.add_box("floor", "base_footprint", (3.0, 3.0, 0.02), (0.0, 0.0, -0.012)))
    n.spin_for(1.0)

    states = srdf_states()
    arm_state = {k: states[k][1] for k in ("stow", "zero", "transport")}
    v, contacts = n.state_valid({**arm_state["stow"], **{f: 0.04 for f in FINGERS}})
    report("stow is collision-free in MoveIt", v is True, ", ".join("-".join(c) for c in contacts))

    print("=== named states ===", flush=True)
    for label in ("stow", "zero", "stow", "transport"):
        go_and_check(n, f"plan+execute -> {label}", "arm", arm_state[label], 0.02)
    go_and_check(n, "gripper -> closed", "gripper", states["closed"][1], 0.005)
    go_and_check(n, "gripper -> open", "gripper", states["open"][1], 0.005)

    print("=== collision rejection: tool0 inside the chassis ===", flush=True)
    cli = ik_cli_path()
    ok, q_bad, txt = ik(cli, 0.22, 0.0, 0.125)
    require(ok, f"ik_cli could not solve the chassis test point: {txt}")
    bad = dict(zip(ARM, q_bad))
    v, contacts = n.state_valid(bad)
    report("goal state reported in collision with base_link", v is False and any("base_link" in c for c in contacts),
           f"valid={v}, contacts: {', '.join('-'.join(c) for c in contacts[:6])}")
    before = {j: n.js[j] for j in ARM}
    r = n.go("arm", bad)
    n.sim_sleep(0.5)
    moved = max(abs(n.js[j] - before[j]) for j in ARM)
    report("plan+execute rejected", r["code"] != 1, r["name"])
    report("arm did not move", moved < 0.01, f"max joint change {moved:.4f} rad")

    print("=== mission grasp poses (analytic IK goals, MoveIt paths) ===", flush=True)
    for label, h in (("pre-grasp  (0.40, 0, 0.225)", 0.225), ("grasp      (0.40, 0, 0.125)", 0.125)):
        ok, q, txt = ik(cli, 0.40, 0.0, h)
        require(ok, f"IK failed for {label}: {txt}")
        go_and_check(n, f"plan+execute -> {label}", "arm", dict(zip(ARM, q)), 0.02)
    go_and_check(n, "plan+execute -> stow", "arm", arm_state["stow"], 0.02)

    fails = results.count(False)
    print(f"\nP07 MoveIt checks: {len(results) - fails}/{len(results)} PASS", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
