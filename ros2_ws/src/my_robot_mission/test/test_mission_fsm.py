"""FSM transitions (SPEC §14): happy path, every failure branch of the SPEC §12 table, and the
timeout path of every state. The mission logic runs against fake_world.FakeWorld, a moving 2-D
robot with fake servers and detectors, on a fake 10 Hz clock (no ROS needed, ~1 s total).
test_mission_ros.py runs the real ROS node against fake action servers and topics."""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_world import FakeWorld  # noqa: E402

from my_robot_mission import core  # noqa: E402

HAPPY = ["INIT", "WAIT_FOR_START", "LOCALIZE", "SEARCH", "APPROACH_TARGET", "REFINE_TARGET", "FINAL_ALIGN_TARGET",
         "PICK", "NAVIGATE_HOME", "FIND_BOX", "APPROACH_BOX", "FINAL_ALIGN_BOX", "PLACE", "STOW_AND_FINISH",
         "MISSION_COMPLETE"]


def mission(w, **params):
    p = {"auto_start": True, "trial_id": "test"}
    p.update(params)
    return core.Mission(w, p)


def states(w):
    return [s for _, s in w.states]


def exits(w):
    return [(r["state"], r["outcome"], r["error_code"]) for r in w.rows]


# ------------------------------------------------------------------------------ happy path
@pytest.mark.parametrize("target", [(0.9, 2.5), (3.3, 2.4), (3.4, -1.2)], ids=["T2", "T1", "T3"])
def test_happy_path(target):
    w = FakeWorld(target=target)
    m = w.run(mission(w))
    assert m.state == "MISSION_COMPLETE", (m.reason, exits(w))
    assert states(w) == HAPPY
    assert w.results[-1]["success"] is True and w.results[-1]["reason"] == ""
    # one CSV row per state exit, in order, all successful
    assert [r["state"] for r in w.rows] == HAPPY[:-1]
    assert all(r["outcome"] == "success" for r in w.rows)
    # the pick went to the true target, the place to the true box, at the right height
    px, py, pz = w.requests["pick"][-1]
    assert math.hypot(px - target[0], py - target[1]) < 0.01 and pz == pytest.approx(0.075)
    bx, by, bz, _ = w.requests["place"][-1]
    assert math.hypot(bx - w.box[0], by - w.box[1]) < 0.01
    assert bz == pytest.approx(0.01 + 0.075 + 0.02)
    # object ended in the box; robot backed off; base commands stop at the end
    assert math.hypot(w.target[0] - w.box[0], w.target[1] - w.box[1]) < 0.01
    assert w.cmds[-1][1:] == (0.0, 0.0)
    # detectors: target on for SEARCH, off for the way home, box on, target on again for PLACE
    assert ("target", True) in w.requests["detector"] and ("box", True) in w.requests["detector"]
    assert w.requests["detector"][-2:] == [("target", False), ("box", False)]
    # LOCALIZE published the home pose with a spread that must be earned down
    assert w.requests["initial_pose"][0][:3] == (-3.0, -2.0, 0.0)


def test_alignment_reaches_the_pick_and_place_distances():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "PICK")
    bx, by = w.base(w.target)
    assert 0.395 <= bx <= 0.41 and abs(by) < 0.015            # SPEC: base-x 0.40, |y| < 0.015
    w.run(m, until=lambda m: m.state == "PLACE")
    bx, by = w.base(w.box)
    assert 0.475 <= bx <= 0.49 and abs(by) < 0.015            # DC-38: box at base-x 0.48
    # final approach speeds stay within SPEC (<= 0.10 m/s closed loop)
    assert max(abs(v) for _, v, _ in w.cmds) <= 0.10 + 1e-9


def test_waits_for_start_without_auto_start():
    w = FakeWorld()
    m = mission(w, auto_start=False)
    w.run(m, until=lambda m: m.state == "WAIT_FOR_START")
    for _ in range(100):
        w.step()
        m.tick(w.t)
    assert m.state == "WAIT_FOR_START"
    ok, _ = m.request_start()
    assert ok
    w.run(m, until=lambda m: m.state == "LOCALIZE")
    ok, msg = m.request_start()
    assert not ok and "LOCALIZE" in msg


# ------------------------------------------------------------------------------ LOCALIZE
def test_localize_converges_with_nomotion_updates_without_spinning():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "SEARCH")
    assert w.requests["nomotion"] >= 1 and not w.requests["spin"]


def test_localize_spins_when_nomotion_does_not_converge():
    w = FakeWorld()
    w.nomotion_works = False
    m = mission(w)
    w.run(m, until=lambda m: m.state == "SEARCH")
    assert w.requests["spin"] == [2 * math.pi]


def test_localize_fails_after_two_spins():
    w = FakeWorld()
    w.nomotion_works = False
    w.spin_converges = False
    m = w.run(mission(w))
    assert m.state == "FAILED" and m.reason == "LOCALIZATION_FAILED"
    assert len(w.requests["spin"]) == 2                       # spin once + 1 extra spin


# ------------------------------------------------------------------------------ SEARCH
def test_search_skips_a_waypoint_whose_navigation_fails():
    w = FakeWorld(target=(0.9, 2.5))
    w.nav_outcomes = ["fail"]                                  # waypoint 1
    m = w.run(mission(w))
    assert m.state == "MISSION_COMPLETE"
    search = [r for r in w.rows if r["state"] == "SEARCH"][0]
    assert search["retries"] == 1
    assert w.requests["spin"][0] == pytest.approx(2 * math.pi)  # first spin is at waypoint 2


def test_search_target_not_found_after_all_waypoints():
    w = FakeWorld()
    w.hidden = {"target"}
    m = w.run(mission(w))
    assert m.state == "FAILED" and m.reason == "TARGET_NOT_FOUND"
    assert len(w.requests["nav"]) == 6 and len(w.requests["spin"]) == 6


def test_search_cancels_the_active_goal_when_the_target_is_seen():
    w = FakeWorld(target=(0.9, 2.5))
    m = mission(w)
    w.run(m, until=lambda m: m.state == "APPROACH_TARGET")
    search_ops = [o for o in w.ops if o.kind in ("nav", "spin")]
    assert all(o.done for o in search_ops)
    assert any(o.cancelled for o in search_ops)


# ------------------------------------------------------------------------------ APPROACH
def test_approach_rotates_the_standoff_out_of_collision():
    w = FakeWorld(target=(3.3, 2.4))                           # T1, 0.6 m south of the north wall
    m = mission(w)
    w.run(m, until=lambda m: m.state == "SEARCH")
    w.cancel_all()                                             # stop SEARCH's goal, then put the robot
    w.robot = [3.3, 2.95, -math.pi / 2]                        # due north: the direct standoff is in the wall
    m.target_map = (3.3, 2.4)
    m.go("APPROACH_TARGET", "test")
    w.run(m, until=lambda m: m.state != "APPROACH_TARGET")
    note = [r for r in w.rows if r["state"] == "APPROACH_TARGET"][0]["note"]
    assert "rotated +90 deg" in note, note                    # +-30, +-60 are in the north/east walls
    sx, sy, syaw = w.requests["nav"][-1]
    assert (sx, sy) == pytest.approx((2.5, 2.4), abs=0.02) and syaw == pytest.approx(0.0, abs=0.03)


def test_approach_retries_twice_then_fails():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "APPROACH_TARGET")
    w.nav_outcomes = ["fail", "fail", "fail"]
    m = w.run(m)
    assert m.state == "FAILED" and m.reason == "APPROACH_TARGET_FAILED"
    assert w.requests["clear"] == 2


def test_approach_succeeds_on_retry():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "APPROACH_TARGET")
    w.nav_outcomes = ["fail", "fail"]
    m = w.run(m)
    assert m.state == "MISSION_COMPLETE"
    row = [r for r in w.rows if r["state"] == "APPROACH_TARGET"][0]
    assert row["retries"] == 2 and w.requests["clear"] == 2


def test_approach_without_a_costmap_uses_the_direct_standoff():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "APPROACH_TARGET")
    w.costmap_outcomes = ["fail"]
    m = w.run(m)
    assert m.state == "MISSION_COMPLETE"
    assert any("no costmap" in t for _, t in w.logs)


def test_standoff_candidates_and_footprint_check():
    c = core.standoff_candidates((0.0, 0.0), (2.0, 0.0), 0.8, [0.0, 90.0])
    assert c[0][:2] == pytest.approx((0.8, 0.0)) and c[0][2] == pytest.approx(math.pi)
    assert c[1][:2] == pytest.approx((0.0, 0.8), abs=1e-9)
    g = core.Grid(0.05, -1.0, -1.0, 40, 40, bytes([0] * 1600))
    assert core.pose_free(g, 0.0, 0.0, 0.3, [0.31, 0.27, -0.27, -0.27], 0.03)
    data = bytearray(1600)
    data[20 * 40 + 26] = core.LETHAL                           # cell centre (0.325, 0.025): inside padded front
    g2 = core.Grid(0.05, -1.0, -1.0, 40, 40, bytes(data))
    assert not core.pose_free(g2, 0.0, 0.0, 0.0, [0.31, 0.27, -0.27, -0.27], 0.03)
    assert core.pose_free(g2, 0.0, 0.0, math.pi / 2, [0.31, 0.27, -0.27, -0.27], 0.03)
    assert not core.pose_free(g, 0.9, 0.0, 0.0, [0.31, 0.27, -0.27, -0.27], 0.03)   # off the grid


# ------------------------------------------------------------------------------ REFINE / ALIGN
def test_refine_failure_reapproaches_once_then_fails():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "REFINE_TARGET")
    w.hidden = {"target"}
    m = w.run(m)
    assert m.state == "FAILED" and m.reason == "REFINE_TARGET_FAILED"
    assert [s for s, o, c in exits(w)].count("REFINE_TARGET") == 2
    assert ("REFINE_TARGET", "retry", "REFINE_FAILED") in exits(w)


def test_final_align_lost_track_backs_up_and_refines():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "FINAL_ALIGN_TARGET")
    w.hidden = {"target"}
    w.run(m, until=lambda m: m.state == "BACKUP")
    start = w.robot[:]
    w.hidden = set()
    w.run(m, until=lambda m: m.state == "REFINE_TARGET")
    assert math.hypot(w.robot[0] - start[0], w.robot[1] - start[1]) == pytest.approx(0.20, abs=0.01)
    m = w.run(m)
    assert m.state == "MISSION_COMPLETE"


def test_final_align_fails_after_one_retry():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "FINAL_ALIGN_TARGET")
    w.hidden = {"target"}
    w.run(m, until=lambda m: m.state == "REFINE_TARGET")
    w.hidden = set()
    w.run(m, until=lambda m: m.state == "FINAL_ALIGN_TARGET")
    w.hidden = {"target"}
    m = w.run(m)
    assert m.state == "FAILED" and m.reason == "FINAL_ALIGN_TARGET_FAILED"


# ------------------------------------------------------------------------------ PICK
def test_pick_grasp_not_detected_backs_up_refines_and_retries():
    w = FakeWorld()
    w.pick_outcomes = [core.GRASP_NOT_DETECTED]
    m = w.run(mission(w))
    assert m.state == "MISSION_COMPLETE"
    assert ("PICK", "retry", "PICK_GRASP_NOT_DETECTED") in exits(w)
    assert len(w.requests["pick"]) == 2
    s = states(w)
    i = s.index("PICK")
    assert s[i + 1:i + 5] == ["BACKUP", "REFINE_TARGET", "FINAL_ALIGN_TARGET", "PICK"]


def test_pick_unreachable_twice_fails():
    w = FakeWorld()
    w.pick_outcomes = [core.UNREACHABLE, core.UNREACHABLE]
    m = w.run(mission(w))
    assert m.state == "FAILED" and m.reason == "PICK_UNREACHABLE"
    assert len(w.requests["pick"]) == 2                        # max 2 pick attempts in total


def test_pick_other_errors_fail_immediately():
    w = FakeWorld()
    w.pick_outcomes = [2]                                       # PLANNING_FAILED
    m = w.run(mission(w))
    assert m.state == "FAILED" and m.reason == "PICK_PLANNING_FAILED"
    assert len(w.requests["pick"]) == 1


# ------------------------------------------------------------------------------ NAVIGATE_HOME
def test_navigate_home_retries_twice():
    for fails, final, reason in ((2, "MISSION_COMPLETE", ""), (3, "FAILED", "NAVIGATE_HOME_FAILED")):
        w = FakeWorld()
        w.on_state["NAVIGATE_HOME"] = lambda world, n=fails: world.nav_outcomes.extend(["fail"] * n)
        m = w.run(mission(w))
        assert (m.state, m.reason) == (final, reason)
        row = [r for r in w.rows if r["state"] == "NAVIGATE_HOME"][0]
        assert row["retries"] == min(fails, 3)
        assert w.requests["clear"] == 2


def test_object_lost_on_the_way_home():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "NAVIGATE_HOME")
    for _ in range(20):
        w.step()
        m.tick(w.t)
    w.attached = False                                         # Gazebo reports the weld gone
    m = w.run(m)
    assert m.state == "FAILED" and m.reason == "OBJECT_LOST"
    assert w.results[-1]["holding"] is True


def test_brief_grasp_glitch_is_ignored():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "NAVIGATE_HOME")
    w.fingers = (0.005, 0.023)                                 # 0.5 s below lost_q, then back
    for _ in range(5):
        w.step()
        m.tick(w.t)
    w.fingers = (0.023, 0.023)
    m = w.run(m)
    assert m.state == "MISSION_COMPLETE"


# ------------------------------------------------------------------------------ BOX side
def test_box_not_found_after_a_full_turn():
    w = FakeWorld()
    w.hidden = {"box"}
    m = w.run(mission(w))
    assert m.state == "FAILED" and m.reason == "BOX_NOT_FOUND"
    turns = [a for a in w.requests["spin"] if a == pytest.approx(0.6)]
    assert sum(turns) >= 2 * math.pi - 1e-6


def test_approach_box_fails_after_two_retries():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "APPROACH_BOX")
    w.nav_outcomes = ["fail", "fail", "fail"]
    m = w.run(m)
    assert m.state == "FAILED" and m.reason == "APPROACH_BOX_FAILED"


def test_final_align_box_retries_once():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "FINAL_ALIGN_BOX")
    w.hidden = {"box"}
    w.run(m, until=lambda m: m.state == "BACKUP")
    w.hidden = set()
    m = w.run(m)
    assert m.state == "MISSION_COMPLETE"
    w2 = FakeWorld()
    m2 = mission(w2)
    w2.run(m2, until=lambda m: m.state == "FINAL_ALIGN_BOX")
    w2.hidden = {"box"}
    m2 = w2.run(m2)
    assert m2.state == "FAILED" and m2.reason == "FINAL_ALIGN_BOX_FAILED"


# ------------------------------------------------------------------------------ PLACE
def test_place_retries_once_on_a_non_release_error():
    w = FakeWorld()
    w.place_outcomes = [3]                                     # EXECUTION_FAILED
    m = w.run(mission(w))
    assert m.state == "MISSION_COMPLETE"
    assert ("PLACE", "retry", "PLACE_EXECUTION_FAILED") in exits(w)
    w2 = FakeWorld()
    w2.place_outcomes = [3, 3]
    m2 = w2.run(mission(w2))
    assert m2.state == "FAILED" and m2.reason == "PLACE_EXECUTION_FAILED"


def test_release_not_confirmed_fails_without_retry():
    w = FakeWorld()
    w.place_outcomes = [core.RELEASE_NOT_CONFIRMED]
    m = w.run(mission(w))
    assert m.state == "FAILED" and m.reason == "PLACE_RELEASE_NOT_CONFIRMED"
    assert len(w.requests["place"]) == 1


# ------------------------------------------------------------------------------ FAILED state
def test_failed_cancels_goals_and_sends_five_zero_commands():
    w = FakeWorld()
    m = mission(w)
    w.run(m, until=lambda m: m.state == "APPROACH_TARGET")
    w.nav_outcomes = ["fail"] * 3
    m = w.run(m)
    t_fail = w.t
    for _ in range(10):
        w.step()
        m.tick(w.t)
    after = [c for c in w.cmds if c[0] >= t_fail - 1e-9]
    assert len(after) == 5 and all(c[1:] == (0.0, 0.0) for c in after)
    assert w.requests["cancel_all"] == 1
    assert w.results[-1]["success"] is False and w.rows[-1]["state"] == "APPROACH_TARGET"


# ------------------------------------------------------------------------------ every timeout
# state -> (setup applied to the world before the run, expected next state, expected reason)
TIMEOUTS = {
    "INIT": (dict(missing_until=1e9), "FAILED", "TIMEOUT_INIT"),
    "LOCALIZE": ({}, "FAILED", "TIMEOUT_LOCALIZE"),
    "SEARCH": ({}, "FAILED", "TARGET_NOT_FOUND"),
    "APPROACH_TARGET": ({}, "FAILED", "TIMEOUT_APPROACH_TARGET"),
    "REFINE_TARGET": ({}, "APPROACH_TARGET", None),
    "FINAL_ALIGN_TARGET": ({}, "BACKUP", None),
    "PICK": ({}, "FAILED", "TIMEOUT_PICK"),
    "NAVIGATE_HOME": ({}, "FAILED", "TIMEOUT_NAVIGATE_HOME"),
    "FIND_BOX": ({}, "FAILED", "BOX_NOT_FOUND"),
    "APPROACH_BOX": ({}, "FAILED", "TIMEOUT_APPROACH_BOX"),
    "FINAL_ALIGN_BOX": ({}, "BACKUP", None),
    "PLACE": ({}, "FAILED", "TIMEOUT_PLACE"),
    "STOW_AND_FINISH": ({}, "FAILED", "TIMEOUT_STOW_AND_FINISH"),
    "BACKUP": (dict(pick_outcomes=[core.GRASP_NOT_DETECTED]), "FAILED", "TIMEOUT_BACKUP"),
}


def test_every_state_with_a_timeout_is_covered():
    with_timeout = {k.split(".", 1)[1].upper() for k in core.DEFAULTS if k.startswith("timeout.")}
    assert with_timeout == set(TIMEOUTS)


@pytest.mark.parametrize("state", list(TIMEOUTS))
def test_timeout(state):
    setup, nxt, reason = TIMEOUTS[state]
    w = FakeWorld()
    for k, v in setup.items():
        setattr(w, k, v)
    if state != "INIT":
        w.freeze_at = state                                    # ops hang, base frozen, AMCL stuck
    if state == "REFINE_TARGET":                               # REFINE fails only by its timeout
        w.on_state["REFINE_TARGET"] = lambda world: world.hidden.add("target")
    m = mission(w)
    w.run(m, until=lambda m: m.state == state)
    t_in = w.t
    w.run(m, until=lambda m: m.state != state)
    limit = core.DEFAULTS["timeout." + state.lower()]
    assert w.t - t_in == pytest.approx(limit, abs=0.25), f"{state} left after {w.t - t_in:.1f} s"
    assert m.state == nxt
    if reason is not None:
        assert m.reason == reason
    row = [r for r in w.rows if r["state"] == state][-1]
    assert "timeout" in row["note"]
    if state == "INIT":
        assert "navigate_to_pose" in row["note"]
