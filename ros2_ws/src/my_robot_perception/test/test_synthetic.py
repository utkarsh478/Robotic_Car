"""Whole single-frame pipeline on ray-cast images with the real camera mounting and intrinsics.
Covers the PHASES P8 pose set: target at camera range 0.6/1.0/1.5/2.0 m x bearing -0.35/0/+0.35,
the box at 0.6-1.4 m (several box yaws), distractor-only, both in view, and orange look-alikes."""
import math

import numpy as np
import pytest

import synthetic as sy
from my_robot_perception import pipeline as pl

E = (0.25, 2.35)          # the gate's target spot in mission_world (middle zone, wall at y 2.95)
B = (-3.0, -0.9)          # home_box


def view(target_xy, rng, bearing, heading=math.pi / 2):
    return sy.camera_pose(*sy.robot_pose_for(target_xy, rng, bearing, heading))


@pytest.mark.parametrize("rng", [0.6, 1.0, 1.5, 2.0])
@pytest.mark.parametrize("bearing", [-0.35, 0.0, 0.35])
def test_target_accuracy(rng, bearing):
    T = view(E, rng, bearing)
    rgb, depth = sy.Scene(wall_y=2.95).cylinder(E).render(T)
    xy, info = pl.detect_target(rgb, depth, sy.K, T)
    assert xy is not None, info["rejected"]
    assert np.linalg.norm(xy - E) < (0.02 if rng <= 1.5 else 0.04)


@pytest.mark.parametrize("rng", [1.0, 2.0])
def test_target_accuracy_with_depth_noise(rng):
    """1 % depth noise (Gazebo has none; real sensors do): still inside the gate limits"""
    T = view(E, rng, 0.2)
    rgb, depth = sy.Scene(wall_y=2.95).cylinder(E).render(T)
    noise = np.random.default_rng(5).normal(0, 1, depth.shape) * 0.01 * np.where(np.isfinite(depth), depth, 0)
    xy, _ = pl.detect_target(rgb, depth + noise.astype(np.float32), sy.K, T)
    assert np.linalg.norm(xy - E) < (0.02 if rng <= 1.5 else 0.04)


@pytest.mark.parametrize("box_yaw", [0.0, 0.3, 0.7])
@pytest.mark.parametrize("rng,bearing", [(0.6, 0.0), (0.9, 0.25), (1.2, -0.25), (1.4, 0.0)])
def test_box_accuracy(box_yaw, rng, bearing):
    T = view(B, rng, bearing)
    rgb, depth = sy.Scene(wall_y=2.95).open_box(B, yaw=box_yaw).render(T)
    res, info = pl.detect_box(rgb, depth, sy.K, T)
    assert res is not None, info["rejected"]
    assert np.linalg.norm(res[0] - B) < 0.03


@pytest.mark.parametrize("name,scene", [
    ("distractor", lambda: sy.Scene(wall_y=2.95).cylinder(E, color=sy.YELLOW)),
    ("O1 orange box", lambda: sy.Scene(wall_y=2.95).box(E, (0.5, 0.5, 0.5), color=sy.O1_ORANGE)),
    ("arm-orange link", lambda: sy.Scene(wall_y=2.95).box(E, (0.04, 0.04, 0.28), z0=0.2, color=sy.ARM_ORANGE)),
    ("green box", lambda: sy.Scene(wall_y=2.95).open_box(E)),
    ("red but box-sized", lambda: sy.Scene(wall_y=2.95).box(E, (0.5, 0.5, 0.5), color=sy.RED)),
])
def test_target_detector_rejects(name, scene):
    T = view(E, 1.0, 0.0)
    rgb, depth = scene().render(T)
    xy, info = pl.detect_target(rgb, depth, sy.K, T)
    assert xy is None, f"{name} detected as target at {xy}"


def test_target_picked_when_distractor_also_in_view():
    T = view(E, 1.0, 0.0)
    t_xy, d_xy = (E[0] + 0.3, E[1]), (E[0] - 0.3, E[1])
    rgb, depth = sy.Scene(wall_y=2.95).cylinder(t_xy).cylinder(d_xy, color=sy.YELLOW).render(T)
    xy, _ = pl.detect_target(rgb, depth, sy.K, T)
    assert np.linalg.norm(xy - t_xy) < 0.01


def test_box_detector_ignores_cylinders():
    T = view(E, 1.0, 0.0)
    rgb, depth = sy.Scene(wall_y=2.95).cylinder(E).cylinder((E[0] + 0.2, E[1]), color=sy.YELLOW).render(T)
    res, _ = pl.detect_box(rgb, depth, sy.K, T)
    assert res is None


@pytest.mark.parametrize("bx", [0.40, 0.55, 0.80])
@pytest.mark.parametrize("y0", [-0.03, 0.0, 0.03])
def test_top_cap_only_when_the_side_is_not_red(bx, y0):
    """DC-35: Gazebo renders the side facing away from the sun without red (P9 run); the lit
    top cap alone must still give the axis, from close range out to 0.8 m."""
    E2 = (0.25, 2.0)
    yaw = -math.pi / 2
    c, s = math.cos(yaw), math.sin(yaw)
    T = sy.camera_pose(E2[0] - (c * bx - s * y0), E2[1] - (s * bx + c * y0), yaw)
    rgb, depth = sy.Scene().cylinder(E2, side_color=(0.25, 0.25, 0.25)).render(T)
    xy, info = pl.detect_target(rgb, depth, sy.K, T)
    assert xy is not None, info["rejected"]
    assert info["fit"]["method"] == "top-cap"
    assert np.linalg.norm(xy - E2) < 0.01


def test_red_plate_in_the_air_is_still_rejected():
    T = view(E, 1.0, 0.0)
    rgb, depth = sy.Scene(wall_y=2.95).box(E, (0.05, 0.05, 0.01), z0=0.30, color=sy.RED).render(T)
    xy, info = pl.detect_target(rgb, depth, sy.K, T)
    assert xy is None, info
