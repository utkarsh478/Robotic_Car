"""Pure geometry: back-projection, transforms, cylinder/box estimators, tracker."""
import math

import numpy as np

from my_robot_perception import geometry as g

K = [[500.0, 0, 320.0], [0, 500.0, 240.0], [0, 0, 1]]


def test_backproject_principal_point_is_on_axis():
    p = g.backproject(np.array([320.0]), np.array([240.0]), np.array([2.0]), K)
    assert np.allclose(p, [[0, 0, 2.0]])


def test_backproject_offsets_scale_with_depth():
    p = g.backproject(np.array([420.0]), np.array([140.0]), np.array([1.5]), K)
    assert np.allclose(p, [[0.3, -0.3, 1.5]])


def test_quaternion_matrix_yaw():
    s, c = math.sin(math.pi / 4), math.cos(math.pi / 4)
    T = g.quat_to_matrix(0, 0, s, c, (1, 2, 3))
    assert np.allclose(g.transform(T, np.array([[1.0, 0, 0]])), [[1, 3, 3]], atol=1e-12)


def half_cylinder(c, r, cam, n=200, z=(0.02, 0.13)):
    """surface points visible from cam, uniform across the silhouette (like image pixels)"""
    view = (np.asarray(c) - cam) / np.linalg.norm(np.asarray(c) - cam)
    side = np.array([-view[1], view[0]])
    s = np.linspace(-0.999, 0.999, n)
    pts = np.asarray(c) + r * (s[:, None] * side - np.sqrt(1 - s ** 2)[:, None] * view)
    zz = np.linspace(z[0], z[1], n)
    return np.column_stack([pts, zz])


def test_cylinder_axis_exact_on_clean_points():
    c = np.array([1.0, 2.0])
    xy, info = g.cylinder_axis(half_cylinder(c, 0.025, np.array([0.0, 0.5])), [0.0, 0.5], 0.025)
    assert np.linalg.norm(xy - c) < 1e-6 and info["method"] == "circle-fit"


def test_cylinder_axis_noisy_points_fall_back_and_stay_close():
    rng = np.random.default_rng(3)
    c = np.array([1.0, 2.0])
    pts = half_cylinder(c, 0.025, np.array([0.0, 0.0]), n=400)
    pts[:, :2] += rng.normal(0, 0.01, (400, 2))
    xy, _ = g.cylinder_axis(pts, [0.0, 0.0], 0.025)
    assert np.linalg.norm(xy - c) < 0.006


def test_cylinder_ignores_top_cap():
    c = np.array([0.5, 0.0])
    side = half_cylinder(c, 0.025, np.array([-1.0, 0.0]))
    cap = np.column_stack([c[0] + np.random.default_rng(0).uniform(-0.02, 0.02, (100, 2)), np.full(100, 0.15)])
    xy, _ = g.cylinder_axis(np.vstack([side, cap]), [-1.0, 0.0], 0.025)
    assert np.linalg.norm(xy - c) < 1e-6


def test_spec_baseline_pushes_along_view():
    xy = g.cylinder_axis_spec(np.array([1.0, 0.0, 0.1]), [0.0, 0.0], 0.025)
    assert np.allclose(xy, [1.025, 0.0])


def test_plausibility_rejects_tall_wide_and_floating():
    tall = np.column_stack([np.zeros(50), np.zeros(50), np.linspace(0, 0.5, 50)])
    wide = np.column_stack([np.linspace(0, 0.4, 50), np.zeros(50), np.full(50, 0.05)])
    floating = np.column_stack([np.zeros(50), np.zeros(50), np.full(50, 0.3)])
    ok = half_cylinder([0, 0], 0.025, np.array([-1.0, 0]))
    assert not g.plausible_cylinder(tall, 0.025, 0.15)[0]
    assert not g.plausible_cylinder(wide, 0.025, 0.15)[0]
    assert not g.plausible_cylinder(floating, 0.025, 0.15)[0]
    assert g.plausible_cylinder(ok, 0.025, 0.15)[0]


def test_tracker_gates_windows_and_times_out():
    tr = g.Tracker(gate=0.15, timeout=10.0, window=3)
    for i in range(5):
        tr.update([1.0 + 0.01 * i, 0.0], t=float(i))
    tr.update([3.0, 0.0], t=5.0)                    # far away -> new track
    assert len(tr.tracks) == 2
    best = tr.best()
    assert best["count"] == 5 and np.allclose(best["mean"], [1.03, 0.0])   # mean of the last 3
    tr.prune(14.5)                                  # first track last seen at t=4 -> dropped
    assert len(tr.tracks) == 1 and tr.best()["count"] == 1
