"""Regression test on frames captured from Gazebo by tools/gates/perception_eval.py
(tools/samples/NN_<kind>/{rgb.png, depth_mm.png, meta.json}). Skipped when no samples exist."""
import glob
import json
import os

import cv2
import numpy as np
import pytest

from my_robot_perception import pipeline as pl

HERE = os.path.dirname(os.path.realpath(__file__))
SAMPLES = os.environ.get("PERCEPTION_SAMPLES",
                         os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "tools", "samples"))
DIRS = sorted(d for d in glob.glob(os.path.join(SAMPLES, "*")) if os.path.isfile(os.path.join(d, "meta.json")))


def load(d):
    meta = json.load(open(os.path.join(d, "meta.json")))
    rgb = cv2.cvtColor(cv2.imread(os.path.join(d, "rgb.png"), cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
    mm = cv2.imread(os.path.join(d, "depth_mm.png"), cv2.IMREAD_UNCHANGED).astype(np.float32)
    depth = np.where(mm > 0, mm / 1000.0, np.inf).astype(np.float32)
    return meta, rgb, depth, np.array(meta["K"]), np.array(meta["T_map_optical"])


@pytest.mark.skipif(not DIRS, reason=f"no samples in {SAMPLES} (captured by gate_p08)")
@pytest.mark.parametrize("d", DIRS, ids=[os.path.basename(d) for d in DIRS])
def test_sample(d):
    meta, rgb, depth, K, T = load(d)
    kind = meta["kind"]
    if kind in ("target", "both"):
        xy, info = pl.detect_target(rgb, depth, K, T)
        assert xy is not None, info["rejected"]
        err = float(np.linalg.norm(xy - np.array(meta["gt_target"])))
        assert err < (0.02 if meta["range"] <= 1.55 else 0.04), f"error {err:.3f} m"
    elif kind == "distractor":
        xy, _ = pl.detect_target(rgb, depth, K, T)
        assert xy is None, f"distractor detected as target at {xy}"
    elif kind == "box":
        res, info = pl.detect_box(rgb, depth, K, T)
        assert res is not None, info["rejected"]
        err = float(np.linalg.norm(res[0] - np.array(meta["gt_box"])))
        assert err < 0.03, f"error {err:.3f} m"
