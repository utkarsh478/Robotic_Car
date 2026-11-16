"""One-frame detection pipelines (no ROS): image + depth + intrinsics + camera pose -> estimate.
The ROS nodes and the offline tests call exactly these functions."""
import numpy as np

from . import geometry as g
from . import vision as vis

TARGET_DEFAULTS = {
    "hue_ranges": [[0, 10], [170, 180]], "s_min": 120, "v_min": 50,
    "morph_kernel": 3, "min_area": 150, "depth_min": 0.2, "depth_max": 4.0,
    "max_points": 600, "radius": 0.025, "height": 0.15, "mid_z": 0.075,
    "depth_gate": 0.08,     # keep points within +- this of the blob's median depth (edge pixels)
}
BOX_DEFAULTS = {
    "hue_ranges": [[45, 85]], "s_min": 100, "v_min": 50,
    "morph_kernel": 3, "min_area": 400, "depth_min": 0.2, "depth_max": 4.0,
    "max_points": 1500, "size": 0.30, "height": 0.15, "floor_z": 0.01,
    "depth_gate": 0.40,
}


def _points(blob, depth, K, T_map_opt, p, with_depth=False):
    u, v, z = vis.sample_pixels(blob, depth, p["depth_min"], p["depth_max"], p["max_points"])
    if len(z) == 0:
        return (None, None) if with_depth else None
    keep = np.abs(z - np.median(z)) <= p["depth_gate"]     # background seen through blob edges
    u, v, z = u[keep], v[keep], z[keep]
    pts = g.transform(T_map_opt, g.backproject(u, v, z, K))
    return (pts, z) if with_depth else pts


def detect_target(rgb, depth, K, T_map_opt, params=None):
    """-> (xy or None, info). Tries blobs largest-first and returns the first plausible one."""
    p = {**TARGET_DEFAULTS, **(params or {})}
    mask = vis.clean(vis.hsv_mask(rgb, p["hue_ranges"], p["s_min"], p["v_min"]), p["morph_kernel"])
    cam_xy = T_map_opt[:2, 3]
    info = {"mask": mask, "blobs": 0, "rejected": []}
    for area, blob in vis.blobs(mask, p["min_area"]):
        info["blobs"] += 1
        pts, zc = _points(blob, depth, K, T_map_opt, p, with_depth=True)
        if pts is None:
            info["rejected"].append((area, "no valid depth"))
            continue
        ok, why = g.plausible_cylinder(pts, p["radius"], p["height"])
        if not ok:
            info["rejected"].append((area, why))
            continue
        xy, fit = g.cylinder_axis(pts, cam_xy, p["radius"], height=p["height"], weights=zc ** 2)
        if xy is None:
            info["rejected"].append((area, f"no side or top-cap points ({fit})"))
            continue
        info.update(area=area, n_points=len(pts), fit=fit, blob=blob,
                    range=float(np.linalg.norm(xy - cam_xy)))
        return xy, info
    return None, info


def detect_box(rgb, depth, K, T_map_opt, params=None):
    """-> ((xy, yaw) or None, info)."""
    p = {**BOX_DEFAULTS, **(params or {})}
    mask = vis.clean(vis.hsv_mask(rgb, p["hue_ranges"], p["s_min"], p["v_min"]), p["morph_kernel"])
    cam_xy = T_map_opt[:2, 3]
    info = {"mask": mask, "blobs": 0, "rejected": []}
    for area, blob in vis.blobs(mask, p["min_area"]):
        info["blobs"] += 1
        pts = _points(blob, depth, K, T_map_opt, p)
        if pts is None:
            info["rejected"].append((area, "no valid depth"))
            continue
        ok, why = g.plausible_box(pts, p["size"], p["height"])
        if not ok:
            info["rejected"].append((area, why))
            continue
        xy, yaw, fit = g.box_floor_centre(pts, cam_xy, p["size"], p["height"])
        if xy is None:
            info["rejected"].append((area, "too few box points"))
            continue
        info.update(area=area, n_points=len(pts), fit=fit, blob=blob,
                    range=float(np.linalg.norm(xy - cam_xy)))
        return (xy, yaw), info
    return None, info
