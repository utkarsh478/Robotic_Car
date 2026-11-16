"""Colour segmentation (OpenCV, no ROS).  SPEC §9.1 steps 2-3.

HSV in OpenCV units: H 0..180, S and V 0..255. A hue range may wrap around 0 (red), so a
colour is a list of [h_lo, h_hi] intervals sharing one S/V floor.
"""
import cv2
import numpy as np


def hsv_mask(rgb, hue_ranges, s_min, v_min):
    """uint8 mask (0/255) of pixels whose hue is in any of `hue_ranges` and S/V above floors."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = np.zeros(rgb.shape[:2], np.uint8)
    for lo, hi in hue_ranges:
        mask |= cv2.inRange(hsv, (int(lo), int(s_min), int(v_min)), (int(hi), 255, 255))
    return mask


def clean(mask, kernel=3):
    if kernel <= 1:
        return mask
    k = np.ones((kernel, kernel), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)


def blobs(mask, min_area):
    """Connected blobs, largest first: list of (area, filled uint8 mask)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        m = np.zeros_like(mask)
        cv2.drawContours(m, [c], -1, 255, thickness=cv2.FILLED)
        out.append((area, m & mask))        # keep holes of the colour mask (e.g. the box opening)
    out.sort(key=lambda t: -t[0])
    return out


def sample_pixels(blob, depth, z_min, z_max, max_points, erode=1):
    """Pixels of a blob with valid depth: (u, v, z) arrays, at most `max_points` (uniform stride).
    The blob is eroded first so silhouette pixels (mixed depth) are not used."""
    m = blob
    if erode > 0:
        m = cv2.erode(blob, np.ones((2 * erode + 1, 2 * erode + 1), np.uint8))
        if cv2.countNonZero(m) < 20:          # tiny/far blob: keep the original
            m = blob
    v, u = np.nonzero(m)
    z = depth[v, u]
    ok = np.isfinite(z) & (z >= z_min) & (z <= z_max)
    u, v, z = u[ok], v[ok], z[ok]
    if len(u) > max_points:
        idx = np.linspace(0, len(u) - 1, max_points).astype(int)
        u, v, z = u[idx], v[idx], z[idx]
    return u.astype(np.float64), v.astype(np.float64), z.astype(np.float64)


def overlay(rgb, mask, color=(0, 255, 0)):
    out = rgb.copy()
    out[mask > 0] = (0.5 * out[mask > 0] + 0.5 * np.array(color)).astype(np.uint8)
    return out
