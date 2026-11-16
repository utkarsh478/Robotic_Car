#!/usr/bin/env python3
"""Render a saved occupancy map to PNG with the true world geometry drawn on top.

  white = free, grey = unknown, black = occupied (from the map)
  red   = true obstacle/wall boundaries from the world SDF
  green = home pose (-3, -2)
Useful for debugging SLAM and for the report.
usage: render_map.py MAP.yaml WORLD.sdf OUT.png [--scale 4]
Pure Python (zlib + struct), no numpy/OpenCV needed.
"""
import argparse
import math
import os
import struct
import sys
import zlib

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "gates"))
import check_map  # noqa: E402  (reuses its PGM reader and SDF boundary sampler)


def write_png(path, w, h, rgb_rows):
    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    raw = b"".join(b"\x00" + bytes(r) for r in rgb_rows)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml")
    ap.add_argument("world_sdf")
    ap.add_argument("out_png")
    ap.add_argument("--scale", type=int, default=4)
    a = ap.parse_args()

    meta = yaml.safe_load(open(a.map_yaml))
    img = meta["image"] if os.path.isabs(meta["image"]) else os.path.join(os.path.dirname(a.map_yaml), meta["image"])
    mw, mh, pix = check_map.read_pgm(img)
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

    # canvas = union of map extent and room (+0.5 m margin), so a misplaced map is visible
    # snap the canvas origin to the map grid so every map cell lands on whole pixels
    ci0 = max(0, int(math.ceil((ox - (-4.6)) / res)))   # cells of margin left of the map
    cj0 = max(0, int(math.ceil((oy - (-3.6)) / res)))   # cells of margin below the map
    x0, y0 = ox - ci0 * res, oy - cj0 * res
    x1, y1 = max(ox + mw * res, 4.6), max(oy + mh * res, 3.6)
    s = a.scale
    W, H = int(math.ceil((x1 - x0) / res - 1e-9)) * s, int(math.ceil((y1 - y0) / res - 1e-9)) * s
    canvas = [[(70, 110, 160)] * W for _ in range(H)]  # blue-ish = outside the map

    def to_px(x, y):
        return int((x - x0) / res * s), H - 1 - int((y - y0) / res * s)

    colors = {254: (255, 255, 255), 205: (170, 170, 170), 0: (0, 0, 0)}
    for j in range(mh):
        for i in range(mw):
            v = pix[(mh - 1 - j) * mw + i]
            c = colors.get(v, (v, v, v))
            px, py = (ci0 + i) * s, H - (cj0 + j + 1) * s
            for dy in range(s):
                row = canvas[py + dy] if 0 <= py + dy < H else None
                if row is None:
                    continue
                for dx in range(s):
                    if 0 <= px + dx < W:
                        row[px + dx] = c

    def dot(x, y, c, r=1):
        px, py = to_px(x, y)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if 0 <= px + dx < W and 0 <= py + dy < H:
                    canvas[py + dy][px + dx] = c

    for _, (_, pts) in check_map.boundary_samples(a.world_sdf).items():
        for x, y in pts:
            dot(x, y, (230, 30, 30))
    for k in range(40):  # home: green ring
        t = 2 * math.pi * k / 40
        dot(-3.0 + 0.15 * math.cos(t), -2.0 + 0.15 * math.sin(t), (20, 180, 40))

    write_png(a.out_png, W, H, [[ch for px in row for ch in px] for row in canvas])
    print(f"wrote {a.out_png} ({W}x{H}); map extent x [{ox:.2f}, {ox + mw * res:.2f}] y [{oy:.2f}, {oy + mh * res:.2f}]")


if __name__ == "__main__":
    main()
