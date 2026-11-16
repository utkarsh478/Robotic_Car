#!/usr/bin/env python3
"""Phase-3 map check: compare the saved map (world frame) with mission_world.sdf.

The ground truth comes from the world file itself (no duplicated numbers): every static
box/cylinder collision, plus the included home_box as its 0.30 x 0.30 outer footprint.
Their visible boundary is sampled every 2.5 cm inside the room (outer faces of the
outer walls can never be seen).

PASS criteria (PHASES.md P3):
  walls      >= 90 % of boundary samples have an occupied cell within 1 cell
  obstacles  >= 80 % (O1-O3)
  home_box   >= 60 % (its west face is never visible from the route)
  free cells outside the room: <= 20
  home pose (-3, -2) is free space
usage: check_map.py MAP.yaml WORLD.sdf [--json OUT]
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

ROOM = (-4.0, 4.0, -3.0, 3.0)
STEP = 0.025
FREE, UNKNOWN, OCC = 254, 205, 0
results = []


def report(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}")


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j].decode())
        i = j
    if tokens[0] != "P5":
        raise SystemExit(f"{path}: not a binary PGM (P5)")
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pix = data[i + 1:i + 1 + w * h]
    if maxval != 255 or len(pix) != w * h:
        raise SystemExit(f"{path}: unexpected PGM ({w}x{h}, max {maxval}, {len(pix)} bytes)")
    return w, h, pix


def pose_of(el):
    p = el.find("pose")
    v = [float(x) for x in p.text.split()] if p is not None and p.text else [0.0] * 6
    return v[0], v[1], v[5]


def boundary_samples(world_path):
    """-> {model_name: (category, [(x, y), ...])}"""
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    shapes = {}
    for m in world.findall("model"):
        if (m.findtext("static") or "").strip() != "true":
            continue
        name = m.get("name")
        mx, my, myaw = pose_of(m)
        for col in m.iter("collision"):
            g = col.find("geometry")
            cx, cy, cyaw = pose_of(col)
            x = mx + cx * math.cos(myaw) - cy * math.sin(myaw)
            y = my + cx * math.sin(myaw) + cy * math.cos(myaw)
            yaw = myaw + cyaw
            if g.find("box") is not None:
                sx, sy, _ = [float(v) for v in g.find("box/size").text.split()]
                shapes.setdefault(name, []).append(("box", x, y, yaw, sx, sy))
            elif g.find("cylinder") is not None:
                shapes.setdefault(name, []).append(("cyl", x, y, float(g.findtext("cylinder/radius"))))
    for inc in world.findall("include"):
        if "home_box" in (inc.findtext("uri") or ""):
            x, y, yaw = pose_of(inc)
            shapes["home_box"] = [("box", x, y, yaw, 0.30, 0.30)]

    out = {}
    for name, parts in shapes.items():
        pts = []
        for s in parts:
            if s[0] == "box":
                _, x, y, yaw, sx, sy = s
                hx, hy = sx / 2, sy / 2
                corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
                for k in range(4):
                    (ax, ay), (bx, by) = corners[k], corners[(k + 1) % 4]
                    n = max(1, int(math.hypot(bx - ax, by - ay) / STEP))
                    for t in range(n):
                        u, v = ax + (bx - ax) * t / n, ay + (by - ay) * t / n
                        pts.append((x + u * math.cos(yaw) - v * math.sin(yaw),
                                    y + u * math.sin(yaw) + v * math.cos(yaw)))
            else:
                _, x, y, r = s
                n = max(8, int(2 * math.pi * r / STEP))
                pts += [(x + r * math.cos(2 * math.pi * k / n), y + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
        pts = [(px, py) for px, py in pts
               if ROOM[0] - 0.01 <= px <= ROOM[1] + 0.01 and ROOM[2] - 0.01 <= py <= ROOM[3] + 0.01]
        cat = "walls" if name.startswith("wall") else ("home_box" if name == "home_box" else "obstacles")
        out[name] = (cat, pts)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml")
    ap.add_argument("world_sdf")
    ap.add_argument("--json")
    a = ap.parse_args()

    with open(a.map_yaml) as f:
        meta = yaml.safe_load(f)
    img = meta["image"] if os.path.isabs(meta["image"]) else os.path.join(os.path.dirname(a.map_yaml), meta["image"])
    w, h, pix = read_pgm(img)
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    print(f"map {w}x{h} @ {res} m, origin ({ox:.2f}, {oy:.2f}), "
          f"x [{ox:.2f}, {ox + w * res:.2f}] y [{oy:.2f}, {oy + h * res:.2f}]")

    def cell(x, y):
        i, j = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
        return (i, j) if 0 <= i < w and 0 <= j < h else None

    def value(i, j):
        return pix[(h - 1 - j) * w + i]  # PGM row 0 is the top (max y)

    def occupied_near(x, y):
        c = cell(x, y)
        if c is None:
            return False
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                i, j = c[0] + di, c[1] + dj
                if 0 <= i < w and 0 <= j < h and value(i, j) == OCC:
                    return True
        return False

    # ---- boundary coverage ----
    samples = boundary_samples(a.world_sdf)
    cats = {"walls": [0, 0], "obstacles": [0, 0], "home_box": [0, 0]}
    per_model = {}
    for name, (cat, pts) in sorted(samples.items()):
        hit = sum(occupied_near(x, y) for x, y in pts)
        per_model[name] = round(hit / max(1, len(pts)), 3)
        cats[cat][0] += hit
        cats[cat][1] += len(pts)
        print(f"      {name:<14s} {cat:<9s} {hit:4d}/{len(pts):4d} = {100 * hit / max(1, len(pts)):5.1f} %")
    thresholds = {"walls": 0.90, "obstacles": 0.80, "home_box": 0.60}
    cov = {}
    for cat, (hit, tot) in cats.items():
        frac = hit / max(1, tot)
        cov[cat] = round(frac, 3)
        report(f"{cat} boundary mapped >= {thresholds[cat] * 100:.0f} %", tot > 0 and frac >= thresholds[cat],
               f"{100 * frac:.1f} % of {tot} samples")

    # ---- no free space outside the room ----
    outside = 0
    for j in range(h):
        for i in range(w):
            if value(i, j) == FREE:
                x, y = ox + (i + 0.5) * res, oy + (j + 0.5) * res
                if x < ROOM[0] - 0.1 or x > ROOM[1] + 0.1 or y < ROOM[2] - 0.1 or y > ROOM[3] + 0.1:
                    outside += 1
    report("free cells outside the room <= 20", outside <= 20, f"{outside} cells (map misaligned or leaking if large)")

    # ---- home pose is free ----
    c = cell(-3.0, -2.0)
    v = value(*c) if c else None
    names = {FREE: "free", UNKNOWN: "unknown", OCC: "occupied"}
    report("home pose (-3, -2) is free space", v == FREE, names.get(v, str(v)))

    if a.json:
        os.makedirs(os.path.dirname(os.path.abspath(a.json)), exist_ok=True)
        with open(a.json, "w") as f:
            json.dump({"coverage": cov, "per_model": per_model, "free_outside": outside,
                       "home_cell": names.get(v, str(v)), "map": {"w": w, "h": h, "res": res, "origin": [ox, oy]}},
                      f, indent=2)

    n_fail = results.count(False)
    print(f"\nmap check: {len(results) - n_fail}/{len(results)} PASS")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
