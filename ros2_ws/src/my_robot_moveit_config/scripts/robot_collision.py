#!/usr/bin/env python3
"""Pure-numpy collision geometry of the robot URDF (no ROS, no FCL).

Used by gen_acm.py (self-collision matrix) and check_named_states.py (stow/transport
verification). Only the primitives our URDF uses: box, cylinder (axis = local z), sphere.

Method: each collision shape is sampled on its surface (spacing `h`). Two convex shapes A, B
overlap iff part of A's surface lies inside B, or B lies inside A. Sampling the shape with the
smaller volume and testing those points against the other one's signed distance covers both
cases (the larger one cannot fit inside the smaller). Error is bounded by the sample spacing,
so a padding >= h/2 makes the test conservative (it reports touching-or-overlap).
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


# ---------------------------------------------------------------- transforms
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def make_tf(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def axis_angle(axis, q):
    """(N,4,4) rotations about a unit axis by angles q (N,)."""
    x, y, z = axis
    c, s = np.cos(q), np.sin(q)
    C = 1 - c
    out = np.zeros((len(q), 4, 4))
    out[:, 0, 0] = c + x * x * C
    out[:, 0, 1] = x * y * C - z * s
    out[:, 0, 2] = x * z * C + y * s
    out[:, 1, 0] = y * x * C + z * s
    out[:, 1, 1] = c + y * y * C
    out[:, 1, 2] = y * z * C - x * s
    out[:, 2, 0] = z * x * C - y * s
    out[:, 2, 1] = z * y * C + x * s
    out[:, 2, 2] = c + z * z * C
    out[:, 3, 3] = 1.0
    return out


def _vec(s, default):
    return [float(v) for v in s.split()] if s else default


# ---------------------------------------------------------------- shapes
class Shape:
    def __init__(self, kind, dims, tf_link, link):
        self.kind, self.dims, self.tf_link, self.link = kind, dims, tf_link, link
        if kind == "box":
            self.volume = dims[0] * dims[1] * dims[2]
            self.radius = 0.5 * math.sqrt(sum(d * d for d in dims))
        elif kind == "cylinder":
            r, length = dims
            self.volume = math.pi * r * r * length
            self.radius = math.hypot(r, length / 2)
        else:
            self.volume = 4.0 / 3.0 * math.pi * dims[0] ** 3
            self.radius = dims[0]

    def surface_points(self, h):
        """(M,3) points on the surface, in the SHAPE frame."""
        if self.kind == "box":
            sx, sy, sz = (d / 2 for d in self.dims)
            pts = []
            for ax, (a, b, c) in enumerate([(sx, sy, sz), (sy, sx, sz), (sz, sx, sy)]):
                u = np.linspace(-b, b, max(2, int(math.ceil(2 * b / h)) + 1))
                v = np.linspace(-c, c, max(2, int(math.ceil(2 * c / h)) + 1))
                U, V = np.meshgrid(u, v)
                for sign in (-1, 1):
                    P = np.zeros((U.size, 3))
                    W = np.full(U.size, sign * a)
                    if ax == 0:
                        P[:, 0], P[:, 1], P[:, 2] = W, U.ravel(), V.ravel()
                    elif ax == 1:
                        P[:, 1], P[:, 0], P[:, 2] = W, U.ravel(), V.ravel()
                    else:
                        P[:, 2], P[:, 0], P[:, 1] = W, U.ravel(), V.ravel()
                    pts.append(P)
            return np.unique(np.round(np.vstack(pts), 9), axis=0)
        if self.kind == "cylinder":
            r, length = self.dims
            n_t = max(8, int(math.ceil(2 * math.pi * r / h)))
            th = np.linspace(0, 2 * math.pi, n_t, endpoint=False)
            zs = np.linspace(-length / 2, length / 2, max(2, int(math.ceil(length / h)) + 1))
            T, Z = np.meshgrid(th, zs)
            side = np.c_[r * np.cos(T.ravel()), r * np.sin(T.ravel()), Z.ravel()]
            caps = []
            for rr in np.linspace(0, r, max(2, int(math.ceil(r / h)) + 1)):
                n = max(1, int(math.ceil(2 * math.pi * rr / h)))
                t = np.linspace(0, 2 * math.pi, n, endpoint=False)
                for z in (-length / 2, length / 2):
                    caps.append(np.c_[rr * np.cos(t), rr * np.sin(t), np.full(n, z)])
            return np.vstack([side] + caps)
        r = self.dims[0]
        n = max(20, int(4 * math.pi * r * r / (h * h)))
        i = np.arange(n) + 0.5
        phi = np.arccos(1 - 2 * i / n)
        th = math.pi * (1 + 5 ** 0.5) * i
        return r * np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]

    def sdf(self, p):
        """Signed distance of points p (...,3) given in the SHAPE frame (negative inside)."""
        if self.kind == "box":
            q = np.abs(p) - np.asarray(self.dims) / 2
            outside = np.linalg.norm(np.maximum(q, 0), axis=-1)
            inside = np.minimum(np.max(q, axis=-1), 0)
            return outside + inside
        if self.kind == "cylinder":
            r, length = self.dims
            d = np.stack([np.linalg.norm(p[..., :2], axis=-1) - r, np.abs(p[..., 2]) - length / 2], axis=-1)
            return np.minimum(np.max(d, axis=-1), 0) + np.linalg.norm(np.maximum(d, 0), axis=-1)
        return np.linalg.norm(p, axis=-1) - self.dims[0]


# ---------------------------------------------------------------- robot
class Robot:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.links = [l.get("name") for l in root.findall("link")]
        self.shapes = {}
        for l in root.findall("link"):
            for c in l.findall("collision"):
                o = c.find("origin")
                tf = make_tf(_vec(o.get("xyz") if o is not None else None, [0, 0, 0]),
                             _vec(o.get("rpy") if o is not None else None, [0, 0, 0]))
                g = c.find("geometry")[0]
                if g.tag == "box":
                    dims = _vec(g.get("size"), None)
                elif g.tag == "cylinder":
                    dims = [float(g.get("radius")), float(g.get("length"))]
                elif g.tag == "sphere":
                    dims = [float(g.get("radius"))]
                else:
                    raise ValueError(f"unsupported collision geometry {g.tag} in {l.get('name')}")
                self.shapes.setdefault(l.get("name"), []).append(Shape(g.tag, dims, tf, l.get("name")))
        self.joints = []
        for j in root.findall("joint"):
            o = j.find("origin")
            ax = j.find("axis")
            lim = j.find("limit")
            self.joints.append({
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "origin": make_tf(_vec(o.get("xyz") if o is not None else None, [0, 0, 0]),
                                  _vec(o.get("rpy") if o is not None else None, [0, 0, 0])),
                "axis": np.array(_vec(ax.get("xyz") if ax is not None else None, [1, 0, 0])),
                "lower": float(lim.get("lower", 0)) if lim is not None else 0.0,
                "upper": float(lim.get("upper", 0)) if lim is not None else 0.0,
            })
        children = {j["child"] for j in self.joints}
        roots = [l for l in self.links if l not in children]
        assert len(roots) == 1, roots
        self.root = roots[0]
        self.moving = [j for j in self.joints if j["type"] in ("revolute", "continuous", "prismatic")]

    def fk(self, q):
        """q: dict joint -> (N,) array (missing joints = 0). Returns link -> (N,4,4) in root frame."""
        n = len(next(iter(q.values()))) if q else 1
        T = {self.root: np.broadcast_to(np.eye(4), (n, 4, 4)).copy()}
        pending = list(self.joints)
        while pending:
            rest = []
            for j in pending:
                if j["parent"] not in T:
                    rest.append(j)
                    continue
                Tj = T[j["parent"]] @ j["origin"]
                v = q.get(j["name"], np.zeros(n))
                if j["type"] in ("revolute", "continuous"):
                    Tj = Tj @ axis_angle(j["axis"], v)
                elif j["type"] == "prismatic":
                    M = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
                    M[:, :3, 3] = np.outer(v, j["axis"])
                    Tj = Tj @ M
                T[j["child"]] = Tj
            if len(rest) == len(pending):
                raise ValueError("disconnected joints")
            pending = rest
        return T

    def rigid_groups(self):
        """Links connected by fixed joints form one rigid body. Returns link -> group id."""
        gid = {l: l for l in self.links}

        def find(a):
            while gid[a] != a:
                a = gid[a]
            return a
        for j in self.joints:
            if j["type"] == "fixed":
                gid[find(j["child"])] = find(j["parent"])
        return {l: find(l) for l in self.links}

    def adjacent_pairs(self):
        """Pairs that MoveIt Setup Assistant calls 'Adjacent': same rigid body, or rigid bodies
        connected directly by one moving joint."""
        g = self.rigid_groups()
        coll = sorted(self.shapes)
        linked = {frozenset((g[j["parent"]], g[j["child"]])) for j in self.moving}
        out = set()
        for i, a in enumerate(coll):
            for b in coll[i + 1:]:
                if g[a] == g[b] or frozenset((g[a], g[b])) in linked:
                    out.add((a, b))
        return out


def pair_clearance(sa, Ta, sb, Tb, pts_cache, h=0.004):
    """Approximate min distance (negative = penetration) between shapes over N poses.
    Ta, Tb: (N,4,4) LINK transforms. Returns (N,) array."""
    small, big, Ts, Tg = (sa, sb, Ta, Tb) if sa.volume <= sb.volume else (sb, sa, Tb, Ta)
    key = id(small)
    if key not in pts_cache:
        pts_cache[key] = small.surface_points(h)
    P = pts_cache[key]
    Ms = Ts @ small.tf_link                      # shape frames
    Mg = Tg @ big.tf_link
    rel = np.linalg.inv(Mg) @ Ms                 # small shape frame -> big shape frame
    pts = np.einsum("nij,mj->nmi", rel[:, :3, :3], P) + rel[:, None, :3, 3]
    return big.sdf(pts).min(axis=1)


def bounding_overlap(sa, Ta, sb, Tb, margin):
    ca = (Ta @ sa.tf_link)[:, :3, 3]
    cb = (Tb @ sb.tf_link)[:, :3, 3]
    return np.linalg.norm(ca - cb, axis=1) < sa.radius + sb.radius + margin
