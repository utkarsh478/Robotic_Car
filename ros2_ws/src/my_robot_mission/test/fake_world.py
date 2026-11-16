"""A small fake world for core.Mission: the IO protocol of my_robot_mission/core.py, backed by a
2-D robot that really moves (velocity commands, Nav2-like goals, spins), detectors that only see
objects in the camera's cone, an AMCL covariance that shrinks with updates, and pick/place
"servers" that succeed only when the robot really stands where the arm can reach.

Scripting:
  w.nav_outcomes / spin_outcomes / pick_outcomes / place_outcomes / costmap_outcomes:
      lists consumed one per call; an entry is "ok", "fail", an int error code (pick/place),
      or "hang" (never finishes). Empty list = "ok".
  w.freeze_at = "STATE": the moment the mission publishes that state, every new op hangs, the
      base ignores velocity commands, AMCL stops converging and ready() reports a missing item.
  w.hidden = {"target", "box"}: objects no detector can see.
"""
import math
import random

from my_robot_mission import core

MAP_TO_ODOM = (0.30, -0.20, 0.05)        # odom = R(a) * map + t: frames really differ here
CAM_X = 0.20                             # camera position in the base frame (SPEC §3.2)
CAM_Y = -0.12


def _tf(p, t):
    x, y, yaw = p
    dx, dy, a = t
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y + dx, s * x + c * y + dy, core.wrap(yaw + a)


def _tf_inv(p, t):
    x, y, yaw = p
    dx, dy, a = t
    c, s = math.cos(a), math.sin(a)
    x, y = x - dx, y - dy
    return c * x + s * y, -s * x + c * y, core.wrap(yaw - a)


# mission_world.sdf obstacles as axis-aligned rectangles (xmin, xmax, ymin, ymax) + circles
RECTS = [(-4.1, 4.1, 3.0, 3.1), (-4.1, 4.1, -3.1, -3.0), (4.0, 4.1, -3.1, 3.1), (-4.1, -4.0, -3.1, 3.1),
         (-1.05, -0.95, -3.0, 1.5), (1.45, 1.55, -1.5, 3.0),
         (-0.05, 0.45, -1.05, -0.55), (-2.5, -2.1, 0.8, 1.6),
         (-3.15, -2.85, -1.05, -0.75)]                       # home box (outer)
CIRCLES = [(2.6, 0.0, 0.25)]


class FakeOp:
    def __init__(self, world, kind, outcome, duration, on_ok=None, step=None):
        self.kind, self.outcome, self.duration, self.on_ok, self.step_fn = kind, outcome, duration, on_ok, step
        self.done, self.ok, self.code, self.status, self.message, self.result = False, False, None, "PENDING", "", None
        self.t0 = world.t
        self.cancelled = False
        world.ops.append(self)

    def finish(self, ok, status, code=None, message="", result=None):
        if not self.done:
            self.done, self.ok, self.status, self.code, self.message, self.result = True, ok, status, code, message, result

    def cancel(self):
        self.cancelled = True
        self.finish(False, "CANCELED")

    def step(self, world):
        if self.done or self.outcome == "hang":
            return
        if self.step_fn is not None:
            self.step_fn(world)
        if world.t - self.t0 + 1e-9 >= self.duration:
            if self.outcome == "ok":
                res = self.on_ok(world) if self.on_ok else None
                if type(res) is tuple:               # (ok, code, message) from a reach check
                    ok, code, msg = res
                    self.finish(ok, "SUCCEEDED" if ok else "ABORTED", code, msg)
                else:
                    self.finish(True, "SUCCEEDED", 0, "ok", res)
            elif self.outcome == "fail":
                self.finish(False, "ABORTED", None, "failed")
            else:                                     # an error code for pick/place
                self.finish(False, "ABORTED", int(self.outcome), f"error {self.outcome}")


class FakeWorld:
    def __init__(self, target=(0.9, 2.5), box=(-3.0, -0.9), robot=(-3.0, -2.0, 0.0), seed=1):
        self.t = 0.0
        self.dt = 0.1
        self.rng = random.Random(seed)
        self.robot = list(robot)
        self.target = list(target)
        self.box = list(box)
        self.target_held = False
        self.v = self.w = 0.0
        self.ops = []
        self.nav_outcomes, self.spin_outcomes, self.pick_outcomes, self.place_outcomes = [], [], [], []
        self.costmap_outcomes = []
        self.enabled = {"target": False, "box": False}
        self.obs = {"target": [], "box": []}
        self.count = {"target": 0, "box": 0}
        self.cache = {"target": None, "box": None}
        self.hidden = set()
        self.cov = None
        self.t_pub = None
        self.nomotion_works = True
        self.spin_converges = True
        self.missing_until = 1.0
        self.attached = None
        self.fingers = (0.04, 0.04)
        self.freeze_at = None
        self.on_state = {}             # state name -> callable(world), run when the mission enters it
        self.frozen = False
        self.states, self.results, self.rows, self.logs = [], [], [], []
        self.cmds = []
        self.requests = {"nav": [], "pick": [], "place": [], "spin": [], "initial_pose": [], "nomotion": 0,
                         "clear": 0, "cancel_all": 0, "detector": [], "reset": []}
        self.grid = None

    # ---------------------------------------------------------------- simulation
    def step(self):
        self.t += self.dt
        if not self.frozen:
            x, y, yaw = self.robot
            self.robot = [x + self.v * math.cos(yaw) * self.dt, y + self.v * math.sin(yaw) * self.dt,
                          core.wrap(yaw + self.w * self.dt)]
        for op in list(self.ops):
            op.step(self)
        for lb in ("target", "box"):
            if self.enabled[lb] and self.visible(lb):
                tx, ty = self.target if lb == "target" else self.box
                self.obs[lb] = (self.obs[lb] + [(tx + self.rng.gauss(0, 0.004), ty + self.rng.gauss(0, 0.004))])[-30:]
                self.count[lb] += 1
            if self.enabled[lb] and self.obs[lb]:
                n = len(self.obs[lb])
                mx = sum(o[0] for o in self.obs[lb]) / n
                my = sum(o[1] for o in self.obs[lb]) / n
                std = math.sqrt(sum((o[0] - mx) ** 2 + (o[1] - my) ** 2 for o in self.obs[lb]) / n) if n > 1 \
                    else float("nan")
                self.cache[lb] = core.Track(mx, my, 0.075 if lb == "target" else 0.01, 0.0, self.count[lb], std)

    def visible(self, lb):
        if lb in self.hidden or (lb == "target" and self.target_held):
            return False
        ox, oy = self.target if lb == "target" else self.box
        bx, by = core.to_base(ox, oy, self.robot)
        cx, cy = bx - CAM_X, by - CAM_Y
        return 0.15 < cx and math.hypot(cx, cy) < 2.7 and abs(math.atan2(cy, cx)) < 0.55

    def run(self, m, until=None, t_max=3000.0):
        """tick the mission at 10 Hz until `until(m)` or a terminal state"""
        while self.t < t_max:
            m.tick(self.t)
            if m.state in core.TERMINAL or (until is not None and until(m)):
                return m
            self.step()
        raise AssertionError(f"mission still in {m.state} at t {self.t:.1f}")

    def base(self, xy):
        return core.to_base(xy[0], xy[1], self.robot)

    def _next(self, lst):
        if self.frozen:
            return "hang"
        return lst.pop(0) if lst else "ok"

    # ---------------------------------------------------------------- IO protocol
    def ready(self):
        if self.frozen:
            return ["navigate_to_pose"]
        return ["navigate_to_pose"] if self.t < self.missing_until else []

    def robot_pose(self, frame):
        return tuple(self.robot) if frame == "map" else _tf(self.robot, MAP_TO_ODOM)

    def map_to_odom(self, x, y, yaw):
        return _tf((x, y, yaw), MAP_TO_ODOM)

    def track(self, label):
        return self.cache[label]

    def detector(self, label, enabled):
        self.enabled[label] = enabled
        self.requests["detector"].append((label, enabled))

    def reset_tracks(self, label):
        self.obs[label], self.cache[label], self.count[label] = [], None, 0
        self.requests["reset"].append(label)

    def nav(self, x, y, yaw):
        self.requests["nav"].append((x, y, yaw))
        goal = (x, y, yaw)

        def drive(world):
            rx, ry, _ = world.robot
            d = math.hypot(x - rx, y - ry)
            if d > 1e-6:
                s = min(0.3 * world.dt, d)
                h = math.atan2(y - ry, x - rx)
                world.robot = [rx + s * math.cos(h), ry + s * math.sin(h), h]

        def arrive(world):
            world.robot = list(goal)
        d = math.hypot(x - self.robot[0], y - self.robot[1])
        return FakeOp(self, "nav", self._next(self.nav_outcomes), d / 0.3 + 1.0, arrive, drive)

    def spin(self, angle):
        self.requests["spin"].append(angle)
        rate = math.copysign(1.0, angle)

        def turn(world):
            world.robot[2] = core.wrap(world.robot[2] + rate * world.dt)

        def done(world):
            if self.spin_converges and self.cov is not None:
                self.cov = (0.01, 0.01, 0.01, self.t)
        return FakeOp(self, "spin", self._next(self.spin_outcomes), abs(angle), done, turn)

    def pick(self, x, y, z):
        req = _tf_inv((x, y, 0.0), MAP_TO_ODOM)
        self.requests["pick"].append((req[0], req[1], z))

        def grasp(world):
            bx, by = world.base(world.target)
            ex, ey = world.base(req)
            if math.hypot(req[0] - world.target[0], req[1] - world.target[1]) > 0.02 or \
                    not (0.36 <= bx <= 0.44) or abs(by) > 0.03 or not (0.36 <= ex <= 0.44):
                return False, core.UNREACHABLE, f"target at base ({bx:.3f}, {by:.3f})"
            world.target_held, world.attached, world.fingers = True, True, (0.023, 0.023)
            return True, 0, "picked"
        return FakeOp(self, "pick", self._next(self.pick_outcomes), 5.0, grasp)

    def place(self, x, y, z, yaw):
        req = _tf_inv((x, y, yaw), MAP_TO_ODOM)
        self.requests["place"].append((req[0], req[1], z, req[2]))

        def release(world):
            bx, by = world.base(world.box)
            if math.hypot(req[0] - world.box[0], req[1] - world.box[1]) > 0.02 or not (0.45 <= bx <= 0.51) \
                    or abs(by) > 0.03:
                return False, core.UNREACHABLE, f"box at base ({bx:.3f}, {by:.3f})"
            world.target = [req[0], req[1]]
            world.target_held, world.attached, world.fingers = False, False, (0.04, 0.04)
            return True, 0, "placed"
        return FakeOp(self, "place", self._next(self.place_outcomes), 5.0, release)

    def costmap(self):
        out = self._next(self.costmap_outcomes)

        def give(world):
            return world.make_grid()
        return FakeOp(self, "costmap", out, 0.2, give)

    def make_grid(self, res=0.05):
        if self.grid is not None:
            return self.grid
        ox, oy, W, H = -4.2, -3.2, int(8.4 / res), int(6.4 / res)
        data = bytearray(W * H)
        for j in range(H):
            y = oy + (j + 0.5) * res
            for i in range(W):
                x = ox + (i + 0.5) * res
                d = min([math.hypot(max(a - x, 0, x - b), max(c - y, 0, y - e)) for a, b, c, e in RECTS]
                        + [max(0.0, math.hypot(x - cx, y - cy) - r) for cx, cy, r in CIRCLES]
                        + ([math.hypot(x - self.target[0], y - self.target[1])] if not self.target_held else []))
                if d <= 0.025:
                    v = core.LETHAL
                elif d <= 0.27:
                    v = core.INSCRIBED
                elif d <= 0.45:
                    v = int(252 * math.exp(-3.0 * (d - 0.27)))
                else:
                    v = 0
                data[j * W + i] = v
        self.grid = core.Grid(res, ox, oy, W, H, bytes(data))
        return self.grid

    def clear_costmaps(self):
        self.requests["clear"] += 1

    def cmd(self, v, w):
        self.cmds.append((self.t, v, w))
        if self.frozen:
            self.v = self.w = 0.0
        else:
            self.v, self.w = v, w

    def cancel_all(self):
        self.requests["cancel_all"] += 1
        for op in self.ops:
            if not op.done:
                op.cancel()

    def initial_pose(self, x, y, yaw, cov):
        self.requests["initial_pose"].append((x, y, yaw, list(cov)))
        self.cov = (cov[0], cov[1], cov[2], self.t)
        self.t_pub = self.t

    def nomotion(self):
        self.requests["nomotion"] += 1
        if self.nomotion_works and not self.frozen and self.cov is not None:
            self.cov = (self.cov[0] * 0.5, self.cov[1] * 0.5, self.cov[2] * 0.5, self.t + 0.05)

    def amcl_cov(self):
        return self.cov

    def grasp(self):
        return self.attached, self.fingers

    def publish_state(self, name):
        self.states.append((round(self.t, 2), name))
        if name in self.on_state:
            self.on_state[name](self)
        if self.freeze_at == name:
            self.frozen = True
            self.v = self.w = 0.0

    def publish_result(self, result):
        self.results.append(result)

    def csv(self, row):
        self.rows.append(row)

    def log(self, level, text):
        self.logs.append((level, text))
