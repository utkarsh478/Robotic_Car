"""Mission state machine, SPEC §12, with no ROS in it.

The ROS node (mission_fsm.py) owns every topic, action and service. It hands this class an
`io` object (see the IO protocol below) and calls `tick(now)` at 10 Hz. Because the logic only
talks to `io`, the pytest suite drives it with a fake world and a fake clock and can reach
every branch and every timeout in milliseconds.

Deviations from SPEC §12, recorded in docs/DESIGN_CHANGES.md:
  DC-38  the box creep ends at base-x 0.48, not 0.45 (box wall vs lidar puck)
  DC-35  "object lost" is judged from the attach state and finger positions, not from finger
         contacts: once Gazebo welds the object, the fingers stop touching it
  DC-41  BACKUP is its own state (logged, with a timeout); LOCALIZE asks AMCL for no-motion
         updates before spinning; the robot is monitored for a lost object until PLACE

IO protocol (all calls return immediately):
  ready() -> list[str]                    missing prerequisites, [] when all present
  robot_pose(frame) -> (x, y, yaw) | None frame "map" or "odom"
  map_to_odom(x, y, yaw) -> (x, y, yaw) | None
  track(label) -> Track | None            latest fused detection ("target" | "box"), map frame
  detector(label, enabled)                enable/disable a detector
  reset_tracks(label)                     clear a detector's tracks (and the cached track)
  nav(x, y, yaw) / spin(angle) / pick(x, y, z) / place(x, y, z, yaw) -> Op
  costmap() -> Op                         Op.result: Grid (raw Nav2 costs) or None
  clear_costmaps()
  cmd(v, w)                               base velocity (Twist on /cmd_vel)
  cancel_all()
  initial_pose(x, y, yaw, cov)            /initialpose
  nomotion()                              AMCL no-motion update request
  amcl_cov() -> (xx, yy, yawyaw, t_received) | None
  grasp() -> (attached: bool | None, fingers: (ql, qr) | None)
  publish_state(name), publish_result(dict), csv(row: dict), log(level, text)
Op: .done, .ok, .code (int, action error code), .status (str), .message, .result, .cancel()
"""
import math
from collections import namedtuple

Track = namedtuple("Track", "x y z yaw count std")
Grid = namedtuple("Grid", "resolution origin_x origin_y width height data")   # data: row-major raw costs

LETHAL, INSCRIBED, NO_INFO = 254, 253, 255

# PickObject / PlaceObject error codes (my_robot_interfaces, SPEC §7)
PICK_CODES = {0: "NONE", 1: "UNREACHABLE", 2: "PLANNING_FAILED", 3: "EXECUTION_FAILED",
              4: "GRASP_NOT_DETECTED", 5: "OBJECT_LOST", 6: "TIMEOUT"}
PLACE_CODES = {0: "NONE", 1: "UNREACHABLE", 2: "PLANNING_FAILED", 3: "EXECUTION_FAILED",
               4: "RELEASE_NOT_CONFIRMED", 6: "TIMEOUT"}
UNREACHABLE, GRASP_NOT_DETECTED, RELEASE_NOT_CONFIRMED = 1, 4, 4

DEFAULTS = {
    "auto_start": False,
    "trial_id": "auto",
    "home": [-3.0, -2.0, 0.0],
    # map frame, flat [x0, y0, x1, y1, ...]; checked in P11 (tools/gates/check_waypoints.py)
    "search_waypoints": [-2.6, 0.2, -2.2, 2.3, 0.0, 1.8, 0.4, -2.2, 2.5, -2.2, 2.3, 1.2],
    # state timeouts, s (SPEC §12 table); 0 = none
    "timeout.init": 90.0, "timeout.localize": 40.0, "timeout.search": 600.0,
    "timeout.approach_target": 180.0, "timeout.refine_target": 10.0, "timeout.final_align_target": 40.0,
    "timeout.pick": 90.0, "timeout.navigate_home": 300.0, "timeout.find_box": 60.0,
    "timeout.approach_box": 120.0, "timeout.final_align_box": 40.0, "timeout.place": 90.0,
    "timeout.stow_and_finish": 30.0, "timeout.backup": 15.0,
    # LOCALIZE
    "localize.initial_cov": [0.0625, 0.0625, 0.0685],     # RViz "2D Pose Estimate" spread
    "localize.max_cov_xy": 0.05, "localize.max_cov_yaw": 0.05,
    "localize.nomotion_period": 0.5, "localize.nomotion_count": 10,
    "localize.spins": 2, "localize.after_spin_wait": 3.0,
    # SEARCH / target
    "target.found_count": 5, "target.found_std": 0.03,
    "target.refine_count": 10, "target.refine_std": 0.02, "target.refine_dwell": 2.0,
    "target.standoff": 0.80,
    "target.align_x": 0.55, "target.final_x": 0.40, "target.max_y": 0.015,
    "target.diameter": 0.05, "target.height": 0.15, "target.mid_z": 0.075,
    # box
    "box.found_count": 5, "box.found_std": 0.03, "box.find_step": 0.6, "box.find_dwell": 1.0,
    "box.standoff": 0.85, "box.align_x": 0.65, "box.final_x": 0.48,       # 0.48: DC-38
    "box.place_clearance": 0.02,
    # standoff candidates: rotations (deg) about the object, tried in this order (SPEC §12)
    "standoff_rotations": [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0],
    "footprint": [0.31, 0.27, -0.27, -0.27],           # front, left, rear, right (DC-8, DC-17)
    "footprint_pad": 0.03,
    # final-approach controller (Nav2 idle, mission drives /cmd_vel)
    "align.nav_settle": 1.5, "align.fresh_obs": 5, "align.track_lost": 3.0,
    "align.bearing_tol": 0.02, "align.k_w": 1.5, "align.w_max": 0.4, "align.w_min": 0.06,
    "align.k_v": 1.0, "align.v_max": 0.10, "align.v_creep": 0.05, "align.v_min": 0.015,
    "align.freeze_dwell": 0.5, "align.freeze_obs": 8,
    # back-ups (odometry, rear is lidar-blind: short and slow)
    "backup.speed": 0.08, "backup.after_align": 0.20, "backup.after_pick": 0.15, "backup.finish": 0.30,
    # retries (SPEC §12)
    "retries.approach_target": 2, "retries.refine_target": 1, "retries.final_align_target": 1,
    "retries.pick_attempts": 2, "retries.navigate_home": 2, "retries.approach_box": 2,
    "retries.final_align_box": 1, "retries.place": 1,
    # grasp monitor while holding (DC-35: attach state + finger hold)
    "grasp.lost_q": 0.010, "grasp.lost_debounce": 1.0, "grasp.period": 0.2,
}

TERMINAL = ("MISSION_COMPLETE", "FAILED")


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def to_base(px, py, pose):
    """world point -> robot base frame (x forward, y left)"""
    x, y, yaw = pose
    dx, dy = px - x, py - y
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def footprint_cells(grid, x, y, yaw, footprint, pad):
    """raw costs of every cell whose centre is inside the padded footprint at (x, y, yaw);
    None if any part is off the grid"""
    front, left, rear, right = footprint
    front, left, rear, right = front + pad, left + pad, rear - pad, right - pad
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(front, left), (front, right), (rear, left), (rear, right)]
    wx = [x + c * a - s * b for a, b in corners]
    wy = [y + s * a + c * b for a, b in corners]
    r = grid.resolution
    i0, i1 = int(math.floor((min(wx) - grid.origin_x) / r)), int(math.floor((max(wx) - grid.origin_x) / r))
    j0, j1 = int(math.floor((min(wy) - grid.origin_y) / r)), int(math.floor((max(wy) - grid.origin_y) / r))
    if i0 < 0 or j0 < 0 or i1 >= grid.width or j1 >= grid.height:
        return None
    out = []
    for j in range(j0, j1 + 1):
        for i in range(i0, i1 + 1):
            cx, cy = grid.origin_x + (i + 0.5) * r - x, grid.origin_y + (j + 0.5) * r - y
            lx, ly = c * cx + s * cy, -s * cx + c * cy
            if rear <= lx <= front and right <= ly <= left:
                out.append(grid.data[j * grid.width + i])
    return out


def pose_free(grid, x, y, yaw, footprint, pad):
    """SPEC §12 "cost is lethal" test, done for the whole footprint: no lethal/unknown cell under
    the padded footprint, and the centre below 'inscribed' (NavFn refuses goals there)."""
    cells = footprint_cells(grid, x, y, yaw, footprint, pad)
    if not cells:
        return False
    i = int(math.floor((x - grid.origin_x) / grid.resolution))
    j = int(math.floor((y - grid.origin_y) / grid.resolution))
    centre = grid.data[j * grid.width + i]
    return all(v < LETHAL for v in cells) and centre < INSCRIBED


def standoff_candidates(obj_xy, robot_xy, dist, rotations_deg):
    """points `dist` from the object, first along object->robot, then rotated about the object;
    each faces the object"""
    ox, oy = obj_xy
    a0 = math.atan2(robot_xy[1] - oy, robot_xy[0] - ox)
    out = []
    for r in rotations_deg:
        a = a0 + math.radians(r)
        sx, sy = ox + dist * math.cos(a), oy + dist * math.sin(a)
        out.append((sx, sy, wrap(a + math.pi), r))
    return out


class Mission:
    def __init__(self, io, params=None):
        self.io = io
        self.P = dict(DEFAULTS)
        self.P.update(params or {})
        self.state = None
        self.t_enter = 0.0
        self.now = 0.0
        self.t_start = None
        self.start_requested = False
        self.holding = False
        self.reason = ""
        self.retries = {}          # counter name -> uses
        self.op = None             # active action/service op of the current state
        self.ph = None             # phase inside the current state
        self.t_ph = 0.0
        self.s = {}                # per-state scratch, cleared on entry
        self.target_map = None     # last good target estimate (x, y) in map
        self.box_map = None
        self.frozen = None         # (x, y, yaw) in odom, handed to PICK / PLACE
        self.backup_plan = None    # (distance, next_state)
        self.result = None
        self.history = []
        self._lost_since = None
        self._t_grasp = -1e9
        self._clock_started = False
        self.go("INIT", None)

    # ------------------------------------------------------------------ framework
    def p(self, key):
        return self.P[key]

    def request_start(self):
        if self.state == "WAIT_FOR_START":
            self.start_requested = True
            return True, "mission starting"
        if self.state == "INIT":
            self.start_requested = True
            return True, "start queued: still in INIT"
        return False, f"mission already in {self.state}"

    def bump(self, name):
        self.retries[name] = self.retries.get(name, 0) + 1
        return self.retries[name]

    def go(self, nxt, outcome, code="", note=""):
        if self.state is not None:
            self.io.csv({"state": self.state, "t_enter": round(self.t_enter, 2), "t_exit": round(self.now, 2),
                         "outcome": outcome, "retries": self.s.get("retries", 0), "error_code": code, "note": note})
            self.history.append((self.state, outcome, code))
            self.io.log("info", f"{self.state} -> {nxt} ({outcome}{' ' + str(code) if code else ''}"
                                f"{': ' + note if note else ''})")
        self.state, self.t_enter, self.op, self.ph, self.t_ph, self.s = nxt, self.now, None, None, self.now, {}
        self.io.publish_state(nxt)
        getattr(self, "enter_" + nxt.lower(), lambda: None)()

    def fail(self, reason, note="", outcome="failure"):
        self.reason = reason
        self.go("FAILED", outcome, reason, note)

    def phase(self, name):
        self.ph, self.t_ph = name, self.now

    def in_phase(self):
        return self.now - self.t_ph

    def tick(self, now):
        if not self._clock_started:                 # the first tick defines t = 0 for INIT
            self._clock_started = True
            self.t_enter = self.t_ph = now
        self.now = now
        if self.state in TERMINAL:
            getattr(self, "tick_" + self.state.lower())()
            return
        if self.holding and self.state not in ("PICK", "PLACE"):
            if self._object_lost():
                return
        limit = self.P.get("timeout." + self.state.lower(), 0.0)
        if limit and now - self.t_enter > limit:
            self.on_timeout()
            return
        getattr(self, "tick_" + self.state.lower())()

    def on_timeout(self):
        st = self.state
        if self.op is not None and not self.op.done:
            self.op.cancel()
        self.io.cmd(0.0, 0.0)
        note = f"timeout after {self.p('timeout.' + st.lower()):.0f} s"
        if st == "INIT":
            note += "; missing: " + ", ".join(self.s.get("missing", []))
        if st == "REFINE_TARGET":
            self.refine_failed(note)
        elif st == "FINAL_ALIGN_TARGET":
            self.final_align_failed(note)
        elif st == "FINAL_ALIGN_BOX":
            self.final_align_box_failed(note)
        elif st == "SEARCH":
            self.fail("TARGET_NOT_FOUND", note, "timeout")
        elif st == "FIND_BOX":
            self.fail("BOX_NOT_FOUND", note, "timeout")
        else:
            self.fail(f"TIMEOUT_{st}", note, "timeout")

    # ------------------------------------------------------------------ helpers
    def found(self, label, count_key, std_key, min_count=None):
        t = self.io.track(label)
        if t is None:
            return None
        need = self.p(count_key) if min_count is None else min_count
        if t.count >= need and math.isfinite(t.std) and t.std < self.p(std_key):
            return t
        return None

    def _object_lost(self):
        if self.now - self._t_grasp < self.p("grasp.period"):
            return False
        self._t_grasp = self.now
        attached, q = self.io.grasp()
        lost = attached is False or (q is not None and min(q) < self.p("grasp.lost_q"))
        if not lost:
            self._lost_since = None
            return False
        if self._lost_since is None:
            self._lost_since = self.now
        if self.now - self._lost_since < self.p("grasp.lost_debounce"):
            return False
        if self.op is not None and not self.op.done:
            self.op.cancel()
        self.io.cmd(0.0, 0.0)
        self.fail("OBJECT_LOST", f"attached {attached}, fingers {q} (lost below {self.p('grasp.lost_q')})")
        return True

    def start_backup(self, dist, nxt):
        self.backup_plan = (dist, nxt)
        self.go("BACKUP", "retry", note=f"back up {dist:.2f} m, then {nxt}")

    # ------------------------------------------------------------------ INIT / WAIT / LOCALIZE
    def tick_init(self):
        missing = self.io.ready()
        self.s["missing"] = missing
        if not missing:
            self.go("WAIT_FOR_START", "success")

    def tick_wait_for_start(self):
        if self.p("auto_start") or self.start_requested:
            self.t_start = self.now
            self.go("LOCALIZE", "success", note="auto_start" if self.p("auto_start") else "/mission/start")

    def enter_localize(self):
        x, y, yaw = self.p("home")
        self.io.initial_pose(x, y, yaw, self.p("localize.initial_cov"))
        self.s.update(t_pub=self.now, n_nomo=0, spins=0)
        self.phase("nomotion")

    def _converged(self):
        c = self.io.amcl_cov()
        if c is None or c[3] <= self.s["t_pub"] + 0.2:
            return False
        return c[0] < self.p("localize.max_cov_xy") and c[1] < self.p("localize.max_cov_xy") \
            and c[2] < self.p("localize.max_cov_yaw")

    def tick_localize(self):
        if self._converged():
            self.go("SEARCH", "success", note=f"{self.s['n_nomo']} no-motion updates, {self.s['spins']} spins")
            return
        if self.ph == "nomotion":
            if self.in_phase() >= self.p("localize.nomotion_period"):
                if self.s["n_nomo"] >= self.p("localize.nomotion_count"):
                    self._localize_spin()
                else:
                    self.io.nomotion()
                    self.s["n_nomo"] += 1
                    self.phase("nomotion")
        elif self.ph == "spin":
            if self.op.done:
                self.phase("after_spin")
        elif self.ph == "after_spin":
            if self.in_phase() >= self.p("localize.after_spin_wait"):
                self._localize_spin()

    def _localize_spin(self):
        if self.s["spins"] >= self.p("localize.spins"):
            c = self.io.amcl_cov()
            self.fail("LOCALIZATION_FAILED", f"AMCL covariance {None if c is None else [round(v, 4) for v in c[:3]]}")
            return
        self.s["spins"] += 1
        self.s["retries"] = self.s["spins"]
        self.op = self.io.spin(2 * math.pi)
        self.phase("spin")

    # ------------------------------------------------------------------ SEARCH
    def enter_search(self):
        self.io.detector("target", True)
        self.io.reset_tracks("target")
        w = self.p("search_waypoints")
        self.s.update(wps=[(w[i], w[i + 1]) for i in range(0, len(w) - 1, 2)], i=0, skipped=[])
        self._search_next()

    def _search_next(self):
        if self.s["i"] >= len(self.s["wps"]):
            self.fail("TARGET_NOT_FOUND", f"all {len(self.s['wps'])} waypoints searched; nav failed at "
                                          f"{self.s['skipped']}")
            return
        wx, wy = self.s["wps"][self.s["i"]]
        r = self.io.robot_pose("map")
        yaw = math.atan2(wy - r[1], wx - r[0]) if r is not None else 0.0
        self.op = self.io.nav(wx, wy, yaw)
        self.phase("nav")

    def tick_search(self):
        t = self.found("target", "target.found_count", "target.found_std")
        if t is not None:
            if self.op is not None and not self.op.done:
                self.op.cancel()
            self.target_map = (t.x, t.y)
            self.go("APPROACH_TARGET", "success",
                    note=f"target at ({t.x:.2f}, {t.y:.2f}), {t.count} obs, std {t.std:.3f}, waypoint {self.s['i'] + 1}")
            return
        if self.op is None or not self.op.done:
            return
        if self.ph == "nav":
            if self.op.ok:
                self.op = self.io.spin(2 * math.pi)
                self.phase("spin")
                return
            self.s["skipped"].append(self.s["i"] + 1)
            self.s["retries"] = len(self.s["skipped"])
            self.io.log("warn", f"SEARCH: waypoint {self.s['i'] + 1} nav {self.op.status}, skipped")
        self.s["i"] += 1
        self._search_next()

    # ------------------------------------------------------------------ APPROACH (target / box)
    def _approach_enter(self, label):
        self.s.update(label=label, tries=0)
        self._approach_start()

    def _approach_start(self):
        self.op = self.io.costmap()
        self.phase("costmap")

    def _approach_tick(self, label, dist, retries_key, ok_next, fail_reason):
        if self.op is None or not self.op.done:
            return
        if self.ph == "costmap":
            obj = self.target_map if label == "target" else self.box_map
            t = self.io.track(label)
            if t is not None and t.count >= 1:
                obj = (t.x, t.y)
                if label == "target":
                    self.target_map = obj
                else:
                    self.box_map = obj
            r = self.io.robot_pose("map")
            if obj is None or r is None:
                self._approach_retry(retries_key, fail_reason, "no object estimate or no robot pose")
                return
            grid = self.op.result if self.op.ok else None
            cands = standoff_candidates(obj, r[:2], dist, self.p("standoff_rotations"))
            pick = None
            if grid is None:
                self.io.log("warn", "APPROACH: no costmap, using the direct standoff unchecked")
                pick = cands[0]
            else:
                for c in cands:
                    if pose_free(grid, c[0], c[1], c[2], self.p("footprint"), self.p("footprint_pad")):
                        pick = c
                        break
            if pick is None:
                self._approach_retry(retries_key, fail_reason, "every standoff candidate is in collision")
                return
            self.s["standoff"] = pick
            self.op = self.io.nav(pick[0], pick[1], pick[2])
            self.phase("nav")
        elif self.ph == "nav":
            sx, sy, syaw, rot = self.s["standoff"]
            if self.op.ok:
                self.go(ok_next, "success", note=f"standoff ({sx:.2f}, {sy:.2f}, {syaw:.2f}), rotated {rot:+.0f} deg")
            else:
                self._approach_retry(retries_key, fail_reason, f"nav {self.op.status}")

    def _approach_retry(self, retries_key, fail_reason, why):
        self.s["tries"] += 1
        self.s["retries"] = self.s["tries"]
        if self.s["tries"] > self.p(retries_key):
            self.fail(fail_reason, why)
            return
        self.io.log("warn", f"{self.state}: {why}; clearing costmaps, retry {self.s['tries']}/{self.p(retries_key)}")
        self.io.clear_costmaps()
        self._approach_start()

    def enter_approach_target(self):
        self._approach_enter("target")

    def tick_approach_target(self):
        self._approach_tick("target", self.p("target.standoff"), "retries.approach_target",
                            "REFINE_TARGET", "APPROACH_TARGET_FAILED")

    def enter_approach_box(self):
        self._approach_enter("box")

    def tick_approach_box(self):
        self._approach_tick("box", self.p("box.standoff"), "retries.approach_box",
                            "FINAL_ALIGN_BOX", "APPROACH_BOX_FAILED")

    # ------------------------------------------------------------------ REFINE_TARGET
    def enter_refine_target(self):
        self.io.cmd(0.0, 0.0)
        self.io.detector("target", True)
        self.io.reset_tracks("target")
        self.s["retries"] = self.retries.get("refine_target", 0)

    def tick_refine_target(self):
        if self.now - self.t_enter < self.p("target.refine_dwell"):
            return
        t = self.found("target", "target.refine_count", "target.refine_std")
        if t is not None:
            self.target_map = (t.x, t.y)
            self.go("FINAL_ALIGN_TARGET", "success", note=f"{t.count} obs, std {t.std:.4f}")

    def refine_failed(self, note):
        t = self.io.track("target")
        why = note + (f"; track {t.count} obs, std {t.std:.4f}" if t is not None else "; no track")
        if self.bump("refine_target") > self.p("retries.refine_target"):
            self.fail("REFINE_TARGET_FAILED", why)
        else:
            self.go("APPROACH_TARGET", "retry", "REFINE_FAILED", why)

    # ------------------------------------------------------------------ FINAL_ALIGN (target / box)
    def _align_enter(self, label):
        self.io.cmd(0.0, 0.0)
        t = self.io.track(label)
        self.s.update(label=label, count0=t.count if t is not None else 0, t_seen=self.now)
        self.frozen = None
        self.phase("settle")

    def _align_tick(self, label, align_x, final_x, on_done, on_fail):
        P = self.p
        if self.ph in ("settle", "rotate", "approach"):
            t = self.io.track(label)
            r = self.io.robot_pose("map")
            if t is not None and t.count > self.s.get("last_count", -1):
                self.s["last_count"], self.s["t_seen"] = t.count, self.now
            if self.now - self.s["t_seen"] > P("align.track_lost"):
                self.io.cmd(0.0, 0.0)
                on_fail(f"no new {label} observations for {P('align.track_lost'):.1f} s in phase {self.ph}")
                return
            if t is None or r is None:
                self.io.cmd(0.0, 0.0)
                return
            bx, by = to_base(t.x, t.y, r)
            bearing = math.atan2(by, bx)
            if self.ph == "settle":
                if self.in_phase() >= P("align.nav_settle") and t.count - self.s["count0"] >= P("align.fresh_obs"):
                    self.phase("rotate")
                return
            if self.ph == "rotate":
                if abs(bearing) < P("align.bearing_tol"):
                    self.io.cmd(0.0, 0.0)
                    self.phase("approach")
                    return
                w = clip(P("align.k_w") * bearing, -P("align.w_max"), P("align.w_max"))
                w = math.copysign(max(abs(w), P("align.w_min")), w)
                self.io.cmd(0.0, w)
                return
            # approach: closed loop on the detection
            if bx <= align_x + 0.002:
                # Stop, then estimate again from scratch: observations taken at rest share one
                # map->odom (AMCL only updates after motion), so the odom pose frozen from them
                # is free of the AMCL corrections made while driving in.
                self.io.cmd(0.0, 0.0)
                self.io.reset_tracks(label)
                self.phase("freeze")
                return
            v = clip(P("align.k_v") * (bx - align_x), P("align.v_min"), P("align.v_max"))
            w = clip(P("align.k_w") * bearing, -P("align.w_max"), P("align.w_max"))
            self.io.cmd(v, w)
            return
        if self.ph == "freeze":
            self.io.cmd(0.0, 0.0)
            t = self.io.track(label)
            if self.in_phase() > P("align.track_lost") and (t is None or t.count < P("align.freeze_obs")):
                on_fail(f"only {0 if t is None else t.count} {label} observations at rest in "
                        f"{P('align.track_lost'):.1f} s (need {P('align.freeze_obs')})")
                return
            if self.in_phase() < P("align.freeze_dwell") or t is None or t.count < P("align.freeze_obs"):
                return
            o = self.io.map_to_odom(t.x, t.y, t.yaw)
            if o is None:
                return
            self.frozen = (o[0], o[1], o[2], t.z)
            self.s["frozen_count"] = t.count
            self.phase("creep")
            return
        r = self.io.robot_pose("odom")
        if r is None:
            self.io.cmd(0.0, 0.0)
            return
        bx, by = to_base(self.frozen[0], self.frozen[1], r)
        bearing = math.atan2(by, bx)
        if self.ph == "creep":
            if bx <= final_x + 0.002:
                self.io.cmd(0.0, 0.0)
                self.phase("check")
                return
            v = clip(P("align.k_v") * (bx - final_x), P("align.v_min"), P("align.v_creep"))
            w = clip(P("align.k_w") * bearing, -P("align.w_max"), P("align.w_max"))
            self.io.cmd(v, w)
        elif self.ph == "check":
            self.io.cmd(0.0, 0.0)
            if self.in_phase() < 0.3:
                return
            if abs(by) < P("target.max_y"):
                on_done(bx, by)
            else:
                self.phase("fine")
        elif self.ph == "fine":
            if abs(bearing) < P("align.bearing_tol") / 2:
                self.io.cmd(0.0, 0.0)
                self.phase("check")
                return
            w = clip(P("align.k_w") * bearing, -P("align.w_max"), P("align.w_max"))
            self.io.cmd(0.0, math.copysign(max(abs(w), P("align.w_min")), w))

    def enter_final_align_target(self):
        self._align_enter("target")

    def tick_final_align_target(self):
        def done(bx, by):
            self.go("PICK", "success", note=f"target frozen in odom ({self.frozen[0]:.3f}, {self.frozen[1]:.3f}), "
                                            f"base ({bx:.3f}, {by:+.3f})")
        self._align_tick("target", self.p("target.align_x"), self.p("target.final_x"), done, self.final_align_failed)

    def final_align_failed(self, note):
        self.io.cmd(0.0, 0.0)
        if self.bump("final_align_target") > self.p("retries.final_align_target"):
            self.fail("FINAL_ALIGN_TARGET_FAILED", note)
        else:
            self.backup_plan = (self.p("backup.after_align"), "REFINE_TARGET")
            self.go("BACKUP", "retry", "ALIGN_FAILED", note)

    def enter_final_align_box(self):
        self._align_enter("box")
        self.s["retries"] = self.retries.get("final_align_box", 0)

    def tick_final_align_box(self):
        def done(bx, by):
            self.go("PLACE", "success", note=f"box frozen in odom ({self.frozen[0]:.3f}, {self.frozen[1]:.3f}, "
                                             f"yaw {self.frozen[2]:.3f}), base ({bx:.3f}, {by:+.3f})")
        self._align_tick("box", self.p("box.align_x"), self.p("box.final_x"), done, self.final_align_box_failed)

    def final_align_box_failed(self, note):
        self.io.cmd(0.0, 0.0)
        if self.bump("final_align_box") > self.p("retries.final_align_box"):
            self.fail("FINAL_ALIGN_BOX_FAILED", note)
        else:
            self.backup_plan = (self.p("backup.after_align"), "FINAL_ALIGN_BOX")
            self.go("BACKUP", "retry", "ALIGN_FAILED", note)

    # ------------------------------------------------------------------ BACKUP (helper state)
    def enter_backup(self):
        self.s["start"] = None

    def tick_backup(self):
        dist, nxt = self.backup_plan
        r = self.io.robot_pose("odom")
        if r is None:
            self.io.cmd(0.0, 0.0)
            return
        if self.s["start"] is None:
            self.s["start"] = r
        moved = math.hypot(r[0] - self.s["start"][0], r[1] - self.s["start"][1])
        if moved >= dist:
            self.io.cmd(0.0, 0.0)
            self.go(nxt, "success", note=f"backed up {moved:.3f} m")
            return
        self.io.cmd(-self.p("backup.speed"), 0.0)

    # ------------------------------------------------------------------ PICK
    def enter_pick(self):
        n = self.bump("pick_attempts")
        self.s["retries"] = n - 1
        x, y, _, z = self.frozen
        self.op = self.io.pick(x, y, z)

    def tick_pick(self):
        if not self.op.done:
            return
        if self.op.ok:
            self.holding = True
            self.go("NAVIGATE_HOME", "success", note=self.op.message)
            return
        name = self.op.status if self.op.code is None else PICK_CODES.get(self.op.code, str(self.op.code))
        if self.op.code in (UNREACHABLE, GRASP_NOT_DETECTED) and \
                self.retries["pick_attempts"] < self.p("retries.pick_attempts"):
            self.backup_plan = (self.p("backup.after_pick"), "REFINE_TARGET")
            self.go("BACKUP", "retry", f"PICK_{name}", self.op.message)
            return
        self.fail(f"PICK_{name}", self.op.message)

    # ------------------------------------------------------------------ NAVIGATE_HOME / FIND_BOX
    def enter_navigate_home(self):
        self.io.detector("target", False)
        self.s["tries"] = 0
        self.op = self.io.nav(*self.p("home"))

    def tick_navigate_home(self):
        if not self.op.done:
            return
        if self.op.ok:
            self.go("FIND_BOX", "success")
            return
        self.s["tries"] += 1
        self.s["retries"] = self.s["tries"]
        if self.s["tries"] > self.p("retries.navigate_home"):
            self.fail("NAVIGATE_HOME_FAILED", f"nav {self.op.status}")
            return
        self.io.log("warn", f"NAVIGATE_HOME: nav {self.op.status}; clearing costmaps, retry {self.s['tries']}")
        self.io.clear_costmaps()
        self.op = self.io.nav(*self.p("home"))

    def enter_find_box(self):
        self.io.detector("box", True)
        self.io.reset_tracks("box")
        self.s["rotated"] = 0.0
        self.phase("dwell")

    def tick_find_box(self):
        t = self.found("box", "box.found_count", "box.found_std")
        if t is not None:
            if self.op is not None and not self.op.done:
                self.op.cancel()
            self.box_map = (t.x, t.y)
            self.go("APPROACH_BOX", "success", note=f"box at ({t.x:.2f}, {t.y:.2f}), {t.count} obs, std {t.std:.3f}, "
                                                    f"after {self.s['rotated']:.2f} rad")
            return
        if self.ph == "dwell":
            if self.in_phase() >= self.p("box.find_dwell"):
                if self.s["rotated"] >= 2 * math.pi - 1e-3:
                    self.fail("BOX_NOT_FOUND", "full turn without a box track")
                    return
                self.op = self.io.spin(self.p("box.find_step"))
                self.phase("spin")
        elif self.ph == "spin" and self.op.done:
            self.s["rotated"] += self.p("box.find_step")
            self.phase("dwell")

    # ------------------------------------------------------------------ PLACE / FINISH
    def enter_place(self):
        n = self.bump("place")
        self.s["retries"] = n - 1
        self.io.detector("target", True)       # the place server confirms the release with it
        x, y, yaw, floor_z = self.frozen          # box detection z = inside floor surface
        z = floor_z + self.p("target.mid_z") + self.p("box.place_clearance")
        self.op = self.io.place(x, y, z, yaw)

    def tick_place(self):
        if not self.op.done:
            return
        if self.op.ok:
            self.holding = False
            self.go("STOW_AND_FINISH", "success", note=self.op.message)
            return
        name = self.op.status if self.op.code is None else PLACE_CODES.get(self.op.code, str(self.op.code))
        if self.op.code != RELEASE_NOT_CONFIRMED and self.retries["place"] <= self.p("retries.place"):
            self.go("PLACE", "retry", f"PLACE_{name}", self.op.message)
            return
        self.fail(f"PLACE_{name}", self.op.message)

    def enter_stow_and_finish(self):
        # the arm is already stowed by the place action (SPEC §11.6 step 5)
        self.s["start"] = None

    def tick_stow_and_finish(self):
        r = self.io.robot_pose("odom")
        if r is None:
            return
        if self.s["start"] is None:
            self.s["start"] = r
        moved = math.hypot(r[0] - self.s["start"][0], r[1] - self.s["start"][1])
        if moved >= self.p("backup.finish"):
            self.io.cmd(0.0, 0.0)
            self.go("MISSION_COMPLETE", "success", note=f"backed up {moved:.3f} m")
            return
        self.io.cmd(-self.p("backup.speed"), 0.0)

    # ------------------------------------------------------------------ terminal states
    def _finish(self, success):
        self.io.cancel_all()
        self.io.detector("target", False)
        self.io.detector("box", False)
        t0 = self.t_start if self.t_start is not None else self.now
        self.result = {"trial_id": self.p("trial_id"), "success": success, "reason": self.reason,
                       "final_state": self.state, "holding": self.holding,
                       "mission_time": round(self.now - t0, 1),
                       "states": [f"{s}:{o}" + (f":{c}" if c else "") for s, o, c in self.history]}
        self.io.publish_result(self.result)
        self.s["zeros"] = 0

    def enter_mission_complete(self):
        self._finish(True)

    def tick_mission_complete(self):
        self._zero_tick()

    def enter_failed(self):
        self._finish(False)
        if self.holding:
            # In transport the arm is already at stow (stow == transport, DC-25) and the gripper
            # stays closed; a failed pick/place stows the arm itself. Nothing to command here.
            self.io.log("warn", "FAILED while holding the object: gripper kept closed, arm in transport (= stow)")
        self.io.log("error", f"MISSION FAILED: {self.reason}")

    def tick_failed(self):
        self._zero_tick()

    def _zero_tick(self):
        if self.s.get("zeros", 5) < 5:            # SPEC: zero velocity x5
            self.io.cmd(0.0, 0.0)
            self.s["zeros"] += 1
