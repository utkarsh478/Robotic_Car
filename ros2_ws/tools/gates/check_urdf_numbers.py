#!/usr/bin/env python3
"""Phase-1 gate helper: assert the generated URDF matches docs/SPEC.md §3.1-3.2.

Usage:
  python3 tools/gates/check_urdf_numbers.py                 # runs xacro on the default model
  python3 tools/gates/check_urdf_numbers.py /tmp/r.urdf     # checks an already-expanded URDF

Exit code 0 = all PASS, 1 = at least one FAIL.
Pure Python (no numpy) so it runs anywhere ROS 2 runs.
"""
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

TOL = 1e-6
DEFAULT_XACRO = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "src", "my_robot_description", "urdf", "my_robot.urdf.xacro")

results = []


def check(name, ok, detail=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  -> ' + detail) if detail else ''}")


def close(a, b, tol=TOL):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def vec(text, n=3):
    return [float(v) for v in (text or " ".join(["0"] * n)).split()]


# ---------- 3x3 / homogeneous helpers ----------
def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def compose(T1, T2):
    R1, t1 = T1
    R2, t2 = T2
    return matmul(R1, R2), [a + b for a, b in zip(matvec(R1, t2), t1)]


def load_urdf(path):
    if path.endswith(".xacro"):
        try:
            out = subprocess.run(["xacro", path], check=True, capture_output=True, text=True).stdout
        except FileNotFoundError:
            sys.exit("xacro not found: source /opt/ros/$ROS_DISTRO/setup.bash first")
        return ET.fromstring(out)
    return ET.parse(path).getroot()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_XACRO
    root = load_urdf(path)

    links = {l.get("name"): l for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    parent_of = {j.find("child").get("link"): j for j in joints.values()}

    def origin(j):
        o = j.find("origin")
        xyz = vec(o.get("xyz") if o is not None else None)
        rpy = vec(o.get("rpy") if o is not None else None)
        return rpy_to_R(*rpy), xyz

    def tf_from_root(link):
        chain = []
        while link in parent_of:
            j = parent_of[link]
            chain.append(j)
            link = j.find("parent").get("link")
        T = ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
        for j in reversed(chain):
            T = compose(T, origin(j))
        return link, T

    def mass(name):
        m = links[name].find("inertial/mass")
        return float(m.get("value")) if m is not None else 0.0

    # ---- tree ----
    roots = [l for l in links if l not in parent_of]
    check("single root is base_footprint", roots == ["base_footprint"], str(roots))
    expected_links = {"base_footprint", "base_link", "left_wheel_link", "right_wheel_link",
                      "front_caster_link", "rear_caster_link", "lidar_link", "imu_link",
                      "camera_mast_link", "camera_link", "camera_optical_link"}
    missing = expected_links - set(links)
    check("all SPEC §4 base/sensor links present", not missing, f"missing {sorted(missing)}" if missing else "")

    # ---- base ----
    _, (_, t) = tf_from_root("base_link")
    check("base_link 0.075 above base_footprint", close(t, [0, 0, 0.075]), str(t))

    _, (_, tl) = tf_from_root("left_wheel_link")
    _, (_, tr) = tf_from_root("right_wheel_link")
    sep = tl[1] - tr[1]
    check("wheel separation 0.38", abs(sep - 0.38) < TOL, f"{sep:.4f}")
    check("wheel axle height 0.075", abs(tl[2] - 0.075) < TOL and abs(tr[2] - 0.075) < TOL)
    for side in ("left", "right"):
        geom = links[f"{side}_wheel_link"].find("collision/geometry")
        shape = geom.find("sphere") if geom.find("sphere") is not None else geom.find("cylinder")
        check(f"{side} wheel collision radius 0.075 ({shape.tag})", abs(float(shape.get("radius")) - 0.075) < TOL)
        vis = links[f"{side}_wheel_link"].find("visual/geometry/cylinder")
        check(f"{side} wheel visual cylinder radius 0.075", vis is not None and abs(float(vis.get("radius")) - 0.075) < TOL)
        j = joints[f"{side}_wheel_joint"]
        check(f"{side}_wheel_joint continuous, axis +y",
              j.get("type") == "continuous" and close(vec(j.find("axis").get("xyz")), [0, 1, 0]))

    for name in ("front", "rear"):
        _, (_, tc) = tf_from_root(f"{name}_caster_link")
        r = float(links[f"{name}_caster_link"].find("collision/geometry/sphere").get("radius"))
        check(f"{name} caster touches ground", abs(tc[2] - r) < TOL, f"bottom at {tc[2] - r:+.4f}")

    # ---- sensors (positions in base_footprint) ----
    expect_pos = {
        "lidar_link": [0.27, 0.0, 0.12],
        "imu_link": [0.0, 0.0, 0.125],
        "camera_mast_link": [0.20, -0.12, 0.32],      # DC-40: top 0.455, not 0.485
        "camera_link": [0.20, -0.12, 0.50],
        "camera_optical_link": [0.20, -0.12, 0.50],
    }
    for name, p in expect_pos.items():
        _, (_, t) = tf_from_root(name)
        check(f"{name} at {p}", close(t, p, 1e-4), "[" + ", ".join(f"{v:.4f}" for v in t) + "]")

    # Camera optical axis (+z of optical frame) must point forward and 0.60 rad down.
    _, (R, _) = tf_from_root("camera_optical_link")
    z_axis = [R[0][2], R[1][2], R[2][2]]
    x_axis = [R[0][0], R[1][0], R[2][0]]
    want_z = [math.cos(0.60), 0.0, -math.sin(0.60)]
    check("optical z = forward, pitched 0.60 rad down", close(z_axis, want_z, 1e-6),
          "[" + ", ".join(f"{v:.4f}" for v in z_axis) + "]")
    check("optical x = robot right (-y)", close(x_axis, [0, -1, 0], 1e-6),
          "[" + ", ".join(f"{v:.4f}" for v in x_axis) + "]")

    # The mast's top cap must stay below the camera's view. Gazebo's RGB-D camera does not clip it
    # at 0.1 m, so a cap just under the lens fills every pixel below its rim (DC-40).
    mast = links["camera_mast_link"].find("visual/geometry/cylinder")
    _, (_, tm) = tf_from_root("camera_mast_link")
    _, (_, tcam) = tf_from_root("camera_link")
    mast_top = tm[2] + float(mast.get("length")) / 2.0
    rim_down = math.atan2(tcam[2] - mast_top, float(mast.get("radius")))
    cam = next(s for s in root.iter("sensor") if s.get("type") in ("rgbd_camera", "depth_camera", "camera"))
    hfov = float(cam.find("camera/horizontal_fov").text)
    w, h = float(cam.find("camera/image/width").text), float(cam.find("camera/image/height").text)
    view_down = 0.60 + math.atan(math.tan(hfov / 2.0) * h / w)
    check("camera mast top outside the camera view", rim_down > view_down + math.radians(5),
          f"mast rim {math.degrees(rim_down):.1f} deg down, image bottom {math.degrees(view_down):.1f} deg down")

    # ---- lidar self-hit geometry: rays at +/-90 deg leave in front of the chassis front ----
    chassis = links["base_link"].find("collision/geometry/box")
    chassis_front = float(chassis.get("size").split()[0]) / 2.0
    _, (_, tl) = tf_from_root("lidar_link")
    check("lidar ahead of chassis front (no self-hits at +/-90 deg)", tl[0] > chassis_front,
          f"lidar x {tl[0]:.3f} vs chassis front {chassis_front:.3f}")

    sensors = {s.get("name"): s for s in root.iter("sensor")}

    # ---- masses ----
    expect_mass = {"base_link": 10.0, "left_wheel_link": 0.5, "right_wheel_link": 0.5,
                   "front_caster_link": 0.1, "rear_caster_link": 0.1, "camera_mast_link": 0.2}
    for name, m in expect_mass.items():
        check(f"mass {name} = {m}", abs(mass(name) - m) < TOL, f"{mass(name)}")
    total = sum(mass(n) for n in links)
    has_arm = "arm_base_link" in links
    want_total = 11.61 + (1.95 if has_arm else 0.0)
    check(f"total mass {want_total:.2f} kg ({'with' if has_arm else 'without'} arm)",
          abs(total - want_total) < 1e-3, f"{total:.3f}")

    # ---- arm + gripper (SPEC §3.3-3.4), only when the arm is in the model ----
    if has_arm:
        arm_links = {"arm_base_link": 0.3, "shoulder_link": 0.3, "upper_arm_link": 0.5, "forearm_link": 0.4,
                     "wrist_link": 0.15, "gripper_base_link": 0.2, "left_finger_link": 0.05, "right_finger_link": 0.05}
        for n, m_ in arm_links.items():
            check(f"mass {n} = {m_}", n in links and abs(mass(n) - m_) < TOL, f"{mass(n) if n in links else 'missing'}")
        spec = {  # joint: (type, axis, origin xyz, lower, upper, effort, velocity)
            "arm_yaw_joint": ("revolute", [0, 0, 1], [0, 0, 0.04], -2.9, 2.9, 10, 1.5),
            "arm_shoulder_joint": ("revolute", [0, 1, 0], [0, 0, 0.06], -1.0, 2.0, 20, 1.0),
            "arm_elbow_joint": ("revolute", [0, 1, 0], [0, 0, 0.28], -2.6, 2.6, 12, 1.2),
            "arm_wrist_joint": ("revolute", [0, 1, 0], [0, 0, 0.24], -2.0, 2.0, 5, 1.5),
            "left_finger_joint": ("prismatic", [0, 1, 0], [0, 0, 0.02], 0.0, 0.04, 20, 0.05),
            "right_finger_joint": ("prismatic", [0, -1, 0], [0, 0, 0.02], 0.0, 0.04, 20, 0.05),
        }
        for jn, (typ, ax, xyz, lo, hi, eff, vel) in spec.items():
            j = joints.get(jn)
            if j is None:
                check(f"joint {jn} present", False)
                continue
            lim = j.find("limit")
            got = (j.get("type"), vec(j.find("axis").get("xyz")), vec(j.find("origin").get("xyz")),
                   float(lim.get("lower")), float(lim.get("upper")), float(lim.get("effort")), float(lim.get("velocity")))
            ok = (got[0] == typ and close(got[1], ax) and close(got[2], xyz)
                  and all(abs(a - b) < TOL for a, b in zip(got[3:], (lo, hi, eff, vel))))
            check(f"joint {jn}: {typ}, axis {ax}, origin {xyz}, limits [{lo}, {hi}], {eff}/{vel}", ok,
                  "" if ok else f"got {got}")
        _, (_, t_tool) = tf_from_root("tool0")
        check("tool0 at zero config: base_footprint (0.05, 0, 0.915) = base_link (0.05, 0, 0.84)",
              close(t_tool, [0.05, 0.0, 0.915], 1e-4), "[" + ", ".join(f"{v:.4f}" for v in t_tool) + "]")
        # fingertips 0.03 beyond tool0, inner face at |y| = q (visual box offset +-0.005, width 0.01)
        for side, sign in (("left", 1), ("right", -1)):
            o = links[f"{side}_finger_link"].find("collision/origin")
            sz = vec(links[f"{side}_finger_link"].find("collision/geometry/box").get("size"))
            inner = vec(o.get("xyz"))[1] - sign * sz[1] / 2
            tip = 0.02 + vec(o.get("xyz"))[2] + sz[2] / 2      # along approach axis from gripper_base_link
            check(f"{side} finger inner face at q (offset {inner:+.3f}) and tip 0.08 from gripper base",
                  abs(inner) < TOL and abs(tip - 0.08) < TOL, f"tip {tip:.3f}")
        sensors_ok = all(s_ in sensors for s_ in ("left_finger_contact", "right_finger_contact"))
        check("finger contact sensors declared", sensors_ok)
        plugins = [p_.get("name") for p_ in root.iter("plugin")]
        check("DetachableJoint plugin declared (parent wrist_link)",
              any("DetachableJoint" in (n or "") for n in plugins)
              and any(e.text == "wrist_link" for e in root.iter("parent_link")))

    # ---- every link that has an inertial has positive diagonal terms >= 1e-5 ----
    bad = []
    for n, l in links.items():
        I = l.find("inertial/inertia")
        if I is not None and min(float(I.get(k)) for k in ("ixx", "iyy", "izz")) < 1e-5 - 1e-12:
            bad.append(n)
    check("all inertia diagonals >= 1e-5", not bad, str(bad) if bad else "")

    # ---- gazebo sensors declared with correct frame ids ----
    for sname, frame in (("lidar", "lidar_link"), ("imu", "imu_link"), ("camera", "camera_optical_link")):
        s = sensors.get(sname)
        for tag in ("gz_frame_id", "ignition_frame_id"):
            el = s.find(tag) if s is not None else None
            fid = el.text if el is not None else None
            check(f"gz sensor '{sname}' {tag} = {frame}", fid == frame, str(fid))

    n_fail = results.count(False)
    print(f"\nSUMMARY: {len(results) - n_fail}/{len(results)} PASS")
    sys.exit(1 if n_fail else 0)


if __name__ == "__main__":
    main()
