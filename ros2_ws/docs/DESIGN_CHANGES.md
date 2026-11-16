# DESIGN_CHANGES — deviations from docs/SPEC.md

Each entry: what changed, why, evidence. Newest last.

## DC-1 (P0) Stack: Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress
- **SPEC:** Ubuntu 24.04 + Jazzy + Harmonic (§1); 22.04 was a "stop and ask" case.
- **Change:** the user chose to stay on Humble + Fortress (option A, 2026-10-20).
- **Why:**
  - the machine already runs Humble, and other Humble workspaces depend on it;
  - only 4 packages were missing;
  - Fortress is the only Gazebo version with official Humble binaries for ros_gz;
  - Humble and Fortress are both supported until May 2027.
- **Evidence:** `docs/ENVIRONMENT.md` (P0 run).
- **Consequences, applied from now on:**
  - The CLI is `ign gazebo` / `ign topic`, not `gz sim` / `gz topic`.
  - Gazebo plugin filenames are `ignition-gazebo-*-system`, class names `ignition::gazebo::systems::*`.
  - Bridge message types are `ignition.msgs.*`.
  - Env vars: `IGN_GAZEBO_RESOURCE_PATH` and `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`. The `GZ_*` ones are set too.
  - Sensor frame ids: `<ignition_frame_id>` is added next to `<gz_frame_id>`. Each Gazebo version ignores the one it doesn't know.

## DC-2 (P2) cmd_vel is geometry_msgs/Twist (unstamped), not TwistStamped
- **SPEC:** §5/§6 say TwistStamped (the Jazzy default).
- **Change:** `use_stamped_vel: false`. The controller listens on `~/cmd_vel_unstamped`, which is remapped to `/cmd_vel` through the gz_ros2_control `<ros><remapping>`. `~/odom` is remapped to `/odom`.
- **Why:**
  - Humble Nav2 publishes plain Twist;
  - `teleop_twist_keyboard` on Humble has no `stamped` option;
  - Humble's spawner has no `--controller-ros-args`.
- **Consequences:**
  - teleop is run with `-r cmd_vel:=/cmd_vel` and no `stamped` param;
  - mission FINAL_ALIGN publishes Twist;
  - Nav2 needs no stamped setting.

## DC-3 (P2) Bridge configured by launch arguments, not gazebo_bridge.yaml
- **SPEC:** §8.4 names `config/gazebo_bridge.yaml`.
- **Change:** the `parameter_bridge` topics are passed as `args` in `my_robot_gazebo.launch.xml`. Ground-truth odometry is remapped to `/evaluator/robot_odom`.
- **Why:** the `topic@ros_type[ign_type` argument form works on every Humble ros_gz release. The YAML key names changed between releases.

## DC-4 (P2) Ground truth comes from the OdometryPublisher system
- **SPEC:** §8.2 says pose-publisher system.
- **Change:** the robot carries `ignition-gazebo-odometry-publisher-system`, publishing `/model/my_robot/odometry`, which is bridged to `/evaluator/robot_odom`.
- **Why:** one plugin gives a ready-to-use nav_msgs/Odometry, with no Pose_V parsing.
- **Note:** gates compare displacements only, so its origin convention doesn't matter. Target and distractor poses get the same treatment in P3.

## DC-5 (P2) evaluator.py deferred to P3
- **Change:** in P2 the gate script reads `/evaluator/robot_odom` directly.
- **Why:** P2 has nothing to score yet. evaluator.py is created in P3 together with the world objects.

## Open item for P4
- **Footprint:** the lidar body reaches x = 0.305 (centre 0.27 + radius 0.035). The SPEC §10 footprint front is 0.29.
- **Action:** set the front edge to 0.31 when writing the Nav2 params.

## DC-6 (P1 fix) robot_state_publisher is launched from Python (rsp.launch.py)
- **SPEC / CLAUDE.md:** use XML launch files by default.
- **Change:** `my_robot_description/launch/rsp.launch.py` starts robot_state_publisher. Both XML launch files include it.
- **Why:** Humble's XML `<param value="$(command 'xacro …')">` parses the URDF as YAML. That fails on any comment containing `word:`.
- **Evidence:** the P1 run on 2026-10-20 failed with `Failed to convert … using yaml rules: mapping values are not allowed here`.
- **Fix:** `ParameterValue(Command(...), value_type=str)` passes the URDF as a plain string.

## DC-7 (P1 fix) Gate scripts don't use `set -u`
- **Why:** colcon's `install/setup.bash` reads `COLCON_TRACE`, which is unset. Under nounset this aborted gate_p01 right after the build (2026-10-20 run).

## DC-8 (P2) Wheel collision is a sphere, not a cylinder
- **Evidence (gate_p02, 2026-10-20):**
  - The straight drive matched truth exactly (1.000 m vs 1.000 m).
  - A 2π rotation by odometry was **7.003 rad** by ground truth (+11 %), with position drift 0.001 m.
  - Back-calculating gives an effective wheel separation of 0.38 × 6.285 / 7.003 = 0.341 m. That is the wheels' inner-rim distance (2 × (0.19 − 0.02) = 0.34). DART was putting the cylinder–plane contact on the inner edge.
- **Change:** wheel `<collision>` is now a sphere of r 0.075 centred on the wheel. Visual and inertia are unchanged.
- **Rejected alternative:** setting `wheel_separation: 0.341`. It only compensates a simulator quirk, and it would change with load (the arm in P5).
- **Side effect:** the collision sphere reaches |y| = 0.265 at axle height (z 0.075). SPEC §10 footprint half-width 0.22 → **0.27** in P4. Together with the lidar open item, the P4 footprint is `[[0.31, 0.27], [0.31, -0.27], [-0.27, -0.27], [-0.27, 0.27]]`.

## DC-9 (P3) Saved map frame = Gazebo world frame
- slam_toolbox starts the map frame at the robot's first pose, which is the home pose (−3, −2, 0).
- `tools/save_map.sh` saves `mission_world_raw.*`. `tools/map_to_world.py` then adds (−3, −2) to the map origin and writes `mission_world.*`.
- **Result:** Nav2/AMCL in P4 use the SPEC world coordinates directly (home, search waypoints, T1–T3). The AMCL initial pose is (−3, −2, 0) in `map`.
- **Offline check:** a synthetic map passes `check_map.py` 5/5 after the correct shift and 0/5 after the wrong one.

## DC-10 (P3) Mapping drive steers on ground truth (gate tooling only)
- `tools/gates/map_drive.py` follows a fixed route using `/evaluator/robot_odom`. It is a gate script, which rule 4 allows. The SLAM map is still built only from `/scan` + `/odom`.
- The route keeps ≥ 0.50 m from every obstacle (checked against the world file) and spins 360° five times so the front-only lidar covers the room.

## DC-11 (P3) mission_sim.launch.xml added
- New launch layer: mission world, robot at home, target/distractor spawned with `ros_gz_sim create -file`, plus evaluator.
- `mapping.launch.xml` uses it with `spawn_objects:=false`.
- The target/distractor carry OdometryPublisher → `/evaluator/{target,distractor}_odom`.

## DC-12 (P3 fix) Gates run isolated from other simulations
- **Evidence (gate_p03, 2026-10-22, 2nd run):** a GUI mission_sim was open while the gate ran.
  - Ground truth jumped back to the spawn point (−3.00, −2.00) right after the robot had reached (−2.55, −2.00), with a 89° yaw error.
  - `t_sim` advanced erratically (the first 0.45 m leg finished at 40 s instead of 7 s).
  - The GUI robot was found tipped over.
- **Cause:** both sims shared `/cmd_vel` (the gate drove both robots), `/scan` (SLAM fused two robots' scans), `/clock` and the ground-truth topics. This also likely explains the first run's folded map (only world x ≤ 1.19 mapped, 408 free cells outside the walls).
- **Change:** every gate sources `tools/gates/gate_env.sh`, which:
  - sets `ROS_DOMAIN_ID=77` and a per-run `IGN_PARTITION`/`GZ_PARTITION`;
  - refuses to start if another Gazebo is running (`ALLOW_OTHER_SIM=1` overrides).
- gate_p03 also deletes the old map before mapping, so a failed drive can't re-score a stale map.
- **Added diagnostics:**
  - map_drive logs SLAM pose vs ground truth (`results/p03_slam_drift.csv`, PASS if ≤ 0.15 m);
  - `tools/render_map.py` draws the map with the true walls overlaid (`results/p03_map.png`).

## DC-13 (P3) slam_toolbox: loop closure off, scan matching tied to odometry
- **Evidence (gate_p03 run 3, 2026-10-22, isolated):**
  - The SLAM-vs-truth error grew 0.001 → 0.121 m, mostly on the y-corridor legs (wp 3→5, 8→11).
  - At t_sim 141 s, near (2.95, 1.78), SLAM jumped to (0.59, 0.29): 2.8 m with a 2° yaw error. That is a false loop closure between look-alike corridors (right zone ↔ middle zone). The map image shows right-zone streaks over the middle zone.
- **Changes:**
  - `do_loop_closing: false`;
  - `correlation_search_space_dimension` 0.5 → 0.3;
  - `distance_variance_penalty` 0.5 → 0.01, `angle_variance_penalty` 1.0 → 0.05;
  - `minimum_distance_penalty` 0.5 → 0.2, `minimum_angle_penalty` 0.9 → 0.5.
- **Why:**
  - Sim wheel odometry is near-exact (P2), and a 180° lidar is degenerate along corridor axes.
  - The defaults barely penalise leaving odometry: a 10 cm offset costs 0.4 % of the match score. It now costs 20 %.
- **For hardware:** re-tune these, since real odometry drifts. Loop closure is worth re-enabling on large sites.

## DC-14 (P3) Mapping drive waits until SLAM is integrating scans
- **Evidence (gate_p03 run 4, 2026-10-22):**
  - SLAM accuracy is now good: max 0.035 m / 0.9° vs truth; W1, W2 and the north/east walls are 100 % mapped.
  - The home zone stayed unknown although the robot spun inside it. The unknown region's edges match the lidar's FOV from the wp2 leg, so scans before about wp2 were never integrated.
  - Knock-on effects: the home cell was unknown, the west wall 49.8 %, home_box 50 %, O3 57 %.
- **Change:**
  - `map_drive.py` waits until `/map` has ≥ 50 occupied cells and TF map→base_footprint exists, then settles 3 s before moving.
  - gate_p03 reports the slam_toolbox "Message Filter dropping" count as evidence.

## DC-15 (P3) DC-14's hypothesis disproved; route gets a return leg plus map snapshots
- **Evidence (gate_p03 run 5, 2026-10-22):**
  - SLAM was ready 3.1 s after launch, with 137 occupied cells from the first scan, and **0** "Message Filter dropping" lines.
  - The home zone is still unknown, in the same shape as before. So the scans were not lost at startup: the first scan's cells were in `/map` and later absent from the final map.
  - SLAM accuracy: max 0.047 m / 0.7°.
- **Change:**
  - `map_drive.py` route extended by a return leg: back through both gaps, a spin west of O1 (its west face), then driving south-west through the home zone and spinning at (−3.3, −2.4). Total 37.5 m, 7 spins, all legs ≥ 0.37 m clear (checked against the world file).
  - Live-map snapshots are saved at start, after the home spin, after wp3, at the end of the outbound route, after the return home spin, and final, to `results/p03_snapshots/*.png`. They show whether the home zone appears and later disappears, which would mean a slam_toolbox map-rendering issue, not coverage.

## DC-16 (P3 result) Gate passed; first-spin observation left open
- **gate_p03 run 6, 2026-10-22: ALL PASS.**
  - SLAM vs truth over 37.5 m: max 0.045 m / 1.0°.
  - Walls 100 % (all six), obstacles 85.7 %, home_box 86.4 %, 0 free cells outside, home cell free.
- **Snapshot evidence on the home corner:**
  - The map stayed 47 × 121 cells through the initial 360° spin at (−2.55, −2.00); occupied cells went 139 → 219 but the map never grew west or south.
  - After wp3 the corner is unknown, with the same outline as runs 4–5. It is filled only by the return leg (snapshot 32).
  - So the corner was never integrated, rather than integrated and later lost. Later spins *did* contribute (the spin west of O1 raised O1 from 78.8 % to 88.8 %).
- **Open (not root-caused):** why scans from the first in-place spin, while the pose graph has only one node, aren't added. The return leg makes coverage independent of it.
- **For P4:** AMCL starts at home, which is mapped now. No action needed.

## DC-17 (P4) Nav2 configuration decisions
- **Params are generated, not hand-written:**
  - `tools/make_nav2_params.py` loads the installed `nav2_bringup/params/nav2_params.yaml` and applies 26 explicit overrides. It stops if a node or section path doesn't exist in the installed default.
  - gate_p04 regenerates the file every run.
  - Checked offline against the Humble branch default: 0 path errors.
- **Footprint** `[[0.31, 0.27], [0.31, -0.27], [-0.27, -0.27], [-0.27, 0.27]]`, replacing `robot_radius` (DC-8 + lidar open item).
- **Local costmap:** a 2D obstacle layer replaces the default voxel layer (single 2D lidar). Inflation radius is 0.45 m in both costmaps.
- **Controller:** RPP (SPEC §10) replaces the Humble default DWB. Goal tolerance 0.10 m / 0.10 rad. Velocity smoother 0.4 m/s, 1.0 rad/s.
- **cmd_vel:** Humble `navigation_launch.py` chains controller → `cmd_vel_nav` → velocity_smoother → `/cmd_vel` (unstamped Twist, matching DC-2). No extra remap is needed.
- **AMCL** starts at the home pose (−3, −2, 0) via `set_initial_pose`, since map == world (DC-9).
- **Deviation from SPEC §10:** the Humble default BT backs up 0.30 m (SPEC: ≤ 0.15 m, rear lidar-blind). It's kept, because the behavior server checks the backup against the costmap, whose static layer covers the rear. A custom BT is only needed if dynamic obstacles behind the robot become a requirement.
- **Collision scoring:**
  - gates test the robot's true pose (chassis box, wheel spheres, lidar puck) against the world-file obstacle boundaries every 0.5 s. This replaces SPEC §8.4's chassis contact sensor and avoids depending on lumped-collision names.
  - The blocked-path test spawns two 0.3 m blocks in the top gap, leaving 0.3 m slots against a robot width of 0.54 m.

## DC-18 (P4) AMCL tuned for simulation accuracy
- **Evidence (gate_p04 run 1, 2026-10-22):**
  - 10/10 goals succeeded, 0 contacts, robot stopped after every goal, and the blocked path passed 4/4 (Nav2 aborted after 25 s).
  - But AMCL vs truth: mean 0.098 m, max 0.253 m and 0.149 rad (358 samples).
  - The final goal errors (0.016–0.243 m) track AMCL's error, because Nav2 stops when AMCL says the robot is within 0.10 m.
- **Cause:** Humble AMCL defaults assume noisy real-world odometry and a loose laser model (alpha 0.2, sigma_hit 0.2, z_rand 0.5, 60 beams, updates every 0.25 m / 0.2 rad). Here the map is within 4.5 cm and sim odometry is near-exact.
- **Change:**
  - alpha1–4 → 0.05, sigma_hit → 0.08;
  - z_hit/z_rand/z_max → 0.9/0.05/0.05;
  - max_beams → 120;
  - update_min_d/a → 0.10 m / 0.10 rad.
  - nav_runner also writes `results/p04_amcl_err.csv`.
- **For hardware:** raise alpha back toward 0.2 once real odometry is measured.
- **Note on the blocked-path test:** the blocks were within the 5 m obstacle range from home, so Nav2 saw them before leaving and aborted early. The test still proves "no route → abort safely", but not discovery while driving.

## DC-19 (P5) Grasp attach: Fortress attaches at startup, so a manager owns the state
- **Evidence:** the Fortress 6.18.0 source (tag `ignition-gazebo6_6.18.0`, `DetachableJoint.hh`) has `attachRequested{true}`. The joint is created as soon as both models exist. `initial_attach` exists only on the branch after 6.18.0. Attach, detach and state topics *are* in 6.18 (re-attach landed in 6.15).
- **Consequence:** without handling, the target would be welded to the gripper from spawn.
- **Change:** `my_robot_bringup/scripts/grasp_attach_manager.py`, started with the arm.
  - Desired state starts DETACHED.
  - It sends the corrective command every 0.25 s until Gazebo's `/gripper/attach_state` matches.
  - It offers `/gripper/set_attached` (SetBool, returns after Gazebo confirms) and publishes `/gripper/attached` (latched Bool).
- `parent_link` is `wrist_link`: `gripper_base_link` is fixed to it and lumped away by URDF→SDF. gate_p05 checks this in the `ign sdf -p` output.

## DC-20 (P5) Grasp point = cylinder top − 0.025 m (SPEC §11.4 said − 0.04)
- With tool0 at top − 0.04, the gripper base plate (tool0 − 0.05 … − 0.03 along the approach) cuts 1 cm into the cylinder top.
- At top − 0.025 there is 5 mm clearance and the fingers still cover 5.5 cm of the 15 cm cylinder.
- P9's pick sequence uses the new value.

## DC-21 (P5) Arm/gripper control details
- `with_arm` now defaults to true in every launch file. `with_arm:=false` reproduces the P1–P4 robot (gate_p05 checks both URDF variants).
- Arm and finger joints use position command interfaces with initial values 0 (fingers 0.04 = open). gz_ros2_control turns position commands into velocity commands capped by the URDF effort, so a blocked finger squeezes with ≤ 20 N.
- `gripper_controller` (JTC) has goal tolerance 0 and goal_time 0 (= not checked), so a finger stalled on the object doesn't abort the goal. Grasp success is judged from finger positions + contacts (SPEC §11.5), never from the action result.
- `arm_controller` has goal tolerance 0.03 rad and goal_time 1.0 s.
- **Test poses** (gate_p05) come from the SPEC §11.1 IK and were checked through the URDF chain: tool0 error 0.0 mm, approach axis (0, 0, −1), finger gap 0.050 m at q 0.025.

## DC-22 (P5 fix) Gate cleanup kills any leftover simulator process
- **Evidence (2026-10-22):** gate_p05 refused to start because of `19775 ign gazebo server`, left over from a GATE_GUI=1 run. In GUI mode Gazebo forks a server that renames itself and survived the process-group kill. The old cleanup `pkill -f "ign gazebo -s -r.*world"` only matched the headless command line.
- **Change:** `gate_env.sh` defines `kill_gate_sims` (same pattern as the start-up guard), and every gate calls it on exit. This is safe because a gate never starts while any simulator runs, so everything alive at shutdown is the gate's own.
- **Tested here:** a process named `ign gazebo server` is killed, and a process merely containing `gate_p05.sh` is spared.

## DC-23 (P6) Analytic IK: four solution branches, physical approach pitch
- **Library:** `my_robot_manipulation`, `libarm_ik` (pure C++17, no ROS), plus `ik_cli` (used by the gate) and gtest.
- **Branches beyond SPEC §11.1, found by the random round-trip tests:**
  1. SPEC computes yaw = atan2(y, x) only, so it can't reach a point that only a backward-leaning arm reaches. IK now also tries yaw + π ("back", reaching over the yaw axis): 92 of 2000 random general-pitch poses needed it.
  2. Turning the yaw by π mirrors the arm plane, so the same physical approach direction has pitch −θ there. The wrist angle is also tried ± 2π.
  - Candidate order: front/elbow-up, front/elbow-down, back/elbow-up, back/elbow-down. The first within limits wins.
- **Input convention:** `ik(p, pitch)` takes the PHYSICAL approach pitch, measured in the vertical plane through the yaw axis and p, positive tilting outward. `fk()` reports pitch relative to the yaw joint. `outward_pitch()` converts. Top-down (π) is the same in both. The tests compare physical direction vectors, not pitch numbers.
- **Mutation tests (offline):**
  - removing the pitch mirror fails the general-pitch round trip;
  - removing the back branch fails both round trips.
- **Offline result:** 11/11 gtest PASS with `-Wall -Wextra -Wpedantic -Werror` (g++ 13). The Phase 5 grasp angles are reproduced to 1e-4 rad.
- **Measured reach:** top-down grasp at cylinder height is possible for base_link x ∈ [0.190, 0.565] straight ahead. The planned grasp at x 0.40 is near the middle.

## DC-24 (P7) MoveIt planning frame = base_footprint (no virtual joint); floor 2 mm below ground
- The SRDF has no virtual joint, so MoveIt's root is the URDF root `base_footprint`. Pick and place happen with the base stopped, so a scene object given in odom can be transformed once when it is added. A floating odom→base virtual joint would add a TF dependency and gain nothing.
- `robot_description` comes from the same xacro and `with_arm:=true` as Gazebo. move_group starts no robot_state_publisher (SPEC pitfalls 7–8; the gate checks for exactly one).
- Floor collision box (SPEC §11.2): 3 × 3 × 0.02, top at z −0.002 in base_footprint, not at 0. The wheel and caster spheres rest exactly on the ground, so a floor at 0 makes every start state "in collision".
- Wheels are `passive_joint`s. The gripper is in `moveit_controllers.yaml` so RViz can use it, but the pick pipeline still commands it directly (SPEC §11.3).

## DC-25 (P7) stow = transport = (0, −0.60, 1.85, 1.8916), not SPEC's guess (0, −0.5, 2.4, 1.24)
- `check_named_states.py` checks SPEC §3.5:
  - joint limits;
  - top-down;
  - tool0 x;
  - height;
  - footprint;
  - camera frustum;
  - clearance ≥ 0.02 m to the chassis, sensors and the robot's own links.
  It runs with fingers open, and again holding the target with the DC-20 grasp: fingers 0.025, cylinder top 0.025 above tool0.
- SPEC's guess fails while holding the target: the cylinder passes **0.018 m** from `arm_base_link` (needs 0.02).
- A grid search (yaw 0, top-down) gave 64 feasible states. The chosen one maximises the worst margin:
  - tool0 at base x 0.120, 0.482 m above ground;
  - min clearance 0.047 m (gripper–camera);
  - footprint margin 0.145 m;
  - entirely behind the camera (not in view);
  - object bottom 0.17 m above the chassis.
- **Cost:** the arm is folded higher (tool0 0.48 vs 0.34 m above ground). Its 1.6 kg raises the CG slightly; the P5 tipping check had the arm horizontal, which is worse.

## DC-26 (P7) ACM: own generator, stricter "Never" rule
- `gen_acm.py` re-implements Setup Assistant's classification (Adjacent / Default / Always / Never) in numpy, using the URDF collision shapes: sampled surfaces + exact signed distance, 3 mm padding.
- **Stricter than Setup Assistant:** a pair is "Never" only if it never collides in 10 000 samples **and** never comes closer than 3 cm.
- **Evidence:** with plain 0-collision "Never" (Setup Assistant's rule), `left_finger–rear_caster` was disabled but collided in 1 of 50 000 fresh samples. With the 3 cm margin:
  - 9 Never + 1 Default (fingers touch at q = 0) + 51 Adjacent are disabled;
  - 59 pairs are checked;
  - 0 collisions among the disabled pairs in 100 000 fresh samples (`--verify`, a different seed). The gate reruns this with 50 000.

## DC-27 (P7) The camera mast/camera block the arm's right-front workspace
- **Offline (geometry model, top-down IK):** 19 of the 189 P7 sweep goals are in collision. All are on the right side, y ≤ −0.10, x ≤ 0.425: the upper arm or forearm hits `camera_link`/`camera_mast_link` (at y −0.12), or at h 0.11 a finger hits the lidar near x 0.30.
- **Gate metric:** PHASES' "≥ 95 % of IK-feasible points plan" is applied to IK-feasible **collision-free** goals. Goals in collision are listed, not counted: no planner can reach them. The gate also checks that MoveIt's collision verdicts match the geometry model, except borderline cases under 3 mm.
- **Consequence for P9/P10:** the final approach must centre the object, |y| ≤ 0.05 (P9 gate: ±0.03). The band x 0.375–0.45, |y| ≤ 0.05 is collision-free at all three heights, and the gate checks that it plans 100 %.
- **Gazebo will not catch this:** the robot model has no `self_collide`, so in simulation the arm passes through its own camera without contact. MoveIt's collision check is the only protection, so every arm motion from P9 on must be planned or checked by MoveIt (SPEC §11.3 already requires this).
- **The one sweep target that didn't plan (both runs):** x 0.350, y −0.10, h 0.29 → `INVALID_MOTION_PLAN` after 0.017 s. The geometry model puts that goal only **4.4 mm** from `camera_link` (forearm). OMPL found a path, but MoveIt's final densely-sampled path check found a state touching the camera. This is a real narrow passage, not a configuration error. **P9/P10 rule:** reject IK goals (grasp, pre-grasp, place) whose clearance is under 1 cm. No point in the mission band came out under 1 cm in the offline check.

## DC-28 (P8) Target and box estimators differ from SPEC §9.1/9.2
- **Every mask pixel is back-projected, not just the centroid.** Pixels are taken from the blob eroded by 1 px, so silhouette edges with mixed depth are skipped. Points more than 8 cm (box: 40 cm) from the blob's median depth are dropped, which removes background seen through edges. The points are then transformed to `map` with one TF lookup per frame.
- **Target axis:** a known-radius circle fit on the side-surface points (z 0.01–0.14; the top cap is excluded).
  - Start value: the points' mean pushed π/4·r away from the camera. That is where the mean of a half circle sampled uniformly across the image lies.
  - If the fit's residual exceeds 0.2·r (noisy depth) or the fit moves more than r away, the start value is used instead.
- **Box:** yaw comes from the minimum-area rectangle of the green points. For each box axis the centre is taken as follows:
  - if the camera is outside the box's extent along that axis, the nearest **outer** face + 0.15;
  - otherwise the midpoint of the extent.
- **Offline evidence** (`test/synthetic.py` ray caster, real mounting, intrinsics, colours and the gate's poses):

  | Case | Error |
  |---|---|
  | SPEC centroid + median depth + push r (target) | **4.0–5.6 mm** systematic bias |
  | circle fit, clean depth | 0.0 mm |
  | circle fit, 1 % depth noise | ≤ 3.3 mm |
  | circle fit, 2 px RGB/depth misregistration + 1 % noise | ≤ 4.9 mm |
  | box, box yaw 0–1.2 rad, range 0.6–1.4 m | ≤ 0.7 mm |
- **False positives:**
  - Obstacle O1 (0.75, 0.45, 0.2) and the arm (0.95, 0.5, 0.1) are both hue 14, just 4 above the red limit of 10.
  - Every blob ≥ 150 px therefore goes through a 3-D shape check: z95 ≤ 0.19 m, not floating, 95 % within 2r + 4 cm of the median. The largest *plausible* blob wins, not simply the largest blob.
  - Red/orange boxes 0.5 m tall and a floating arm link are rejected in the tests.
- **`v_min` 70 → 50 (target):** in the renders the red cylinder's shadow side reaches only V 75. Hue and saturation still separate red from every other colour in the world.

## DC-29 (P8) Detector interfaces beyond SPEC
- `/perception/<label>_raw`: one Detection per frame (observation_count 1), for evaluation and debugging.
- `~/reset` (std_srvs/Trigger) clears the tracks. The gate uses it between poses; the mission can use it after a pick.
- Debug image per detector: `/perception/<label>/debug_image`, published only when someone subscribes. SPEC had a single `/perception/debug_image`, which the two detectors would have interleaved.
- The tracker keeps mean and std over the **last 30** observations, not all of them. Early far-away estimates stop counting once the robot is close; `observation_count` is still the total.
- `hue_ranges` is a flat list of [lo, hi] pairs, because ROS parameters can't be nested lists.

## DC-30 (P8) The perception gate uses a ground-truth map frame
- The gate teleports the robot to 18 poses with `/world/mission_world/set_pose`, so wheel odometry no longer matches the robot's real pose.
- `tools/gates/gt_localizer.py` publishes map→odom = ground-truth base pose · (odom→base)⁻¹, planar. It is gate-only (reads `/evaluator/robot_odom`) and lives outside `src/`.
- The measured error is therefore perception alone. In the mission AMCL adds its own error (P4: mean 3 cm), but the final approach works on the detection in `base_footprint`, where the localization error cancels.
- **Range and bearing in the gate are horizontal, from the camera.** "Range 0.6 m" measured from base_footprint would put a ±0.35 rad target outside the 1.2 rad field of view, because the camera sits 0.20 m forward and 0.12 m to the right.

## DC-31 (P8 fix) One camera subscription for both detectors
- **Evidence (first gate_p08 run, 2026-11-02):** whenever a frame was processed the result was excellent (target error ≤ 2.3 mm, red-mask V p5 114, S p5 185), and pytest passed on the 7 Gazebo frames saved. But per 3 s pose the target detector produced 0–7 estimates instead of ~45, and the box detector none at all in 4 poses.
- **Cause (most likely):** the image streams are ~2 MB per RGB+depth pair at 15 Hz. Each detector and the eval node subscribed to both streams with best-effort QoS, so DDS carried three copies of each. Large best-effort messages are dropped when the receive buffers overflow, and the synchroniser needs *both* images of a pair. P2's 15 Hz check had only one subscriber.
- **Change:**
  - `perception_node` runs both detector nodes in one process. Only `target_detector` subscribes and hands each decoded frame to `box_detector`. Names, parameters, services and topics are unchanged.
  - The eval node subscribes to the images only for the moment it saves a sample.
  - The detectors now log `synced image pairs received / frames processed / dropped (no TF)` every 10 s, and the gate prints those lines, so the next run tells transport loss and TF loss apart.
  - New gate check: ≥ 5 target estimates per second at every pose.
- Samples are now saved at every pose, including poses with no detection: those are exactly the frames needed for debugging.

## DC-32 (P8 fix 2) TF lookups: one listener, no spin thread, multi-threaded executor
- **Evidence (second gate_p08 run, 2026-11-14):**
  - The detectors received 52–67 synced image pairs per 10 s, and **every** processed frame was dropped with "no TF". 0 detections.
  - `perception_node` died with `IndexError: wait set index too big`.
  - The eval node saved only 8 of 20 samples.
- **Cause:** `TransformListener(buffer, node, spin_thread=True)` in Humble adds the *same* node to a second executor and spins it in its own thread. With the node also spun by the main executor, two threads raced on one wait set, which crashed the listener thread. After that, /tf was only processed by the main single-threaded executor. That executor was itself blocked inside `lookup_transform(..., timeout=0.1)` waiting for /tf, so every lookup at a fresh image stamp timed out. The same pattern caused the intermittent detections in the first run.
- **Fix:**
  - Only the subscribing detector owns a Buffer and TransformListener (no spin thread). It does one lookup per frame and passes the transform to its peers.
  - Image processing, reset and stats share one mutually exclusive callback group. The listener uses its own reentrant group, and `run()` uses a 4-thread `MultiThreadedExecutor`, so /tf keeps arriving while a frame waits for its transform.
  - The stats line now also prints the last TF error.
  - The eval node waits for a sample's transform by spinning (`can_transform` + `spin_once`) instead of blocking.

## DC-33 (P9) Pick server and gate
- **Server** (`pick_place_server.cpp`, C++17, compiled warning-free against ROS 2 Humble / MoveIt 2.5.9 before delivery). It follows SPEC §11.3–11.5 with these specifics:
  - **Clearance (DC-27):** pre-grasp and grasp states must keep ≥ 1 cm from the robot itself (FCL self-distance with the SRDF ACM) and from the world, otherwise `UNREACHABLE`. This is checked *before* the target is added to the scene, because the open fingers are meant to surround it.
  - **Vertical moves:** every waypoint must use the same IK branch, change no joint by more than 0.3 rad between waypoints, and pass MoveIt's collision check.
  - **Attached object:** a cylinder whose top is 0.025 above tool0 (DC-20), attached to `gripper_base_link`, with touch links gripper_base, both fingers and wrist.
  - **Re-verify after the lift:** the finger-contact window is 1.0 s instead of 0.5 s, the mission's "no contact for > 1.0 s" rule. Contact reports may thin out once the fixed joint carries the load; if they stop entirely, the gate shows `OBJECT_LOST` with the contact ages.
  - At the start of every pick, a stale `target` (attached or in the world) is removed from the scene.
  - Failure cleanup: a missed grasp opens the gripper, retracts vertically and stows, with no attach. An object lost after the lift is detached in Gazebo and in the scene, then the gripper opens and the arm stows.
- **Launch:** `manipulation.launch.py`, not `.xml`. It takes the robot description, SRDF, kinematics and joint limits from `move_group.launch.py`'s `moveit_config()`, so MoveIt and the server cannot disagree.
- **Gate:** PHASES teleports the robot so the target sits at base-x 0.40. The gate instead does what the mission will do:
  - teleport so the target is at base-x **0.55** (where the whole cylinder is in view, SPEC §3.2);
  - estimate the target with perception, publishing in the **odom** frame (`map_frame:=odom`, valid despite the teleport because odom→camera is robot-relative);
  - creep straight ahead on odometry to base-x 0.38–0.42;
  - send PickObject.

  So each pick also includes the perception estimate and the odometry creep. Lateral offsets range over ±0.03, headings over ±0.12 rad.
- **Gate checks:**
  - ≥ 9/10 picks: result success, ground-truth target lifted > 5 cm and Gazebo attach confirmed;
  - 3/3 missed grasps (target teleported 0.2 m away when the descent starts): `GRASP_NOT_DETECTED`, never attached, fingers open, arm at stow;
  - 3/3 hold tests: 2 m drive + 90° turn, object moves < 2 cm relative to tool0;
  - the target is not knocked before the fingers close (< 5 mm).

  Gazebo has no self-collision on the robot, so a "no MoveIt collision" check would only restate the plans themselves; knocking the target is the observable proxy.

## DC-34 (P9 fix) Clearance counts only the arm and gripper links; perception diagnostics
- **Evidence (first gate_p09 run):** the only pick that reached the server was rejected with "pre-grasp clearance 0.002 m". The geometry model gives ≥ 4.4 cm at that pose (upper arm to arm_base_link).
- **Cause:** `PlanningScene::distanceToCollision` measures *every* robot link against the world. The wheels and casters rest 2 mm above the floor box (DC-24), so every pose measured 0.002 m.
- **Fix:** both the self and world distances use a `DistanceRequest` with `active_components_only` = shoulder, upper arm, forearm, wrist, gripper base and both fingers. In MoveIt's FCL callback a pair counts if at least one of its links is active, so the arm is still checked against the chassis, camera, lidar and floor. The log names the closest pair.
- **Perception:** in 12 of 13 trials the gate got no estimate with ≥ 10 observations within 6 s, even though the target is fully in view at base-x 0.55 (checked with the ray caster for all 13 poses).
  - The gate now waits up to 15 s and counts single-frame detections and track messages. It also prints the detector throughput lines.
  - RViz no longer opens with `GATE_GUI=1`, because its MotionPlanning display is heavy; use `GATE_RVIZ=1` to get it.

## DC-35 (P9 fix 2) Unlit cylinder side: top-cap estimate. Welded object: hold the fingers, verify by attach state
- **Evidence (third gate_p09 run, 2026-11-14):**
  - **Perception:** robot north of the target looking south. The red mask covers only image rows v 311–350, at V 255 (saturated). The ray caster puts the cylinder's side down to v ≈ 434, but those pixels are not red-ish (S < 60).
    - So only the sunlit top cap is segmented, and the blob is rejected as "floating" or "no side-surface points".
    - In P8 the camera always faced the lit south side.
  - **Grasp, pick 7 trace:**
    1. The fingers stall at L 0.030 / R 0.020 with contact on both; Gazebo reports attached at 8.9 s.
    2. During the lift the target rises with the gripper to z 0.096, so the weld holds.
    3. Meanwhile both fingers close to 0 and the contacts stop.
    4. The server reported `OBJECT_LOST` and detached at 11.1 s; the target fell.
- **Cause, grasp:** the weld makes the target part of the robot's articulation. The robot model has no self-collision, so the fingers, still commanded to 0.0, pass through it. The contact sensors therefore can't confirm a held object after the attach in this simulation.
- **Fix, perception:** if fewer than 30 side points are seen, the axis is the top cap's centroid (points within 1.5 cm of z = 0.15), weighted by depth² to undo the perspective density.
  - Renders with a non-red side: error ≤ 4.9 mm at base-x 0.40–0.80. Beyond ~1 m the cap is smaller than `min_area`, so nothing is detected.
  - The "floating" rule now allows points at the object's height (z5 > height + 4 cm).
  - **Open risk for P11:** from the target's unlit (north) side, detection works only within about 1 m. The search must also see it from another side or come close. The saved frame `/tmp/p09_view_*.png` will show what that side actually looks like.
- **Fix, grasp:**
  - After the attach, each finger is commanded to hold the position where it stalled.
  - Re-verify after the lift = fingers still inside [0.015, 0.035] **and** Gazebo confirms the attach (`/gripper/attached`).
  - `reverify_require_contact: false` in simulation, true for hardware.
  - **P11 consequence:** the transport monitor's "no contact for > 1 s" rule (SPEC §12) must use the attach state in simulation.

## DC-36 (P9 fix 3) Planning-scene changes are synchronous and confirmed
- **Evidence (fourth gate_p09 run):**
  - Every pick that reached the server after a failure succeeded (5/5): lifted 35.7 cm, attach confirmed. Hold tests moved 6.0 and 9.3 mm; missed grasps 3/3.
  - Every pick right *after a successful one* failed: "grasp clearance 0.002 m (floor – target)".
- **Cause:** the previous pick's cylinder was still attached to the gripper in MoveIt's scene. At the grasp pose it sits 2 mm above the floor box. `clearTarget()` did ask for it to be removed, but `PlanningSceneInterface::removeCollisionObjects` only publishes on a topic, and our PlanningSceneMonitor hears about any change a moment after move_group applies it. So the clearance check ran on the old scene. Detaching in MoveIt also puts the object back into the world, where the asynchronous removal could miss it.
- **Fix:**
  - Every target change (add, remove, attach, detach) goes through the synchronous service (`applyCollisionObject` / `applyAttachedCollisionObject`).
  - After each change, `waitScene()` polls our own monitored scene until it shows the expected state (max 3 s; logs a warning otherwise).
  - `clearTarget()` detaches first, then removes the world object.
- **Gate:** the hold test runs after the first 3 *successful* picks, instead of fixed trial numbers that could miss. The arm settles 0.5 s before each measurement.

## DC-37 (P9 fix 4) Hold the fingers BEFORE the attach
- **Evidence (fifth gate_p09 run):**
  - Synchronous scene updates fixed the clearance failures (0 left). Hold tests 3/3 (6.0, 3.3, 3.0 mm), missed grasps 3/3.
  - But only 4/10 picks passed. The traces of the other 6 show the cause:
    1. Gazebo reports attached during `verify`.
    2. Over the next ~0.4 s the fingers, still commanded to 0.0, close from e.g. 0.020/0.030 to 0.013/0.023 through the now non-colliding object.
    3. Only then did the DC-35 hold arrive, freezing them below the 0.015 stall minimum. Re-verify then said UNCERTAIN/MISSED and the object was released.
  - The 4 passes were picks where the hold happened to arrive fast enough.
- **Fix:**
  - After a verified grasp, each finger is first commanded to hold 2 mm inside its stall position. While the object still collides this is a light squeeze and nothing moves.
  - *Then* the server attaches. After the weld, each finger moves those 2 mm and stops.
  - Re-verify in simulation = each finger within 4 mm of its hold command **and** Gazebo confirms the attach.
  - Parameters: `hold_squeeze` 0.002, `hold_tolerance` 0.004.

## DC-38 (P10) Place action; box at base-x 0.48, not 0.45
- **Box distance:** SPEC puts the box centre at base-x 0.45 for the place, so the box front wall would be at 0.30. The lidar puck reaches x 0.305, so the wall would sit 5 mm *inside* the lidar (geometry check, P10). At 0.48 the gap is 2.5 cm, and 1.8 cm with 0.05 rad of relative yaw.
  - At 0.48 the place poses solve with ≥ 4.2 cm arm clearance. The held object clears the rim by 3 cm at pre-place and stands 2 cm above the box floor at release.
  - **P11:** FINAL_ALIGN_BOX must creep to base-x **0.48**.
- **Place sequence** (SPEC §11.6 with these specifics):
  - **Box walls:** added to the scene from the place goal. Centre = goal xy; inside floor = goal z − 0.075 − 0.02; box yaw = the goal's orientation, which is the box detector's yaw. Then IK and the ≥ 1 cm clearance check, *with* the box and the held object in the scene.
  - **Order of release:** **open first, then detach.** While welded, the fingers pass through the object. Detaching while they are still 2 mm inside it (the DC-37 hold) would let Gazebo push the object out.
  - **Verify the release:** both fingers ≥ 0.035 and no finger contact for 0.5 s.
  - **Retract before the perception check.** SPEC verifies first and retracts after, but at the place pose the gripper hides the object's top from the camera. The check then needs ≥ 3 fresh `target_raw` detections (after resetting the tracker), with their median within 0.10 m of the place point. That works with the top-cap estimate (DC-35) because only the top shows above the rim.
  - Then stow. The box walls are removed from the scene at the end of every place, success or failure: the box is fixed in the robot's planning frame and would otherwise move with the robot.
- **Gate:** each trial picks a target placed at base-x 0.40 in front of the box (P9 code), estimates the box with `box_detector` (odom frame), creeps on odometry to base-x 0.48, then places.
  - Box offsets ±0.03, headings ±0.05.
  - Success = result success + ground truth: object inside the inner bounds minus 1 cm, standing (tilt < 10°), bottom below the rim.
  - The box is static in Gazebo, so PHASES' "box displacement" check cannot be measured. Arm-vs-box safety rests on MoveIt checking every pose and waypoint with the walls in the scene.

## DC-39 (P10 fix) Perception release check after the stow
- **Evidence (second gate_p10 run):**
  - **Placement worked:** 8 of 8 completed trials left the object inside the box, ≤ 6 mm from its centre, upright, bottom on the box floor (z 0.010). Box estimate error after the creep was 0.6–1.3 mm. Arm stowed, fingers open, attach off 8/8.
  - **The release check failed:** every place ended `RELEASE_NOT_CONFIRMED` with **0** target detections in 5 s. The check ran with the arm retracted 0.15 m above the box.
- **Offline:** the ray caster (robot only, no arm) gives the object in the box at image rows 355–479, detected with 0 mm error. So in Gazebo the difference is the arm, still raised over the box.
- **Change:** retract → **stow** → perception check. In stow the whole arm is outside the camera frustum (DC-25). If the check still fails, the gate now runs the detector diagnostics on that view and saves the frame.
- **Earlier gate fix (same phase):** the pick set-up first put the target at base-x 0.40, where the camera sees only its top edge (0 detections in all 10 trials). It now uses P9's sequence: estimate at 0.55, creep to 0.40, pick.

## DC-40 (P10 fix 2) The camera mast's top cap blinded the bottom third of the image
- **Evidence (third gate_p10 run, frames `/tmp/p09_view_place_{1,4}.png`):**
  - Every frame has a light-grey dome over the bottom of the image, apex at (u 320, v 328), edges at v ≈ 414 at u 32 / 608. It hides the box floor and the object in the box.
  - Projecting the mast's top rim (r 0.015, top at z 0.485, 0.015 below the camera centre at z 0.50) gives (320, 328), (32, 414), (608, 414): the same curve. Everything more than 45° below horizontal was mast.
  - Gazebo Fortress's RGB-D camera draws it although its `<clip><near>` is 0.1 m and the cap is 0.02 m away.
  - This matches every red pixel that was seen: only the 3 trials with the target 3 cm to the left showed red, and only the top ~12 rows of the cap. That corner of the cap sits right at the 45° line.
- **Fix (deviation from SPEC §3.2):**
  - Mast length 0.30 → **0.27**; joint z 0.26 → **0.245** in base_link. The mast now spans z 0.185–0.455 above ground, and its link origin is at z 0.32.
  - The top is now 0.045 below the camera centre, so its rim is 71.6° below horizontal. The image bottom is 61.5° below. The camera stays where it was (TF and P8 calibration unchanged); its mount simply no longer touches the post.
  - `check_urdf_numbers.py` now checks the real requirement, "mast top outside the camera view". It FAILs on the old URDF (rim 45.0° below).
  - The collision matrix was regenerated from the new URDF and came out identical (61 disables). `gen_acm --verify` and `check_named_states` pass. Stow clearance is still 0.047 m.
- **Corrections to earlier records:**
  - **DC-35:** the cylinder side was not "unlit". At base-x 0.55 the side lies 45–55° below the camera, which was entirely behind the mast. The top-cap fallback stays, since the box walls hide the side of a placed object.
  - **DC-39:** the arm was not blocking the view. The release check stays after the stow anyway, since that is still the right order.
  - **P10 run 1:** "at 0.40 the camera sees only its top edge" had the same cause.
- **Follow-up before P11:** rerun the P8 gate on the new view, so the perception numbers are recorded with the bottom of the image visible.

## DC-41 (P11) Mission FSM: structure and deviations from SPEC §12
- **Structure:**
  - `my_robot_mission/core.py`: the whole state machine, with no ROS in it.
  - `mission_fsm.py`: the ROS node. It owns every action, service and topic, polls them without blocking, and ticks the core at 10 Hz.
  - **Tests:**
    - `test/test_mission_fsm.py` runs the core against `fake_world.py`, a moving 2-D robot with fake servers and camera-cone detectors. It covers the happy path for T1/T2/T3, every failure branch, and the timeout of all 14 states that have one.
    - `test/test_mission_ros.py` runs the real node against fake ROS servers in their own DDS domain.
    - `test/test_params.py` checks that the YAML and code defaults match.
- **Deviations:**
  1. **BACKUP state:** a helper state of its own, logged in the CSV, with a 15 s timeout. Back-ups drive the odometry on `/cmd_vel` (0.08 m/s) instead of using Nav2's `/backup`. SPEC already lets the mission publish during back-ups.
  2. **LOCALIZE:** `/initialpose` uses RViz's default spread (0.0625 / 0.0625 / 0.0685), which is above the 0.05 limits, so convergence has to be earned. The node first asks AMCL for no-motion updates (`/request_nomotion_update`, 2 Hz, up to 10). Only then does it spin 2π, at most twice (SPEC: "1 extra spin").
  3. **Object lost** = `/gripper/attached` false, or a finger below 0.010, held for more than 1.0 s (DC-35: contacts go silent after the weld). It is monitored at 5 Hz from NAVIGATE_HOME until PLACE, not only during transport.
  4. **Box creep** ends at base-x **0.48** (DC-38).
  5. **Freezing the pose:** after the closed-loop stop (target at 0.55, box at 0.65), the tracker is reset. The odom pose is frozen from ≥ 8 observations taken at rest. AMCL only updates after motion, so all those observations share one map→odom, and corrections made while driving in cannot leak into the pick/place goal.
  6. **Frames:** detections are consumed in `map` (SPEC §9). PickObject/PlaceObject goals are in `odom` (SPEC §7) and are converted with the map→odom TF at the freeze.
  7. **Standoff test:**
     - The whole footprint (+0.03 m) is checked against Nav2's raw costs from `/global_costmap/get_costmap`: no lethal cell under it, and the centre below "inscribed".
     - If no costmap arrives, the direct standoff is used unchecked, and a warning is logged.
     - Retries clear both costmaps.
  8. **FAILED while holding:** nothing is commanded. In transport the arm is already at stow (stow = transport, DC-25). A failed pick or place stows the arm itself.
  9. **Timeouts** run from state entry.
     - For REFINE_TARGET and FINAL_ALIGN_* the timeout is the normal failure path (re-approach, or back up).
     - A PLACE timeout cancels the goal and fails without a retry, because the arm's state after a cancel is unknown.
  10. **Results:** `trial_id` and `results_dir` parameters set where the CSV goes: `results/<dir>/mission_<trial_id>.csv`.
  11. **`full_mission.launch.xml`:**
      - MoveIt starts at +10 s, the pick/place server at +15 s, the mission at +20 s; INIT waits 90 s for everything.
      - Perception starts with both detectors off, in `map`.
      - my_robot_bringup does not `<exec_depend>` on the packages it launches, because my_robot_navigation already depends on it (cycle).
- **Search waypoints:** unchanged from SPEC.
  - `tools/gates/check_waypoints.py` checks them against the world geometry: ≥ 0.6 m from T1–T3, spin-in-place clear at 16 headings, a free standoff from every waypoint that sees a target, and every target within 2.7 m of a waypoint.
  - During mission 1 the gate checks them again against the live costmap.
- **Scoring:** the evaluator is still the Phase-3 version. In P11 the gate scores each mission with `tools/gates/mission_watch.py`: placement inside the box, standing, below the rim, and zero chassis contacts. Trial scoring moves into evaluator.py in P12.
- **Known risks:**
  - The SEARCH exit needs stddev < 0.03 while spinning; if detection timing blurs that, the target is found at the next waypoint instead.
  - The AMCL no-motion service name is taken from Humble's amcl_node. If it is absent, LOCALIZE spins instead and logs a warning.
