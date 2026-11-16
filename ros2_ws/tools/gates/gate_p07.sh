#!/usr/bin/env bash
# Phase-7 gate: MoveIt 2 config (SRDF, ACM, named states), move_group on the Gazebo arm,
# collision rejection, reachability sweep.  (~6 min)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p07.sh
#   GATE_GUI=1 ... also opens Gazebo and RViz (MotionPlanning) so you can watch.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
source "$WS/tools/gates/gate_env.sh" || exit 1
HR="${HEADLESS_RENDERING:-true}"
if [ "${GATE_GUI:-0}" = "1" ]; then HEADLESS=false; else HEADLESS=true; fi

FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }
PIDS=()
stop_all() {
  for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done
  sleep 4
  for p in "${PIDS[@]}"; do kill -9 -- -"$p" 2>/dev/null; done
  PIDS=()
  type kill_gate_sims >/dev/null 2>&1 && kill_gate_sims
}
trap stop_all EXIT
command -v nvidia-smi >/dev/null && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

echo "=== preflight ==="
MISSING=""
for p in moveit_ros_move_group moveit_configs_utils moveit_kinematics moveit_planners_ompl \
         moveit_simple_controller_manager moveit_ros_visualization; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || MISSING="$MISSING ros-$ROS_DISTRO-${p//_/-}"
done
if [ -n "$MISSING" ]; then
  fail "MoveIt packages missing. Install, then rerun:"
  echo "      sudo apt install$MISSING"
  exit 1
fi
pass "MoveIt packages installed"
python3 -c "import numpy" 2>/dev/null && pass "numpy" || { fail "numpy missing: sudo apt install python3-numpy"; exit 1; }

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_interfaces my_robot_manipulation \
     my_robot_description my_robot_bringup my_robot_moveit_config > /tmp/p07_build.log 2>&1; then
  pass "colcon build (+ my_robot_moveit_config)"
else
  fail "colcon build (see /tmp/p07_build.log)"; grep -E "rror" /tmp/p07_build.log | head -20; exit 1
fi
source install/setup.bash

echo "=== SRDF / ACM / named states (offline, from the installed xacro) ==="
xacro src/my_robot_description/urdf/my_robot.urdf.xacro with_arm:=true > /tmp/p07_robot.urdf \
  && pass "xacro" || { fail "xacro failed"; exit 1; }
python3 - <<'EOF' && pass "SRDF names exist in the URDF" || fail "SRDF references unknown links/joints"
import re, sys, xml.etree.ElementTree as ET
u = ET.parse("/tmp/p07_robot.urdf").getroot()
links = {l.get("name") for l in u.findall("link")}
joints = {j.get("name") for j in u.findall("joint")}
s = ET.parse("src/my_robot_moveit_config/config/my_robot.srdf").getroot()
bad = []
for c in s.iter("chain"):
    bad += [n for n in (c.get("base_link"), c.get("tip_link")) if n not in links]
for g in s.iter("group"):
    bad += [j.get("name") for j in g.findall("joint") if j.get("name") not in joints]
for gs in s.iter("group_state"):
    bad += [j.get("name") for j in gs.findall("joint") if j.get("name") not in joints]
for d in s.iter("disable_collisions"):
    bad += [n for n in (d.get("link1"), d.get("link2")) if n not in links]
for p in s.iter("passive_joint"):
    bad += [p.get("name")] if p.get("name") not in joints else []
for e in s.iter("end_effector"):
    bad += [e.get("parent_link")] if e.get("parent_link") not in links else []
print(f"      {len(list(s.iter('disable_collisions')))} disable_collisions, "
      f"{len(list(s.iter('group_state')))} named states, unknown names: {bad or 'none'}")
sys.exit(1 if bad else 0)
EOF
python3 src/my_robot_moveit_config/scripts/check_named_states.py --urdf /tmp/p07_robot.urdf > /tmp/p07_states.txt 2>&1
sed 's/^/      /' /tmp/p07_states.txt
tail -1 /tmp/p07_states.txt | grep -q "ALL PASS" && pass "stow/transport meet SPEC 3.5 (FK + geometry)" \
  || fail "named state constraints (see above)"
python3 src/my_robot_moveit_config/scripts/gen_acm.py --urdf /tmp/p07_robot.urdf --verify --trials 50000 \
  > /tmp/p07_acm.txt 2>&1
sed 's/^/      /' /tmp/p07_acm.txt
tail -1 /tmp/p07_acm.txt | grep -q "^PASS" && pass "ACM: no disabled pair collides in 50 000 fresh samples" \
  || fail "ACM verification"

echo "=== Gazebo + move_group ==="
setsid ros2 launch my_robot_bringup my_robot_gazebo.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  > /tmp/p07_gazebo.log 2>&1 &
PIDS+=($!); disown
for i in $(seq 1 120); do
  ACTIVE=$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|arm_controller|gripper_controller).*active')
  [ "$ACTIVE" -ge 3 ] && break
  sleep 1
done
[ "$ACTIVE" -ge 3 ] && pass "arm + gripper controllers active (${i} s)" \
  || { fail "controllers not active"; grep -iE "error|fail" /tmp/p07_gazebo.log | tail -10; exit 1; }

setsid ros2 launch my_robot_moveit_config move_group.launch.py > /tmp/p07_move_group.log 2>&1 &
PIDS+=($!); disown
for i in $(seq 1 90); do
  grep -q "You can start planning now" /tmp/p07_move_group.log && break
  sleep 1
done
if grep -q "You can start planning now" /tmp/p07_move_group.log; then
  pass "move_group up (${i} s)"
else
  fail "move_group did not report ready in 90 s (see /tmp/p07_move_group.log)"
  grep -E "ERROR|FATAL|rror" /tmp/p07_move_group.log | head -15; exit 1
fi
# errors during START-UP only (later planning errors, e.g. for the deliberate chassis goal, are expected)
# Known benign: Humble logs "No 3D sensor plugin(s) defined for octomap updates" at ERROR level
# whenever no depth sensor feeds the octomap. We use none by design (scene objects come from
# perception as primitives, SPEC 11.2), so that one line is listed but not counted.
BENIGN="No 3D sensor plugin\(s\) defined for octomap updates"
STARTUP_ERR=$(sed -n '1,/You can start planning now/p' /tmp/p07_move_group.log | grep -E "\[(ERROR|FATAL)\]" | grep -vE "$BENIGN")
sed -n '1,/You can start planning now/p' /tmp/p07_move_group.log | grep -E "$BENIGN" >/dev/null \
  && echo "      (ignored, benign: 'No 3D sensor plugin(s) defined for octomap updates' - no octomap sensor by design)"
[ -z "$STARTUP_ERR" ] && pass "move_group start-up: no ERROR/FATAL lines" \
  || { fail "move_group start-up errors:"; echo "$STARTUP_ERR" | head -10 | sed 's/^/      /'; }
WARNS=$(sed -n '1,/You can start planning now/p' /tmp/p07_move_group.log | grep -c "\[WARN\]")
echo "      ($WARNS start-up warnings; listed at the end)"
UST=$(ros2 param get /move_group use_sim_time 2>/dev/null)
echo "$UST" | grep -q "True" && pass "move_group use_sim_time = True" || fail "move_group use_sim_time: $UST"
NRSP=$(ros2 node list 2>/dev/null | grep -c "robot_state_publisher")
[ "$NRSP" -eq 1 ] && pass "exactly one robot_state_publisher" || fail "$NRSP robot_state_publisher nodes"
if [ "${GATE_GUI:-0}" = "1" ]; then
  setsid ros2 launch my_robot_moveit_config moveit_rviz.launch.py > /tmp/p07_rviz.log 2>&1 &
  PIDS+=($!); disown
  echo "      RViz (MotionPlanning) started"
fi

echo "=== plan + execute (named states, gripper, collision rejection, grasp poses) ==="
python3 tools/gates/p07_checks.py || FAILS=$((FAILS + 1))

echo "=== reachability sweep (plan only) ==="
python3 tools/gates/reach_sweep.py > /tmp/p07_sweep.txt 2>&1
RC=$?
grep -vE "^  x " /tmp/p07_sweep.txt
echo "      (per-point lines: /tmp/p07_sweep.txt, table: results/p07_reach_sweep.csv)"
[ $RC -eq 0 ] || FAILS=$((FAILS + 1))

if [ "$WARNS" -gt 0 ]; then
  echo
  echo "move_group start-up warnings (information):"
  sed -n '1,/You can start planning now/p' /tmp/p07_move_group.log | grep "\[WARN\]" | sed 's/^/      /' | head -15
fi
echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P07: ALL PASS"; else echo "GATE P07: $FAILS FAILURE(S)"; fi
exit "$FAILS"
