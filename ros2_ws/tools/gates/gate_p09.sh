#!/usr/bin/env bash
# Phase-9 gate: /pick_object (pick_place_server) in mission_world.  (~15 min)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p09.sh
#   GATE_GUI=1 ... opens Gazebo so you can watch the picks; GATE_RVIZ=1 also opens RViz (MotionPlanning,
#   heavy: it slows everything else down).
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

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_interfaces my_robot_manipulation my_robot_description \
     my_robot_bringup my_robot_moveit_config my_robot_perception > /tmp/p09_build.log 2>&1; then
  pass "colcon build (+ pick_place_server)"
else
  fail "colcon build (see /tmp/p09_build.log)"; grep -E "error" /tmp/p09_build.log | head -20; exit 1
fi
WARN=$(grep -cE "my_robot_manipulation/(src|include|test)/.*warning:" /tmp/p09_build.log)
[ "$WARN" -eq 0 ] && pass "no compiler warnings in our code" \
  || { fail "$WARN compiler warnings in my_robot_manipulation"; grep -E "warning:" /tmp/p09_build.log | head -10; }
source install/setup.bash

echo "=== unit tests (IK + grasp logic) ==="
colcon test --packages-select my_robot_manipulation --event-handlers console_direct+ > /tmp/p09_test.log 2>&1
RES=$(colcon test-result --test-result-base build/my_robot_manipulation 2>&1 | tail -1)
echo "$RES" | grep -qE " 0 errors, 0 failures" && pass "gtest: $RES" || { fail "gtest: $RES (see /tmp/p09_test.log)";
  grep -E "\[  FAILED  \]" /tmp/p09_test.log | head -10 | sed 's/^/      /'; }

echo "=== Gazebo + move_group + pick_place_server + perception (frame odom) ==="
setsid ros2 launch my_robot_bringup mission_sim.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  > /tmp/p09_sim.log 2>&1 &
PIDS+=($!); disown
for i in $(seq 1 120); do
  ACTIVE=$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|diff_drive_controller|arm_controller|gripper_controller).*active')
  [ "$ACTIVE" -ge 4 ] && break
  sleep 1
done
[ "$ACTIVE" -ge 4 ] && pass "controllers active (${i} s)" || { fail "controllers not active"; tail -20 /tmp/p09_sim.log; exit 1; }

setsid ros2 launch my_robot_moveit_config move_group.launch.py > /tmp/p09_move_group.log 2>&1 &
PIDS+=($!); disown
for i in $(seq 1 90); do grep -q "You can start planning now" /tmp/p09_move_group.log && break; sleep 1; done
grep -q "You can start planning now" /tmp/p09_move_group.log && pass "move_group up (${i} s)" \
  || { fail "move_group not ready (see /tmp/p09_move_group.log)"; exit 1; }

setsid ros2 launch my_robot_manipulation manipulation.launch.py > /tmp/p09_server.log 2>&1 &
PIDS+=($!); disown
for i in $(seq 1 60); do grep -q "pick_place_server ready" /tmp/p09_server.log && break; sleep 1; done
if grep -q "pick_place_server ready" /tmp/p09_server.log; then
  pass "pick_place_server ready (${i} s)"
else
  fail "pick_place_server not ready (see /tmp/p09_server.log)"; grep -E "ERROR|FATAL|rror" /tmp/p09_server.log | head -10; exit 1
fi

setsid ros2 launch my_robot_perception perception.launch.xml enabled:=true map_frame:=odom \
  > /tmp/p09_perception.log 2>&1 &
PIDS+=($!); disown
if [ "${GATE_RVIZ:-0}" = "1" ]; then
  setsid ros2 launch my_robot_moveit_config moveit_rviz.launch.py > /tmp/p09_rviz.log 2>&1 &
  PIDS+=($!); disown
fi
sleep 5

echo "=== pick trials (10 picks incl. 3 hold tests, 3 missed grasps) ==="
python3 tools/gates/pick_trials.py || FAILS=$((FAILS + 1))

echo "      perception throughput (camera = 150 pairs per 10 s):"
grep -E "frames processed" /tmp/p09_perception.log | tail -6 | sed 's/.*\]: /        /'
echo "      pick_place_server summary lines:"
grep -E "pick (SUCCEEDED|FAILED)|grasp (OK|MISSED|UNCERTAIN)|after lift|clearance" /tmp/p09_server.log | sed 's/.*\]: /        /' | tail -40
ERRS=$(grep -cE "\[(ERROR|FATAL)\]" /tmp/p09_server.log)
echo "      ($ERRS ERROR/FATAL lines in /tmp/p09_server.log; missed-grasp trials log some by design)"

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P09: ALL PASS"; else echo "GATE P09: $FAILS FAILURE(S)"; fi
exit "$FAILS"
