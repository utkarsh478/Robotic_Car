#!/usr/bin/env bash
# Phase-8 gate: target + box perception vs ground truth in mission_world.  (~5 min)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p08.sh
#   GATE_GUI=1 ... opens Gazebo (watch the robot being teleported to each test pose)
#   CAPTURE_SAMPLES=1 ... replace the saved sample frames in tools/samples/
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
python3 -c "import cv2" 2>/dev/null || MISSING="$MISSING python3-opencv"
python3 -c "import pytest" 2>/dev/null || MISSING="$MISSING python3-pytest"
for p in cv_bridge message_filters; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 || MISSING="$MISSING ros-$ROS_DISTRO-${p//_/-}"
done
if [ -n "$MISSING" ]; then fail "missing packages. Install, then rerun:"; echo "      sudo apt install$MISSING"; exit 1; fi
pass "opencv $(python3 -c 'import cv2; print(cv2.__version__)'), cv_bridge, message_filters, pytest"

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_interfaces my_robot_description my_robot_bringup \
     my_robot_perception > /tmp/p08_build.log 2>&1; then
  pass "colcon build (+ my_robot_perception)"
else
  fail "colcon build (see /tmp/p08_build.log)"; grep -E "rror" /tmp/p08_build.log | head -20; exit 1
fi
source install/setup.bash
for e in perception_node target_detector box_detector; do
  [ -x "install/my_robot_perception/lib/my_robot_perception/$e" ] && pass "executable $e installed" \
    || fail "executable $e missing"
done

echo "=== Gazebo (mission_world, objects) + ground-truth map frame + detectors ==="
setsid ros2 launch my_robot_bringup mission_sim.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  > /tmp/p08_sim.log 2>&1 &
PIDS+=($!); disown
for i in $(seq 1 120); do
  ACTIVE=$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|diff_drive_controller|arm_controller).*active')
  [ "$ACTIVE" -ge 3 ] && break
  sleep 1
done
[ "$ACTIVE" -ge 3 ] && pass "controllers active (${i} s)" || { fail "controllers not active"; tail -20 /tmp/p08_sim.log; exit 1; }
setsid python3 tools/gates/gt_localizer.py > /tmp/p08_gtloc.log 2>&1 &
PIDS+=($!); disown
setsid ros2 launch my_robot_perception perception.launch.xml enabled:=true > /tmp/p08_perception.log 2>&1 &
PIDS+=($!); disown
for i in $(seq 1 30); do
  ros2 node list 2>/dev/null | grep -q "/box_detector" && ros2 node list 2>/dev/null | grep -q "/target_detector" && break
  sleep 1
done
ros2 node list 2>/dev/null | grep -q "/target_detector" && pass "detectors running" \
  || { fail "detectors not running"; tail -20 /tmp/p08_perception.log; exit 1; }
NMAP=$(ros2 topic info -v /tf 2>/dev/null | grep -c "Node name: gt_localizer")
[ "$NMAP" -ge 1 ] && pass "map->odom from gt_localizer (gate only; no AMCL running)" || fail "gt_localizer not publishing /tf"
if [ "${GATE_GUI:-0}" = "1" ]; then
  echo "      watch the masks:  ROS_DOMAIN_ID=$ROS_DOMAIN_ID ros2 run rqt_image_view rqt_image_view /perception/target/debug_image"
fi

echo "=== accuracy vs ground truth ==="
python3 tools/gates/perception_eval.py || FAILS=$((FAILS + 1))
echo "      detector throughput (logged every 10 s; camera = 150 pairs per 10 s):"
grep -E "frames processed" /tmp/p08_perception.log | tail -6 | sed 's/.*\]: /        /'
grep -E "Traceback|Error" /tmp/p08_perception.log | head -5 | sed 's/^/      detector log: /'

echo "=== pytest (geometry, synthetic renders, saved Gazebo samples) ==="
colcon test --packages-select my_robot_perception --event-handlers console_direct+ > /tmp/p08_test.log 2>&1
grep -E "passed|failed|error" /tmp/p08_test.log | tail -3 | sed 's/^/      /'
RES=$(colcon test-result --test-result-base build/my_robot_perception 2>&1 | tail -1)
echo "      $RES"
echo "$RES" | grep -qE " 0 errors, 0 failures" && pass "pytest: $RES" || { fail "pytest (see /tmp/p08_test.log)";
  grep -E "^(FAILED|ERROR)" /tmp/p08_test.log | head -10 | sed 's/^/      /'; }
NS=$(ls tools/samples 2>/dev/null | wc -l)
[ "$NS" -ge 20 ] && pass "$NS sample frames in tools/samples (replayed by test_samples.py)" \
  || fail "only $NS sample frames in tools/samples"

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P08: ALL PASS"; else echo "GATE P08: $FAILS FAILURE(S)"; fi
exit "$FAILS"
