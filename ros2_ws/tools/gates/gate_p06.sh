#!/usr/bin/env bash
# Phase-6 gate: interfaces package + analytic IK library (unit tests + Gazebo FK-vs-TF).  (~4 min)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p06.sh
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
source "$WS/tools/gates/gate_env.sh" || exit 1
HR="${HEADLESS_RENDERING:-true}"
if [ "${GATE_GUI:-0}" = "1" ]; then HEADLESS=false; else HEADLESS=true; fi

FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }
LAUNCH_PID=""
stop_launch() {
  if [ -n "$LAUNCH_PID" ]; then
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null; sleep 4; kill -9 -- -"$LAUNCH_PID" 2>/dev/null
    LAUNCH_PID=""
  fi
  kill_gate_sims
}
trap stop_launch EXIT
command -v nvidia-smi >/dev/null && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_interfaces my_robot_manipulation \
     my_robot_description my_robot_bringup > /tmp/p06_build.log 2>&1; then
  pass "colcon build (interfaces, manipulation, description, bringup)"
else
  fail "colcon build (see /tmp/p06_build.log)"; grep -E "error|Error" /tmp/p06_build.log | head -20; exit 1
fi
WARN=$(grep -cE "my_robot_manipulation/(src|include|test)/.*warning:" /tmp/p06_build.log)
[ "$WARN" -eq 0 ] && pass "no compiler warnings in our code (-Wall -Wextra -Wpedantic)" \
  || { fail "$WARN compiler warnings in my_robot_manipulation"; grep -E "warning:" /tmp/p06_build.log | head -10; }
source install/setup.bash

echo "=== interfaces ==="
for i in msg/Detection action/PickObject action/PlaceObject; do
  ros2 interface show my_robot_interfaces/$i > /tmp/p06_iface.txt 2>&1 && [ -s /tmp/p06_iface.txt ] \
    && pass "my_robot_interfaces/$i ($(grep -cvE '^\s*(#|$)' /tmp/p06_iface.txt) fields/constants)" \
    || fail "my_robot_interfaces/$i not available"
done

echo "=== unit tests (gtest) ==="
colcon test --packages-select my_robot_manipulation --event-handlers console_direct+ > /tmp/p06_test.log 2>&1
grep -E "^\[ +(OK|FAILED|PASSED|info) " /tmp/p06_test.log | sed 's/^/      /'
RES=$(colcon test-result --test-result-base build/my_robot_manipulation 2>&1 | tail -1)
echo "      $RES"
echo "$RES" | grep -qE "0 errors, 0 failures" && pass "gtest: $RES" || fail "gtest: $RES (see /tmp/p06_test.log)"

echo "=== FK vs TF in Gazebo ==="
setsid ros2 launch my_robot_bringup my_robot_gazebo.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  > /tmp/p06_launch.log 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 120); do
  ACTIVE=$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|arm_controller).*active')
  [ "$ACTIVE" -ge 2 ] && break
  sleep 1
done
if [ "$ACTIVE" -ge 2 ]; then
  pass "arm controller active (${i} s)"
  python3 tools/gates/fk_vs_tf.py || FAILS=$((FAILS + 1))
else
  fail "arm controller not active"; grep -iE "error|fail" /tmp/p06_launch.log | tail -10
fi

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P06: ALL PASS"; else echo "GATE P06: $FAILS FAILURE(S)"; fi
exit "$FAILS"
