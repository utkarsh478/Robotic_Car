#!/usr/bin/env bash
# Phase-1 gate. Usage:  cd ~/ros2_ws && bash tools/gates/gate_p01.sh
# Builds my_robot_description, validates the URDF, launches the display headless,
# checks TF. Prints PASS/FAIL per item, exits non-zero on any failure.
# no "set -u": colcon's install/setup.bash reads unset variables
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
source "$WS/tools/gates/gate_env.sh" || exit 1

FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }
LAUNCH_PID=""
cleanup() { [ -n "$LAUNCH_PID" ] && { kill -INT -- -"$LAUNCH_PID" 2>/dev/null; sleep 2; kill -9 -- -"$LAUNCH_PID" 2>/dev/null; }; }
trap cleanup EXIT

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_description > /tmp/p01_build.log 2>&1; then
  pass "colcon build my_robot_description"
else
  fail "colcon build (see /tmp/p01_build.log)"; tail -20 /tmp/p01_build.log; exit 1
fi
source install/setup.bash

echo "=== URDF ==="
URDF=/tmp/p01_robot.urdf
if xacro src/my_robot_description/urdf/my_robot.urdf.xacro > "$URDF" 2>/tmp/p01_xacro.err; then
  pass "xacro expands"
else
  fail "xacro: $(head -3 /tmp/p01_xacro.err)"; exit 1
fi
if command -v check_urdf >/dev/null; then
  if check_urdf "$URDF" > /tmp/p01_check_urdf.log 2>&1; then
    grep -q "root Link: base_footprint" /tmp/p01_check_urdf.log && pass "check_urdf OK, root base_footprint" \
      || fail "check_urdf root is not base_footprint"
  else
    fail "check_urdf: $(tail -3 /tmp/p01_check_urdf.log)"
  fi
else
  echo "WARN  check_urdf not installed (sudo apt install liburdfdom-tools); skipped"
fi
python3 tools/gates/check_urdf_numbers.py "$URDF" || FAILS=$((FAILS + 1))

echo "=== TF (display.launch.xml headless) ==="
setsid ros2 launch my_robot_description display.launch.xml rviz:=false gui:=false > /tmp/p01_launch.log 2>&1 &
LAUNCH_PID=$!
sleep 4
python3 tools/gates/tf_check_p01.py || FAILS=$((FAILS + 1))
grep -iE "error|exception" /tmp/p01_launch.log && fail "errors in launch log (/tmp/p01_launch.log)" \
  || pass "launch log clean"

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P01: ALL PASS"; else echo "GATE P01: $FAILS FAILURE(S)"; fi
exit "$FAILS"
