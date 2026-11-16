#!/usr/bin/env bash
# Phase-2 gate (Humble + Gazebo Fortress). Usage:
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p02.sh
# Optional: HEADLESS_RENDERING=false bash tools/gates/gate_p02.sh   (if sensors stay silent headless)
# no "set -u": colcon's install/setup.bash reads unset variables
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
source "$WS/tools/gates/gate_env.sh" || exit 1
HR="${HEADLESS_RENDERING:-true}"
# GATE_GUI=1 -> open the Gazebo window (and RViz in the mapping run) for this gate's own sim.
if [ "${GATE_GUI:-0}" = "1" ]; then HEADLESS=false; RVIZ=true; else HEADLESS=true; RVIZ=false; fi

FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }
LAUNCH_PID=""
cleanup() {
  if [ -n "$LAUNCH_PID" ]; then
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null; sleep 3; kill -9 -- -"$LAUNCH_PID" 2>/dev/null
  fi
  pkill -f "ign gazebo -s -r.*test_world" 2>/dev/null
}
trap cleanup EXIT

# Use the RTX GPU for sensor rendering on hybrid-graphics laptops (harmless elsewhere).
command -v nvidia-smi >/dev/null && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

echo "=== preflight: installed plugin names (CLAUDE.md rule 3) ==="
command -v ign >/dev/null && pass "ign CLI: $(ign gazebo --versions 2>/dev/null | head -1)" || fail "ign CLI not found (ros-humble-ros-gz)"
LIB=/opt/ros/$ROS_DISTRO/lib/libgz_ros2_control-system.so
[ -f "$LIB" ] && pass "gz_ros2_control system plugin: $LIB" \
  || fail "missing $LIB; found: $(ls /opt/ros/$ROS_DISTRO/lib | grep -E 'ros2_control-system' | tr '\n' ' ')"
HW=$(grep -rhoE '(name|type)="[a-z0-9_]+/[A-Za-z]+System"' /opt/ros/$ROS_DISTRO/share/gz_ros2_control/ 2>/dev/null | sort -u | tr '\n' ' ')
echo "$HW" | grep -q 'gz_ros2_control/GazeboSimSystem' && pass "hardware plugin gz_ros2_control/GazeboSimSystem registered" \
  || fail "hardware plugin name differs; installed: $HW (override xacro arg gz_hw_plugin)"
DEMO=$(ros2 pkg prefix gz_ros2_control_demos 2>/dev/null)/share/gz_ros2_control_demos
if [ -d "$DEMO" ]; then
  echo "  demo plugin tag: $(grep -rhoE '<plugin filename="[^"]+" name="[^"]+"' "$DEMO" | sort -u | head -2 | tr '\n' ' ')"
fi

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_description my_robot_bringup > /tmp/p02_build.log 2>&1; then
  pass "colcon build description + bringup"
else
  fail "colcon build (see /tmp/p02_build.log)"; tail -20 /tmp/p02_build.log; exit 1
fi
source install/setup.bash

echo "=== P1 regression ==="
xacro src/my_robot_description/urdf/my_robot.urdf.xacro > /tmp/p02_robot.urdf 2>/tmp/p02_xacro.err \
  && python3 tools/gates/check_urdf_numbers.py /tmp/p02_robot.urdf > /tmp/p02_urdf_check.log 2>&1 \
  && pass "check_urdf_numbers ($(tail -1 /tmp/p02_urdf_check.log))" \
  || { fail "URDF checks (see /tmp/p02_urdf_check.log /tmp/p02_xacro.err)"; tail -5 /tmp/p02_urdf_check.log; }

echo "=== launch (headless, headless_rendering:=$HR) ==="
setsid ros2 launch my_robot_bringup my_robot_gazebo.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  > /tmp/p02_launch.log 2>&1 &
LAUNCH_PID=$!

ACTIVE=0
for i in $(seq 1 120); do
  ACTIVE=$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|diff_drive_controller).*active')
  [ "$ACTIVE" -ge 2 ] && break
  sleep 1
done
if [ "$ACTIVE" -ge 2 ]; then
  pass "joint_state_broadcaster + diff_drive_controller active (${i} s)"
else
  fail "controllers not active after 120 s"
  ros2 control list_controllers 2>&1 | head
  echo "--- last launch log lines ---"; grep -iE "error|fail|could not|unable" /tmp/p02_launch.log | tail -15
  exit 1
fi

echo "=== runtime checks ==="
python3 tools/gates/p02_checks.py || FAILS=$((FAILS + 1))

ERR=$(grep -c "\[ERROR\]" /tmp/p02_launch.log)
[ "$ERR" -eq 0 ] && pass "no [ERROR] lines in launch log" \
  || echo "WARN  $ERR [ERROR] line(s) in /tmp/p02_launch.log: $(grep "\[ERROR\]" /tmp/p02_launch.log | head -3)"

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P02: ALL PASS"; else echo "GATE P02: $FAILS FAILURE(S)"; fi
exit "$FAILS"
