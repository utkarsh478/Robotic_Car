#!/usr/bin/env bash
# Phase-5 gate: 4-DOF arm + gripper, controllers, startup detach, stall grasp.   (~4 min)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p05.sh
#   GATE_GUI=1 ...  to watch;  REGRESSION=1 ... to also rerun gate_p02 with the arm on.
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
  pkill -f "ign gazebo -s -r.*test_world" 2>/dev/null; sleep 1
}
trap stop_launch EXIT
command -v nvidia-smi >/dev/null && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_description my_robot_bringup my_robot_navigation \
     > /tmp/p05_build.log 2>&1; then
  pass "colcon build"
else
  fail "colcon build (see /tmp/p05_build.log)"; tail -20 /tmp/p05_build.log; exit 1
fi
source install/setup.bash

echo "=== model ==="
for arm in true false; do
  xacro src/my_robot_description/urdf/my_robot.urdf.xacro with_arm:=$arm > /tmp/p05_robot_$arm.urdf 2>/tmp/p05_xacro.err \
    && python3 tools/gates/check_urdf_numbers.py /tmp/p05_robot_$arm.urdf > /tmp/p05_urdf_$arm.log 2>&1 \
    && pass "check_urdf_numbers with_arm:=$arm ($(tail -1 /tmp/p05_urdf_$arm.log))" \
    || { fail "check_urdf_numbers with_arm:=$arm"; grep FAIL /tmp/p05_urdf_$arm.log; cat /tmp/p05_xacro.err; }
done
# What Gazebo actually gets after URDF->SDF (fixed-joint lumping renames/merges links)
if ign sdf -p /tmp/p05_robot_true.urdf > /tmp/p05_robot.sdf 2>/tmp/p05_sdf.err; then
  for name in left_finger_link_collision right_finger_link_collision; do
    grep -q "<collision name='$name'>\|<collision name=\"$name\">" /tmp/p05_robot.sdf \
      && pass "SDF has collision $name (contact sensor target)" \
      || fail "SDF has no collision '$name'; contact sensors will be silent. Names found: $(grep -o "collision name=.[a-z_]*finger[a-z_]*" /tmp/p05_robot.sdf | sort -u | tr '\n' ' ')"
  done
  grep -q "<link name='wrist_link'>\|<link name=\"wrist_link\">" /tmp/p05_robot.sdf \
    && pass "SDF keeps link wrist_link (DetachableJoint parent)" || fail "SDF has no link wrist_link"
else
  fail "ign sdf -p failed: $(head -3 /tmp/p05_sdf.err)"
fi

echo "=== launch (test_world, arm on) ==="
setsid ros2 launch my_robot_bringup my_robot_gazebo.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  > /tmp/p05_launch.log 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 120); do
  ACTIVE=$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|diff_drive_controller|arm_controller|gripper_controller).*active')
  [ "$ACTIVE" -ge 4 ] && break
  sleep 1
done
if [ "$ACTIVE" -ge 4 ]; then pass "4 controllers active (${i} s)"; else
  fail "only $ACTIVE/4 controllers active"; ros2 control list_controllers 2>&1 | head
  grep -iE "error|fail" /tmp/p05_launch.log | tail -15; exit 1
fi

echo "=== arm / gripper checks ==="
python3 tools/gates/p05_checks.py || FAILS=$((FAILS + 1))
grep -E "grasp_attach_manager.*(Gazebo reports|up:)" /tmp/p05_launch.log | sed 's/^/      log: /' | tail -4
stop_launch

if [ "${REGRESSION:-0}" = "1" ]; then
  echo "=== regression: gate_p02 with the arm on ==="
  bash tools/gates/gate_p02.sh > /tmp/p05_regress_p02.log 2>&1 \
    && pass "gate_p02 still passes with the arm" \
    || { fail "gate_p02 regression (see /tmp/p05_regress_p02.log)"; grep FAIL /tmp/p05_regress_p02.log; }
fi

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P05: ALL PASS"; else echo "GATE P05: $FAILS FAILURE(S)"; fi
exit "$FAILS"
