#!/usr/bin/env bash
# Phase-4 gate: Nav2 + AMCL on the saved map.   (~10 min)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p04.sh
#   GATE_GUI=1 bash tools/gates/gate_p04.sh   -> Gazebo + RViz (costmaps, plans, particles) open
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
source "$WS/tools/gates/gate_env.sh" || exit 1
HR="${HEADLESS_RENDERING:-true}"
if [ "${GATE_GUI:-0}" = "1" ]; then HEADLESS=false; RVIZ=true; else HEADLESS=true; RVIZ=false; fi

FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }
LAUNCH_PID=""
stop_launch() {
  if [ -n "$LAUNCH_PID" ]; then
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null; sleep 5; kill -9 -- -"$LAUNCH_PID" 2>/dev/null
    LAUNCH_PID=""
  fi
  pkill -f "ign gazebo -s -r.*mission_world" 2>/dev/null; sleep 1
}
trap stop_launch EXIT
command -v nvidia-smi >/dev/null && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

echo "=== preflight ==="
MAP=src/my_robot_navigation/maps/mission_world.yaml
[ -f "$MAP" ] && pass "saved map present ($MAP)" || { fail "no saved map: run gate_p03.sh first"; exit 1; }
python3 tools/make_nav2_params.py || { fail "nav2 params generation"; exit 1; }

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_description my_robot_bringup my_robot_navigation \
     > /tmp/p04_build.log 2>&1; then
  pass "colcon build"
else
  fail "colcon build (see /tmp/p04_build.log)"; tail -20 /tmp/p04_build.log; exit 1
fi
source install/setup.bash

echo "=== launch nav_sim (headless:=$HEADLESS) ==="
setsid ros2 launch my_robot_navigation nav_sim.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  rviz:="$RVIZ" spawn_objects:=false > /tmp/p04_nav.log 2>&1 &
LAUNCH_PID=$!

for i in $(seq 1 120); do
  [ "$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|diff_drive_controller).*active')" -ge 2 ] && break
  sleep 1
done
[ "$i" -lt 120 ] && pass "controllers active (${i} s)" || { fail "controllers not active"; exit 1; }

NODES="map_server amcl controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother"
for i in $(seq 1 90); do
  INACTIVE=""
  for n in $NODES; do ros2 lifecycle get /$n 2>/dev/null | grep -q "^active" || INACTIVE="$INACTIVE $n"; done
  [ -z "$INACTIVE" ] && break
  sleep 1
done
if [ -z "$INACTIVE" ]; then
  pass "all 9 Nav2 lifecycle nodes active (${i} s)"
else
  fail "lifecycle nodes not active after 90 s:$INACTIVE"
  grep -E "\[ERROR\]|\[FATAL\]" /tmp/p04_nav.log | tail -15
  exit 1
fi
for n in amcl controller_server planner_server bt_navigator; do
  [ "$(ros2 param get /$n use_sim_time 2>/dev/null)" = "Boolean value is: True" ] || fail "$n use_sim_time is not true"
done
pass "use_sim_time true on amcl/controller/planner/bt_navigator"

echo "=== 10 navigation goals ==="
python3 tools/gates/nav_runner.py || FAILS=$((FAILS + 1))

echo "=== blocked path ==="
python3 tools/gates/p04_blocked.py || FAILS=$((FAILS + 1))

ERR=$(grep -c "\[ERROR\]" /tmp/p04_nav.log)
echo "      [ERROR] lines in launch log: $ERR (see /tmp/p04_nav.log; some are expected during the blocked-path test)"

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P04: ALL PASS"; else echo "GATE P04: $FAILS FAILURE(S)"; fi
exit "$FAILS"
