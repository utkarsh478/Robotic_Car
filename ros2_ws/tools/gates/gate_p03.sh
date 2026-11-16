#!/usr/bin/env bash
# Phase-3 gate: mission world, objects, evaluator, SLAM map.   (~6-8 min, headless)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p03.sh
# Part A: mission_sim with objects -> evaluator / ground-truth / isolation checks.
# Part B: mapping run (no loose objects) -> scripted drive -> save map -> compare with world.
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
stop_launch() {
  if [ -n "$LAUNCH_PID" ]; then
    kill -INT -- -"$LAUNCH_PID" 2>/dev/null; sleep 4; kill -9 -- -"$LAUNCH_PID" 2>/dev/null
    LAUNCH_PID=""
  fi
  pkill -f "ign gazebo -s -r.*mission_world" 2>/dev/null; sleep 1
}
trap stop_launch EXIT
command -v nvidia-smi >/dev/null && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

wait_controllers() {  # $1 = log file
  for i in $(seq 1 120); do
    [ "$(ros2 control list_controllers 2>/dev/null | grep -cE '(joint_state_broadcaster|diff_drive_controller).*active')" -ge 2 ] \
      && { pass "controllers active (${i} s)"; return 0; }
    sleep 1
  done
  fail "controllers not active after 120 s"; grep -iE "error|fail|could not|unable" "$1" | tail -15; return 1
}

echo "=== build ==="
if colcon build --symlink-install --packages-select my_robot_description my_robot_bringup my_robot_navigation \
     > /tmp/p03_build.log 2>&1; then
  pass "colcon build description + bringup + navigation"
else
  fail "colcon build (see /tmp/p03_build.log)"; tail -20 /tmp/p03_build.log; exit 1
fi
source install/setup.bash
python3 tools/gates/check_slam_params.py || FAILS=$((FAILS + 1))

echo "=== Part A: mission world + objects + evaluator ==="
setsid ros2 launch my_robot_bringup mission_sim.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
  > /tmp/p03_sim.log 2>&1 &
LAUNCH_PID=$!
if wait_controllers /tmp/p03_sim.log; then
  python3 tools/gates/p03_eval_check.py || FAILS=$((FAILS + 1))
  grep -q "Unable to find uri\[model://home_box\]\|Unable to find file" /tmp/p03_sim.log \
    && fail "home_box model not found (IGN_GAZEBO_RESOURCE_PATH)" || pass "home_box model resolved"
fi
stop_launch

echo "=== Part B: SLAM mapping run ==="
# never score a stale map from an earlier run
rm -rf src/my_robot_navigation/maps/mission_world.{yaml,pgm} src/my_robot_navigation/maps/mission_world_raw.{yaml,pgm} \
      results/p03_map.png results/p03_map_check.json results/p03_snapshots
setsid ros2 launch my_robot_navigation mapping.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" rviz:="$RVIZ" \
  > /tmp/p03_map.log 2>&1 &
LAUNCH_PID=$!
if wait_controllers /tmp/p03_map.log; then
  for i in $(seq 1 60); do ros2 topic list 2>/dev/null | grep -qx /map && break; sleep 1; done
  ros2 topic list | grep -qx /map && pass "slam_toolbox publishing /map" || fail "no /map from slam_toolbox"
  echo "--- driving the mapping route (~6 min) ---"
  python3 tools/gates/map_drive.py 2>&1 | tee /tmp/p03_drive.log
  grep -q "^FAIL  SLAM pose vs truth\|^FAIL  no SLAM pose" /tmp/p03_drive.log && FAILS=$((FAILS + 1))
  if grep -q "^PASS  mapping drive complete" /tmp/p03_drive.log; then
    if bash tools/save_map.sh > /tmp/p03_save.log 2>&1; then
      pass "map saved: $(tail -1 /tmp/p03_save.log)"
    else
      fail "save_map.sh"; tail -10 /tmp/p03_save.log
    fi
  else
    fail "mapping drive did not complete"
  fi
fi
DROPS=$(grep -c "Message Filter dropping" /tmp/p03_map.log)
echo "      slam_toolbox dropped scans (TF not ready): $DROPS  (from /tmp/p03_map.log)"
stop_launch

echo "=== map vs world ==="
MAP=src/my_robot_navigation/maps/mission_world.yaml
if [ -f "$MAP" ]; then
  mkdir -p results
  python3 tools/gates/check_map.py "$MAP" src/my_robot_bringup/worlds/mission_world.sdf \
    --json results/p03_map_check.json || FAILS=$((FAILS + 1))
  python3 tools/render_map.py "$MAP" src/my_robot_bringup/worlds/mission_world.sdf results/p03_map.png --scale 4 \
    && echo "      map image (black = mapped, red = true walls): results/p03_map.png"
  for y in results/p03_snapshots/*.yaml; do
    [ -f "$y" ] && python3 tools/render_map.py "$y" src/my_robot_bringup/worlds/mission_world.sdf "${y%.yaml}.png" --scale 3 >/dev/null
  done
  ls results/p03_snapshots/*.png >/dev/null 2>&1 && echo "      live-map snapshots: results/p03_snapshots/*.png"
else
  fail "no map at $MAP"
fi

echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P03: ALL PASS"; else echo "GATE P03: $FAILS FAILURE(S)"; fi
exit "$FAILS"
