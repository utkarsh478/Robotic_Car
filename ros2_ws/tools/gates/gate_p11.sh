#!/usr/bin/env bash
# Phase-11 gate: autonomous mission FSM, end to end.  (~10 min of builds/tests + ~10-15 min per mission)
#   cd ~/Robotic_car/ros2_ws && source /opt/ros/humble/setup.bash && bash tools/gates/gate_p11.sh
#   GATE_GUI=1        open Gazebo for the missions (PHASES asks for headless; use it to watch)
#   P11_TRIALS=N      missions to run (default 3; the gate needs 3 consecutive successes)
#   P11_TARGET="x y"  target position (default T1 3.3 2.4)
# Each mission is a fresh launch of full_mission.launch.xml with auto_start:=true and no input.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
source "$WS/tools/gates/gate_env.sh" || exit 1
HR="${HEADLESS_RENDERING:-true}"
if [ "${GATE_GUI:-0}" = "1" ]; then HEADLESS=false; else HEADLESS=true; fi
TRIALS="${P11_TRIALS:-3}"
read -r TX TY <<< "${P11_TARGET:-3.3 2.4}"
RESULTS="$WS/results/p11"
mkdir -p "$RESULTS"
rm -f /tmp/p11_watch_*.log /tmp/p11_*_live.log /tmp/p11_*_live.rc

FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }
PIDS=()
stop_all() {
  for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done
  sleep 5
  for p in "${PIDS[@]}"; do kill -9 -- -"$p" 2>/dev/null; done
  PIDS=()
  type kill_gate_sims >/dev/null 2>&1 && kill_gate_sims
}
trap stop_all EXIT
command -v nvidia-smi >/dev/null && export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia

echo "=== build (whole workspace) ==="
if colcon build --symlink-install > /tmp/p11_build.log 2>&1; then
  pass "colcon build"
else
  fail "colcon build (see /tmp/p11_build.log)"; grep -E "error|Failed" /tmp/p11_build.log | head -20; exit 1
fi
source install/setup.bash

echo "=== FSM tests (pytest: fake world + real node against fake servers) ==="
colcon test --packages-select my_robot_mission --pytest-args -v --event-handlers console_direct+ > /tmp/p11_test.log 2>&1
RES=$(colcon test-result --test-result-base build/my_robot_mission 2>&1 | tail -1)
echo "$RES" | grep -qE " 0 errors, 0 failures" && pass "pytest: $RES" \
  || { fail "pytest: $RES (see /tmp/p11_test.log)"; grep -E "^(FAILED|ERROR)" /tmp/p11_test.log | head -15 | sed 's/^/      /'; }
TO=$(grep -cE "test_timeout\[[A-Z_]+\] PASSED" /tmp/p11_test.log)
grep -qE "passed" /tmp/p11_test.log && echo "      $(grep -E '[0-9]+ passed' /tmp/p11_test.log | tail -1)"
[ "$TO" -ge 14 ] && pass "timeout path exercised for all 14 states with a timeout" \
  || echo "      (timeout tests: see /tmp/p11_test.log; pytest -v lists test_timeout[STATE] for every state)"

echo "=== search waypoints vs world geometry (SPEC §12 task 4) ==="
python3 tools/gates/check_waypoints.py > /tmp/p11_waypoints.log 2>&1
WF=$?
grep -E "^FAIL" /tmp/p11_waypoints.log | sed 's/^/      /'
[ "$WF" -eq 0 ] && pass "waypoints and standoffs clear ($(grep -c ^PASS /tmp/p11_waypoints.log) checks, /tmp/p11_waypoints.log)" \
  || fail "$WF waypoint checks failed (see /tmp/p11_waypoints.log)"

echo "=== isolation (static) ==="
bash tools/gates/check_isolation.sh || FAILS=$((FAILS + 1))

echo "=== full missions: $TRIALS x full_mission.launch.xml, target ($TX, $TY), auto_start, headless=$HEADLESS ==="
STREAK=0; BEST=0; OKS=0
for i in $(seq 1 "$TRIALS"); do
  ID="p11_t${i}_$(date +%H%M%S)"
  setsid ros2 launch my_robot_bringup full_mission.launch.xml headless:="$HEADLESS" headless_rendering:="$HR" \
    auto_start:=true target_x:="$TX" target_y:="$TY" trial_id:="$ID" results_dir:="$RESULTS" \
    > "/tmp/p11_launch_$i.log" 2>&1 &
  PIDS+=($!); disown
  echo "   mission $i ($ID): log /tmp/p11_launch_$i.log"
  if [ "$i" -eq 1 ]; then
    # live checks while mission 1 runs: costmap vs waypoints, and who reads ground truth
    ( for k in $(seq 1 120); do ros2 node list 2>/dev/null | grep -qx /mission_fsm && break; sleep 1; done
      sleep 5
      python3 tools/gates/check_waypoints.py --live > /tmp/p11_waypoints_live.log 2>&1; echo $? > /tmp/p11_waypoints_live.rc
      bash tools/gates/check_isolation.sh --live > /tmp/p11_isolation_live.log 2>&1; echo $? > /tmp/p11_isolation_live.rc
    ) &
    LIVE_PID=$!
  fi
  python3 tools/gates/mission_watch.py --trial "$ID" --timeout "${P11_WALL_TIMEOUT:-1800}" | tee "/tmp/p11_watch_$i.log"
  if grep -q '^RESULT .*"success": true' "/tmp/p11_watch_$i.log"; then
    OKS=$((OKS + 1)); STREAK=$((STREAK + 1)); [ "$STREAK" -gt "$BEST" ] && BEST=$STREAK
    echo "   mission $i: SUCCESS"
  else
    STREAK=0
    echo "   mission $i: FAILURE — last mission_fsm lines:"
    grep -E "mission_fsm\]" "/tmp/p11_launch_$i.log" | tail -12 | sed 's/.*\]: /        /'
  fi
  CSV="$RESULTS/mission_$ID.csv"
  [ -f "$CSV" ] && echo "   CSV $CSV ($(($(wc -l < "$CSV") - 1)) state exits)" || echo "   (no CSV at $CSV)"
  if [ "$i" -eq 1 ]; then
    wait "$LIVE_PID" 2>/dev/null
    echo "   live checks during mission 1:"
    sed 's/^/      /' /tmp/p11_waypoints_live.log /tmp/p11_isolation_live.log 2>/dev/null
    [ "$(cat /tmp/p11_waypoints_live.rc 2>/dev/null)" = "0" ] && pass "live costmap: waypoints and standoffs free" \
      || fail "live costmap check (see /tmp/p11_waypoints_live.log)"
    [ "$(cat /tmp/p11_isolation_live.rc 2>/dev/null)" = "0" ] && pass "live isolation: /evaluator/* read only by the evaluator/gate" \
      || fail "live isolation check (see /tmp/p11_isolation_live.log)"
  fi
  stop_all
  sleep 3
done

echo "=== summary ==="
[ "$BEST" -ge 3 ] && pass "3 consecutive full-mission successes at ($TX, $TY), no input after launch  -> $OKS/$TRIALS, best streak $BEST" \
  || fail "3 consecutive full-mission successes  -> $OKS/$TRIALS succeeded, best streak $BEST"
python3 - "$TRIALS" <<'PYEOF'
import json, sys
for i in range(1, int(sys.argv[1]) + 1):
    try:
        line = [l for l in open(f"/tmp/p11_watch_{i}.log") if l.startswith("RESULT ")][-1]
    except (OSError, IndexError):
        print(f"      mission {i}: no RESULT line")
        continue
    r = json.loads(line[7:])
    print(f"      {r['trial']}: {'OK  ' if r['success'] else 'FAIL'} mission {r['mission_time_sim']} s sim / "
          f"{r['wall_time']} s wall, target {r['target_offset_mm']} mm from the box centre, bottom z "
          f"{r['target_bottom_z']}, tilt {r['target_tilt_deg']} deg, chassis contacts {r['chassis_contacts']}, "
          f"reason {r['reason']!r}")
PYEOF
echo
if [ "$FAILS" -eq 0 ]; then echo "GATE P11: ALL PASS"; else echo "GATE P11: $FAILS FAILURE(S)"; fi
exit "$FAILS"
