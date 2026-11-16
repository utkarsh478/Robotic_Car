#!/usr/bin/env bash
# P11 task 5 (CLAUDE.md rule 4): the autonomy stack never sees ground truth.
#   bash tools/gates/check_isolation.sh          static check only
#   bash tools/gates/check_isolation.sh --live   also checks the running ROS graph
#                                                (run while full_mission.launch.xml is up)
# Static: no source/config/launch file of perception, manipulation, mission or navigation
#         mentions "evaluator", "set_pose" or "/model/" (Gazebo ground-truth / teleport topics).
# Live:   every /evaluator/* topic is subscribed only by the evaluator or a gate tool
#         (node names evaluator, pNN_*, gate_*), and the mission node's own subscriptions
#         include no /evaluator/* or /model/* topic.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS" || exit 1
FAILS=0
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILS=$((FAILS + 1)); }

HITS=$(grep -rnE "evaluator|set_pose|/model/" \
  src/my_robot_perception src/my_robot_manipulation src/my_robot_mission src/my_robot_navigation \
  --exclude-dir=__pycache__ --exclude-dir=.pytest_cache --exclude="*.pyc" 2>/dev/null)
if [ -z "$HITS" ]; then
  pass "static: no evaluator / set_pose / /model/ reference in perception, manipulation, mission, navigation"
else
  fail "static: ground-truth references in the autonomy packages:"; echo "$HITS" | head -20 | sed 's/^/      /'
fi

if [ "$1" = "--live" ]; then
  : "${ROS_DISTRO:?source the workspace first}"
  TOPICS=$(ros2 topic list 2>/dev/null | grep '^/evaluator/')
  if [ -z "$TOPICS" ]; then
    fail "live: no /evaluator/* topics in the graph (is the simulation running?)"
  fi
  for t in $TOPICS; do
    SUBS=$(ros2 topic info -v "$t" 2>/dev/null | awk '/^Node name:/ {n=$3} /^Endpoint type: SUBSCRIPTION/ {print n}' | sort -u)
    BAD=$(echo "$SUBS" | grep -vE '^(evaluator|p[0-9]+_.*|gate_.*|)$')
    if [ -z "$BAD" ]; then
      pass "live: $t subscribers: $(echo $SUBS | tr '\n' ' ')"
    else
      fail "live: $t is read by autonomy node(s): $(echo $BAD | tr '\n' ' ')"
    fi
  done
  if ros2 node list 2>/dev/null | grep -qx "/mission_fsm"; then
    MSUBS=$(ros2 node info /mission_fsm 2>/dev/null | awk '/Subscribers:/ {s=1; next} /Publishers:/ {s=0} s {print $1}' | tr -d ':')
    BAD=$(echo "$MSUBS" | grep -E '^/(evaluator|model|world)/')
    [ -z "$BAD" ] && pass "live: /mission_fsm subscribes to: $(echo $MSUBS | tr '\n' ' ')" \
                  || fail "live: /mission_fsm subscribes to ground truth: $BAD"
  else
    fail "live: /mission_fsm is not running"
  fi
fi
exit "$FAILS"
