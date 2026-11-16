# Sourced by every gate script. Isolates the gate from any other ROS 2 / Gazebo session.
#
# Why: on 2026-10-22 a GUI simulation was open while gate_p03 ran. Both sims shared
# /cmd_vel (the gate drove BOTH robots), /scan (SLAM fused scans from two robots),
# /clock and the ground-truth topics, which corrupted the map and tipped the GUI robot.
#
#  - ROS_DOMAIN_ID: gate nodes only see gate nodes (ROS 2 / DDS side).
#  - IGN_PARTITION / GZ_PARTITION: gate Gazebo only talks to the gate bridge (Gazebo side).
#  - Another Gazebo still costs CPU/GPU (lower RTF), so by default the gate refuses to start.
#    Override with ALLOW_OTHER_SIM=1 if you really want both.

export ROS_DOMAIN_ID="${GATE_DOMAIN_ID:-77}"
export IGN_PARTITION="gate_$$"
export GZ_PARTITION="$IGN_PARTITION"

# Match only real simulator processes: the command line must START with the simulator
# (Fortress runs as "ruby /usr/bin/ign gazebo ..."), not merely mention it.
SIM_PATTERN='^(ruby[^ ]* )?[^ ]*(ign gazebo|ign-gazebo|gz sim|gzserver)'
OTHER_SIM=$(pgrep -af "$SIM_PATTERN")
if [ -n "$OTHER_SIM" ] && [ "${ALLOW_OTHER_SIM:-0}" != "1" ]; then
  echo "FAIL  another simulator is running; close it (Ctrl-C its launch) and rerun:"
  echo "$OTHER_SIM" | sed 's/^/        /'
  echo "      (or ALLOW_OTHER_SIM=1 to run anyway; topics are isolated, but RTF will drop)"
  exit 1
fi
ros2 daemon stop >/dev/null 2>&1   # the daemon caches the old domain's graph
echo "gate isolation: ROS_DOMAIN_ID=$ROS_DOMAIN_ID IGN_PARTITION=$IGN_PARTITION"
echo "  watch this run from another terminal (GUI client only, no second sim):"
echo "    IGN_PARTITION=$IGN_PARTITION ign gazebo -g"
echo "    ROS_DOMAIN_ID=$ROS_DOMAIN_ID rviz2   # after: source install/setup.bash"

# Kill every simulator process. Safe at gate shutdown: the gate refused to start while any
# simulator was running, so whatever runs now is the gate's own. Needed because in GUI mode
# Gazebo forks a server that renames itself "ign gazebo server" and can outlive the launch
# (seen 2026-10-22 after a GATE_GUI=1 run).
kill_gate_sims() {
  pkill -INT -f "$SIM_PATTERN" 2>/dev/null; sleep 2
  pkill -9 -f "$SIM_PATTERN" 2>/dev/null
  pgrep -af "$SIM_PATTERN" >/dev/null && echo "WARN  simulator processes still alive: $(pgrep -af "$SIM_PATTERN" | head -3)"
  return 0
}
