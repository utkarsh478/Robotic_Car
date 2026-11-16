#!/usr/bin/env bash
# Save the current slam_toolbox map and convert it to the world frame (map == world).
# Run while mapping.launch.xml is up:   bash tools/save_map.sh
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAPS="$WS/src/my_robot_navigation/maps"
mkdir -p "$MAPS"
ros2 run nav2_map_server map_saver_cli -f "$MAPS/mission_world_raw" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true \
  || { echo "FAIL  map_saver_cli"; exit 1; }
python3 "$WS/tools/map_to_world.py" "$MAPS/mission_world_raw.yaml" "$MAPS/mission_world.yaml" --dx -3.0 --dy -2.0
