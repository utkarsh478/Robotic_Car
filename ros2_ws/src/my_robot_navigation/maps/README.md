# maps/
- `mission_world.yaml` / `.pgm` — the map Nav2 loads. **map frame = Gazebo world frame.**
  Produced by gate_p03.sh (or by hand, see launch/mapping.launch.xml).
- `mission_world_raw.*` — slam_toolbox output before the origin shift. Its map frame is the
  robot's start pose (-3, -2, 0); tools/map_to_world.py shifts the origin so map == world.
