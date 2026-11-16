# A perceived object, fused over several observations. SPEC §7.
std_msgs/Header header          # frame_id: map (pose expressed in map == Gazebo world, DC-9)
string label                    # "target" | "box"
geometry_msgs/Pose pose         # target: cylinder axis point at mid-height; box: centre of the box floor (inside)
float32 confidence              # 0..1
float32 position_stddev         # m, std-dev of the fused observations
uint32 observation_count
