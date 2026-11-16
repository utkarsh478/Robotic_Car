cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

# Every directory with runtime files must be installed, or launch files fail with
# "file not found" even though the file exists in src/.
# meshes/ only if meshes are ever used.
install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
