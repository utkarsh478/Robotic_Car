cmake_minimum_required(VERSION 3.8)
project(my_robot_manipulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# CLAUDE.md: compile our code warning-clean
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# ---- analytic arm kinematics + grasp logic: pure C++17, no ROS dependency ----
add_library(arm_ik SHARED src/arm_ik.cpp src/grasp_logic.cpp)
target_include_directories(arm_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ---- CLI used by tools/gates/fk_vs_tf.py and the P7 reach sweep ----
add_executable(ik_cli src/ik_cli.cpp)
target_link_libraries(ik_cli arm_ik)

# ---- pick/place action server (Phase 9: pick) ----
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(pick_place_server src/pick_place_server.cpp)
target_link_libraries(pick_place_server arm_ik)
ament_target_dependencies(pick_place_server
  rclcpp rclcpp_action moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_msgs
  control_msgs geometry_msgs sensor_msgs shape_msgs std_msgs std_srvs tf2_ros tf2_geometry_msgs
  ros_gz_interfaces my_robot_interfaces)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS arm_ik EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ik_cli pick_place_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm_ik test/test_arm_ik.cpp)
  target_link_libraries(test_arm_ik arm_ik)
  ament_add_gtest(test_grasp_logic test/test_grasp_logic.cpp)
  target_link_libraries(test_grasp_logic arm_ik)
endif()

ament_export_include_directories(include)
ament_export_libraries(arm_ik)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
