cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds rviz models
  DESTINATION share/${PROJECT_NAME}
)

# Executable Python scripts -> lib/<pkg> so `ros2 run my_robot_bringup evaluator.py` works.
install(
  PROGRAMS scripts/evaluator.py scripts/grasp_attach_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
