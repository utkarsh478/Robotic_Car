cmake_minimum_required(VERSION 3.8)
project(my_robot_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
# dev tools (gen_acm.py, check_named_states.py) are run from src/, see their docstrings
install(PROGRAMS scripts/gen_acm.py scripts/check_named_states.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/robot_collision.py DESTINATION lib/${PROJECT_NAME})

ament_package()
