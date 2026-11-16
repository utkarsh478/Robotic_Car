"""pick_place_server with the same MoveIt description as move_group (Phase 9).
Start after Gazebo and move_group:
  ros2 launch my_robot_manipulation manipulation.launch.py
"""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_spec = importlib.util.spec_from_file_location(
    "my_robot_move_group_launch",
    os.path.join(get_package_share_directory("my_robot_moveit_config"), "launch", "move_group.launch.py"))
_mg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mg)


def generate_launch_description():
    cfg = _mg.moveit_config()
    params = os.path.join(get_package_share_directory("my_robot_manipulation"), "config", "manipulation_params.yaml")
    server = Node(
        package="my_robot_manipulation",
        executable="pick_place_server",
        name="pick_place_server",
        output="screen",
        parameters=[
            cfg.robot_description,
            cfg.robot_description_semantic,
            cfg.robot_description_kinematics,
            cfg.joint_limits,
            LaunchConfiguration("params"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params", default_value=params),
        server,
    ])
