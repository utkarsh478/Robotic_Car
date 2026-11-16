"""RViz with the MotionPlanning display for my_robot (run while move_group is up).

  ros2 launch my_robot_moveit_config moveit_rviz.launch.py
Plan in the MotionPlanning panel: Planning Group "arm", Goal State "stow"/"zero"/"transport",
Plan, then Execute. The orange interactive marker is position-only (4-DOF arm).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import importlib.util

_spec = importlib.util.spec_from_file_location(
    "my_robot_move_group_launch",
    os.path.join(get_package_share_directory("my_robot_moveit_config"), "launch", "move_group.launch.py"))
_mg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mg)


def generate_launch_description():
    cfg = _mg.moveit_config()
    rviz_file = os.path.join(get_package_share_directory("my_robot_moveit_config"), "rviz", "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            cfg.robot_description,
            cfg.robot_description_semantic,
            cfg.robot_description_kinematics,
            cfg.planning_pipelines,
            cfg.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=rviz_file),
        rviz,
    ])
