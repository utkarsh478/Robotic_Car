"""move_group for my_robot (Phase 7).  Start it AFTER Gazebo (my_robot_gazebo.launch.xml).

- robot_description is built from the SAME xacro and args as Gazebo/robot_state_publisher
  (my_robot_description/urdf/my_robot.urdf.xacro, with_arm:=true) -> SPEC pitfall 7.
- No robot_state_publisher here: the Gazebo launch already runs the only one -> pitfall 8.
- use_sim_time true; trajectories go to arm_controller / gripper_controller (gz_ros2_control).

  ros2 launch my_robot_moveit_config move_group.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config():
    xacro_file = os.path.join(get_package_share_directory("my_robot_description"), "urdf", "my_robot.urdf.xacro")
    return (
        MoveItConfigsBuilder("my_robot", package_name="my_robot_moveit_config")
        .robot_description(file_path=xacro_file, mappings={"with_arm": "true"})
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


def generate_launch_description():
    cfg = moveit_config()
    use_sim_time = LaunchConfiguration("use_sim_time")
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            cfg.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
                "octomap_resolution": 0.05,   # unused (no octomap sensor); silences the default-0.1 warning
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        move_group,
    ])
