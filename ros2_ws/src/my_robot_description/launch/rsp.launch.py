"""robot_state_publisher with the xacro-expanded URDF.

Python, not XML, on purpose: Humble's XML <param value="$(command ...)"> tries to parse
the URDF as YAML and fails on any comment containing "word:". ParameterValue(...,
value_type=str) passes it through untouched. Included by display.launch.xml and
my_robot_bringup/launch/my_robot_gazebo.launch.xml.

Args: use_sim_time (default false), with_arm (default true).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_arm = LaunchConfiguration("with_arm")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("my_robot_description"), "urdf", "my_robot.urdf.xacro"])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " with_arm:=", with_arm]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("with_arm", default_value="true"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ])
