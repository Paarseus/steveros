"""Launch URDF visualization with joint_state_publisher_gui and RViz."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("steveros_description"), "urdf", "steveros.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("steveros_description"), "rviz", "steveros.rviz"]
    )

    robot_description = {"robot_description": Command(["xacro ", urdf_path])}

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
