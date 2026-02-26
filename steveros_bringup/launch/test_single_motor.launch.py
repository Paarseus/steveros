"""Launch single-motor test with ros2_control on real hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    motor_id = LaunchConfiguration("motor_id")
    motor_type = LaunchConfiguration("motor_type")
    can_interface = LaunchConfiguration("can_interface")

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("steveros_description"), "urdf", "test_single_motor.urdf.xacro"]
    )
    controllers_config = PathJoinSubstitution(
        [FindPackageShare("steveros_bringup"), "config", "test_single_motor_controllers.yaml"]
    )

    robot_description = Command([
        "xacro ", urdf_path,
        " motor_id:=", motor_id,
        " motor_type:=", motor_type,
        " can_interface:=", can_interface,
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_config,
        ],
        output="both",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    test_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["test_controller", "--controller-manager", "/controller_manager"],
    )

    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[test_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("motor_id", default_value="21"),
            DeclareLaunchArgument("motor_type", default_value="RS03"),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_controller,
        ]
    )
