"""Launch KBot 20-DOF humanoid with ros2_control, controllers, and optional RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_rviz = LaunchConfiguration("use_rviz")

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("steveros_description"), "urdf", "steveros.urdf.xacro"]
    )
    controllers_config = PathJoinSubstitution(
        [FindPackageShare("steveros_bringup"), "config", "controllers.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("steveros_description"), "rviz", "steveros.rviz"]
    )

    robot_description = Command([
        "xacro ", urdf_path, " use_mock_hardware:=", use_mock_hardware
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

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )

    right_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_leg_controller", "--controller-manager", "/controller_manager"],
    )

    left_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_leg_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawn all limb controllers after joint_state_broadcaster is up
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                right_arm_controller_spawner,
                left_arm_controller_spawner,
                right_leg_controller_spawner,
                left_leg_controller_spawner,
            ],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="Use mock hardware (no CAN bus needed)",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz",
            ),
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_controllers,
            rviz_node,
        ]
    )
