"""Launch SteveROS humanoid in MuJoCo simulation with ros2_control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    suspended = LaunchConfiguration("suspended")

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("steveros_description"), "urdf", "steveros.urdf.xacro"]
    )
    controllers_config = PathJoinSubstitution(
        [FindPackageShare("steveros_mujoco"), "config", "controllers_sim.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("steveros_description"), "rviz", "steveros.rviz"]
    )

    mujoco_scene = PythonExpression([
        "'scene_suspended.xml' if '", suspended, "' == 'true' else 'scene.xml'",
    ])

    robot_description = Command([
        "xacro ", urdf_path,
        " use_mock_hardware:=false",
        " sim_mujoco:=true",
        " mujoco_scene:=", mujoco_scene,
    ])

    ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_config,
            {"use_sim_time": True},
        ],
        output="both",
        on_exit=Shutdown(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        parameters=[{"use_sim_time": True}],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    right_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_leg_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    left_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_leg_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

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
        arguments=["-d", rviz_config, "-f", "floating_base_link"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz",
            ),
            DeclareLaunchArgument(
                "suspended",
                default_value="true",
                description="Suspend robot in air (no free joint / ground contact)",
            ),
            ros2_control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            delay_controllers,
            rviz_node,
        ]
    )
