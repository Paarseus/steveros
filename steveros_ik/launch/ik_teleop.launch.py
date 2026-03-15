"""Launch full VR teleoperation stack: bringup + Pink IK + teleop-xr.

Single command to launch everything:
  ros2 launch steveros_ik ik_teleop.launch.py

Pipeline:
  1. steveros bringup (mock hardware, ros2_control, robot_state_publisher)
  2. RViz (single instance)
  3. Load and activate forward controllers, deactivate trajectory controllers
  4. IK node (Pink QP differential IK at 100 Hz)
  5. XR bridge (engage/clutch, head-based axis mapping)
  6. teleop-xr ROS2 bridge (WebXR server on port 4443)

Then open https://<your-ip>:4443 in Quest browser.
Hold grip/trigger to engage arm control.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('ik_use_rviz')

    ik_config = PathJoinSubstitution([
        FindPackageShare('steveros_ik'),
        'config', 'ik_upper_body.yaml',
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare('steveros_description'),
        'rviz', 'steveros.rviz',
    ])

    # --- Include bringup WITHOUT its own RViz (we launch our own) ---
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('steveros_bringup'),
                'launch', 'steveros.launch.py',
            ])
        ),
        launch_arguments={
            'use_mock_hardware': 'true',
            'sim_mujoco': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    # --- Single RViz instance ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    # --- Controller switching (delayed to let bringup finish spawning) ---
    # Load forward controllers, then switch from trajectory to forward
    spawn_right_fwd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'right_arm_forward_controller'],
        output='screen',
    )
    spawn_left_fwd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'left_arm_forward_controller'],
        output='screen',
    )
    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'right_arm_controller', 'left_arm_controller',
            '--activate', 'right_arm_forward_controller', 'left_arm_forward_controller',
        ],
        output='screen',
    )

    # Chain: load right -> load left -> switch
    chain_left = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_right_fwd,
            on_exit=[spawn_left_fwd],
        )
    )
    chain_switch = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_left_fwd,
            on_exit=[switch_controllers],
        )
    )

    # Delay controller loading to give bringup time to start controller_manager
    delayed_controller_setup = TimerAction(
        period=8.0,
        actions=[spawn_right_fwd],
    )

    # --- IK node (Pink QP solver) ---
    ik_node = Node(
        package='steveros_ik',
        executable='ik_node',
        name='ik_node',
        parameters=[ik_config],
        output='screen',
    )

    # --- XR bridge (engage/clutch + head-based axis mapping) ---
    xr_bridge_node = Node(
        package='steveros_ik',
        executable='xr_bridge',
        name='xr_bridge',
        output='screen',
    )

    # --- teleop-xr ROS2 bridge (WebXR server + ROS2 topic publisher) ---
    # Use system Python (3.12) to match ROS Jazzy rclpy; conda python3 is 3.13
    teleop_xr = ExecuteProcess(
        cmd=['/usr/bin/python3', '-m', 'teleop_xr.ros2', '--mode', 'teleop'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ik_use_rviz', default_value='true',
            description='Launch RViz',
        ),

        # Bringup (no RViz)
        bringup,

        # Single RViz
        rviz_node,

        # Controller switching (delayed 8s for bringup)
        delayed_controller_setup,
        chain_left,
        chain_switch,

        # IK + teleop nodes (ik_node waits for /robot_description internally)
        ik_node,
        xr_bridge_node,
        teleop_xr,
    ])
