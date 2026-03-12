"""Launch IK node with forward position controllers.

Loads, activates forward controllers and deactivates trajectory controllers
for jitter-free real-time IK streaming.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('steveros_ik'),
        'config', 'ik_upper_body.yaml',
    ])

    # Load the forward controllers (they're defined in controllers.yaml but not spawned by bringup)
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

    # After both are loaded, switch: deactivate trajectory, activate forward
    switch_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'switch_controllers',
            '--deactivate', 'right_arm_controller', 'left_arm_controller',
            '--activate', 'right_arm_forward_controller', 'left_arm_forward_controller',
        ],
        output='screen',
    )

    ik_node = Node(
        package='steveros_ik',
        executable='ik_node',
        name='ik_node',
        parameters=[config],
        output='screen',
    )

    # Chain: load right -> load left -> switch -> ik_node starts immediately
    load_then_switch = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_right_fwd,
            on_exit=[spawn_left_fwd],
        )
    )

    switch_after_load = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_left_fwd,
            on_exit=[switch_controllers],
        )
    )

    return LaunchDescription([
        spawn_right_fwd,
        load_then_switch,
        switch_after_load,
        ik_node,
    ])
