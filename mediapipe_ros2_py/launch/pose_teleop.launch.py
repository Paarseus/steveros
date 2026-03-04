"""Launch camera + MediaPipe pose + pose-to-joints bridge for arm teleop."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device', default_value='/dev/video4',
        description='Video device (use v4l2-ctl --list-devices to find RGB cam)')
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='/image_raw')
    topic_prefix_arg = DeclareLaunchArgument(
        'topic_prefix', default_value='/mediapipe')
    ema_alpha_arg = DeclareLaunchArgument(
        'ema_alpha', default_value='0.3')
    traj_dur_arg = DeclareLaunchArgument(
        'trajectory_duration', default_value='0.1')

    video_device = LaunchConfiguration('video_device')
    image_topic = LaunchConfiguration('image_topic')
    topic_prefix = LaunchConfiguration('topic_prefix')
    ema_alpha = LaunchConfiguration('ema_alpha')
    traj_dur = LaunchConfiguration('trajectory_duration')

    # v4l2 camera node
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
        }],
    )

    # MediaPipe pose detection
    mp_node = Node(
        package='mediapipe_ros2_py',
        executable='mp_node',
        name='mp_node',
        output='screen',
        parameters=[{
            'model': 'pose',
            'image_topic': image_topic,
            'topic_prefix': topic_prefix,
            'publish_debug_image': True,
        }],
    )

    # Pose-to-joints bridge
    bridge_node = Node(
        package='mediapipe_ros2_py',
        executable='pose_to_joints',
        name='pose_to_joints',
        output='screen',
        parameters=[{
            'ema_alpha': ema_alpha,
            'trajectory_duration': traj_dur,
        }],
    )

    return LaunchDescription([
        video_device_arg,
        image_topic_arg,
        topic_prefix_arg,
        ema_alpha_arg,
        traj_dur_arg,
        camera_node,
        mp_node,
        bridge_node,
    ])
