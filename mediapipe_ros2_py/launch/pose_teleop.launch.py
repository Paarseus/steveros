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
    traj_dur_arg = DeclareLaunchArgument(
        'trajectory_duration', default_value='0.1')

    # One Euro Filter parameters
    filter_min_cutoff_arg = DeclareLaunchArgument(
        'filter_min_cutoff', default_value='1.0',
        description='One Euro Filter min cutoff (Hz). Lower = less jitter at rest')
    filter_beta_arg = DeclareLaunchArgument(
        'filter_beta', default_value='0.007',
        description='One Euro Filter beta. Higher = less lag during fast motion')
    filter_yaw_min_cutoff_arg = DeclareLaunchArgument(
        'filter_yaw_min_cutoff', default_value='0.5',
        description='Heavier filtering on shoulder yaw (depth-derived, noisier)')
    dead_zone_arg = DeclareLaunchArgument(
        'dead_zone', default_value='0.015',
        description='Dead zone threshold (rad) to suppress micro-oscillations')

    # MediaPipe confidence thresholds
    min_tracking_confidence_arg = DeclareLaunchArgument(
        'min_tracking_confidence', default_value='0.65',
        description='Min tracking confidence (higher = faster re-detection)')

    video_device = LaunchConfiguration('video_device')
    image_topic = LaunchConfiguration('image_topic')
    topic_prefix = LaunchConfiguration('topic_prefix')
    traj_dur = LaunchConfiguration('trajectory_duration')
    filter_min_cutoff = LaunchConfiguration('filter_min_cutoff')
    filter_beta = LaunchConfiguration('filter_beta')
    filter_yaw_min_cutoff = LaunchConfiguration('filter_yaw_min_cutoff')
    dead_zone = LaunchConfiguration('dead_zone')
    min_tracking_confidence = LaunchConfiguration('min_tracking_confidence')

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
            'min_tracking_confidence': min_tracking_confidence,
        }],
    )

    # Pose-to-joints bridge
    bridge_node = Node(
        package='mediapipe_ros2_py',
        executable='pose_to_joints',
        name='pose_to_joints',
        output='screen',
        parameters=[{
            'trajectory_duration': traj_dur,
            'filter_min_cutoff': filter_min_cutoff,
            'filter_beta': filter_beta,
            'filter_yaw_min_cutoff': filter_yaw_min_cutoff,
            'dead_zone': dead_zone,
        }],
    )

    return LaunchDescription([
        video_device_arg,
        image_topic_arg,
        topic_prefix_arg,
        traj_dur_arg,
        filter_min_cutoff_arg,
        filter_beta_arg,
        filter_yaw_min_cutoff_arg,
        dead_zone_arg,
        min_tracking_confidence_arg,
        camera_node,
        mp_node,
        bridge_node,
    ])
