import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():

    bag_path = '/home/smarc2user/colcon_ws/real_bags/rosbag2_2025_04_11-13_35_23/rosbag2_2025_04_11-13_35_23_0.db3'

    # Replay rosbag
    rosbag_replay = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--start-offset', '2.0', '--rate', '2.0'],
    )

    foxglove = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    agent0_path_plotter = Node(
        name='agent0_path_plotter',
        executable='tf_to_path',
        package='monitoring',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame': 'agent0/gps_link',
            'path_topic': 'leader/path',
            'publish_rate': 2.0  # Publish rate in Hz
        }]
    )

    follower_path_plotter = Node(
        name='follower_path_plotter',
        executable='tf_to_path',
        package='monitoring',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame': 'follower/gps_link',
            'path_topic': 'follower/path',
            'publish_rate': 2.0  # Publish rate in Hz
        }]
    )

    return LaunchDescription([
        foxglove,
        rosbag_replay,
        agent0_path_plotter,
        follower_path_plotter,
    ])