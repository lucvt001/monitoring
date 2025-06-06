import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

launch_args = [
    DeclareLaunchArgument('rosbag', description='True to start ros2bag record.'),
]

def generate_launch_description():

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        condition=IfCondition(PythonExpression([LaunchConfiguration('rosbag')]))
    )
    
    mqtt_client = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('mqtt_client'), 'launch', 'standalone.launch.ros2.xml')
        ), launch_arguments={'params_file': os.path.join(get_package_share_directory('monitoring'), 'config', 'mqtt_live_monitoring.yaml')}.items()
    )

    # Read the yaml file and broadcast all the static transforms: leader -> agentX
    formation_shape_broadcaster = Node(
        name='formation_shape_broadcaster',
        executable='formation_shape_broadcaster',
        package='formation_controller',
        parameters=[{
            'yaml_file_path': os.path.join(get_package_share_directory('formation_controller'), 'config', 'formation_shape.yaml'),
            'leader_frame': 'leader1/gps_link',
        }]
    )

    leader1_tf = Node(
        name='leader1_tf',
        executable='gps_heading_to_tf',
        package='formation_controller',
        output='screen',
        parameters=[{
            'origin_gps_topic': 'origin_gps',
            'current_gps_topic': 'leader1/gps',
            'heading_topic': 'leader1/heading',
            'parent_frame': 'world',
            'child_frame': 'leader1/gps_link',
        }]
    )

    leader2_tf = Node(
        name='leader2_tf',
        executable='gps_heading_to_tf',
        package='formation_controller',
        output='screen',
        parameters=[{
            'origin_gps_topic': 'origin_gps',
            'current_gps_topic': 'leader2/gps',
            'heading_topic': 'leader2/heading',
            'parent_frame': 'world',
            'child_frame': 'leader2/gps_link',
        }]
    )

    follower_tf = Node(
        name='follower_tf',
        executable='gps_heading_to_tf',
        package='formation_controller',
        output='screen',
        parameters=[{
            'origin_gps_topic': 'origin_gps',
            'current_gps_topic': 'follower/gps',
            'heading_topic': 'follower/heading',
            'parent_frame': 'world',
            'child_frame': 'follower/gps_link',
        }]
    )

    leader1_path_plotter = Node(
        name='leader1_path_plotter',
        executable='tf_to_path',
        package='monitoring',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame': 'leader1/gps_link',
            'path_topic': 'leader1/path/gps_path',
            'publish_rate': 2.0  # Publish rate in Hz
        }]
    )

    leader2_path_plotter = Node(
        name='leader2_path_plotter',
        executable='tf_to_path',
        package='monitoring',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame': 'leader2/gps_link',
            'path_topic': 'leader2/path/gps_path',
            'publish_rate': 2.0  # Publish rate in Hz
        }]
    )

    follower_target_path_plotter = Node(
        name='follower_target_path_plotter',
        executable='tf_to_path',
        package='monitoring',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame': 'follower/target',
            'path_topic': 'leader1/follower_target_path',
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
            'path_topic': 'follower/path/gps_path',
            'publish_rate': 2.0  # Publish rate in Hz
        }]
    )

    return LaunchDescription([
        *launch_args,
        mqtt_client,
        formation_shape_broadcaster,
        leader1_tf,
        leader2_tf,
        follower_tf,
        leader1_path_plotter,
        leader2_path_plotter,
        follower_path_plotter,
        follower_target_path_plotter,
        TimerAction(period=4.0, actions=[rosbag_record]),
    ])
