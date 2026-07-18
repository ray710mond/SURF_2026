from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('surf_multirobot_sim'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))

    world_file = package_share / 'worlds' / 'outdoor.sdf'
    robot_file = package_share / 'models' / 'diffbot' / 'model.sdf'
    bridge_file = package_share / 'config' / 'bridge.yaml'
    bonxai_file = package_share / 'config' / 'bonxai.yaml'
    communication_file = package_share / 'config' / 'communication.yaml'
    fast_lio_file = package_share / 'config' / 'fast_lio_sim.yaml'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r -v 3 ', LaunchConfiguration('gz_extra_args'), ' ', str(world_file),
            ],
        }.items(),
    )

    robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'outdoor',
            '-name', 'robot1',
            '-file', str(robot_file),
            '-x', '-4.0', '-y', '-1.5', '-z', '0.001', '-Y', '0.0',
        ],
    )

    robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'outdoor',
            '-name', 'robot2',
            '-file', str(robot_file),
            '-x', '4.0', '-y', '1.5', '-z', '0.001', '-Y', '3.14159',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {'config_file': str(bridge_file)},
            {'use_sim_time': True},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(package_share / 'config' / 'two_robots.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    bonxai_nodes = [
        Node(
            package='bonxai_ros',
            executable='bonxai_server_node',
            namespace=robot_name,
            name='bonxai_server_node',
            output='screen',
            parameters=[
                str(bonxai_file),
                {
                    'topic_in': f'/{robot_name}/points',
                    'delta_topic_in': f'/{robot_name}/comm/voxel_delta',
                    'map_storage.path': str(package_share / 'maps' / 'static_map.bonxai'),
                    'map_storage.save_on_shutdown': True,
                },
            ],
        )
        for robot_name in ('robot1', 'robot2')
    ]

    communication_senders = [
        Node(
            package='surf_multirobot_sim',
            executable='voxel_delta_sender',
            namespace=robot_name,
            name='voxel_delta_sender',
            output='screen',
            parameters=[str(communication_file), {'robot_name': robot_name}],
        )
        for robot_name in ('robot1', 'robot2')
    ]

    communication_receivers = [
        Node(
            package='surf_multirobot_sim',
            executable='voxel_delta_receiver',
            namespace=robot_name,
            name='voxel_delta_receiver',
            output='screen',
            parameters=[str(communication_file), {'robot_name': robot_name}],
        )
        for robot_name in ('robot1', 'robot2')
    ]

    def link_node(source, destination, radio, **profile):
        reliable = radio == 'wifi'
        return Node(
            package='surf_multirobot_sim',
            executable='link_emulator',
            name=f'{source}_to_{destination}_{radio}',
            output='screen',
            parameters=[{
                'link_name': f'{source}_to_{destination}_{radio}',
                'input_topic': f'/{source}/comm/{radio}_tx',
                'output_topic': f'/{destination}/comm/{radio}_rx',
                'metrics_topic': f'/{source}/comm/{radio}_metrics',
                'trace_path': LaunchConfiguration(f'{radio}_trace'),
                'reliable_qos': reliable,
                **profile,
            }],
        )

    link_emulators = []
    for source, destination in (('robot1', 'robot2'), ('robot2', 'robot1')):
        link_emulators.extend([
            link_node(
                source, destination, 'halow', bandwidth_mbps=4.0,
                latency_ms=25.0, jitter_ms=5.0, loss_percent=0.5,
                queue_depth=2, freshness_first=True,
            ),
            link_node(
                source, destination, 'wifi', bandwidth_mbps=80.0,
                latency_ms=8.0, jitter_ms=2.0, loss_percent=0.1,
                queue_depth=20, freshness_first=False,
            ),
        ])

    fast_lio_nodes = [
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            namespace=robot_name,
            name='fast_lio',
            output='screen',
            parameters=[
                str(fast_lio_file),
                {
                    'common.lid_topic': f'/{robot_name}/points',
                    'common.imu_topic': f'/{robot_name}/imu',
                    'common.map_frame': f'{robot_name}/odom',
                    'common.body_frame': f'{robot_name}/base_link',
                },
            ],
            remappings=[
                ('/Odometry', f'/{robot_name}/lio/odom'),
                ('/path', f'/{robot_name}/lio/path'),
                ('/cloud_registered', f'/{robot_name}/lio/cloud_registered'),
                ('/cloud_registered_body', f'/{robot_name}/lio/cloud_body'),
                ('/cloud_effected', f'/{robot_name}/lio/cloud_effected'),
                ('/Laser_map', f'/{robot_name}/lio/map'),
            ],
        )
        for robot_name in ('robot1', 'robot2')
    ]

    ground_truth_bridges = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'{robot_name}_ground_truth_bridge',
            output='screen',
            arguments=[
                f'/model/{robot_name}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            remappings=[
                (f'/model/{robot_name}/pose', f'/{robot_name}/ground_truth_tf'),
            ],
        )
        for robot_name in ('robot1', 'robot2')
    ]

    localization_nodes = [
        Node(
            package='surf_multirobot_sim',
            executable='map_odom_localizer',
            namespace=robot_name,
            name='map_odom_localizer',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name,
                'map_frame': 'map',
                'base_z_offset': 0.25,
            }],
        )
        for robot_name in ('robot1', 'robot2')
    ]

    static_transforms = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'map',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.50',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'robot1/base_link',
                '--child-frame-id', 'robot1/lidar_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.50',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'robot2/base_link',
                '--child-frame-id', 'robot2/lidar_link',
            ],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'halow_trace', default_value='',
            description='CSV trace used to replay measured HaLow link conditions',
        ),
        DeclareLaunchArgument(
            'wifi_trace', default_value='',
            description='CSV trace used to replay measured 5 GHz Wi-Fi link conditions',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with the two-robot mapping display',
        ),
        DeclareLaunchArgument(
            'gz_extra_args', default_value='',
            description='Additional Gazebo arguments; use -s for headless server mode',
        ),
        gazebo,
        rviz,
        TimerAction(period=2.0, actions=[robot1, robot2]),
        TimerAction(
            period=3.0,
            actions=[
                bridge,
                *bonxai_nodes,
                *communication_senders,
                *communication_receivers,
                *link_emulators,
                *fast_lio_nodes, *ground_truth_bridges,
                *localization_nodes, *static_transforms,
            ],
        ),
    ])
