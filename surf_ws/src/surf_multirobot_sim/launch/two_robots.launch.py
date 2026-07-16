from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('surf_multirobot_sim'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))

    world_file = package_share / 'worlds' / 'outdoor.sdf'
    robot_file = package_share / 'models' / 'diffbot' / 'model.sdf'
    bridge_file = package_share / 'config' / 'bridge.yaml'
    bonxai_file = package_share / 'config' / 'bonxai.yaml'
    fast_lio_file = package_share / 'config' / 'fast_lio_sim.yaml'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 3 {world_file}'}.items(),
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
    )

    shared_cloud_mux = Node(
        package='surf_multirobot_sim',
        executable='shared_cloud_mux',
        name='shared_cloud_mux',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    bonxai = Node(
        package='bonxai_ros',
        executable='bonxai_server_node',
        name='bonxai_server_node',
        output='screen',
        parameters=[str(bonxai_file)],
    )

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
        gazebo,
        rviz,
        TimerAction(period=2.0, actions=[robot1, robot2]),
        TimerAction(
            period=3.0,
            actions=[
                bridge, shared_cloud_mux, bonxai,
                *fast_lio_nodes, *ground_truth_bridges,
                *localization_nodes, *static_transforms,
            ],
        ),
    ])
