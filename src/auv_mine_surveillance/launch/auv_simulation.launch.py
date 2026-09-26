"""
AUV Mine Surveillance — Master Launch File
ROS2 Humble + Gazebo Classic | ARA 306
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess,
    LogInfo, TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare('auv_mine_surveillance')

    # ── Launch Arguments ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('use_gui',       default_value='true'),
        DeclareLaunchArgument('use_rviz',      default_value='false'),
        DeclareLaunchArgument('enable_slam',   default_value='true'),
        DeclareLaunchArgument('enable_logger', default_value='true'),
        DeclareLaunchArgument('world_file',    default_value='minefield.world'),
        DeclareLaunchArgument('log_directory', default_value='/tmp/auv_mission_logs'),
        DeclareLaunchArgument('spawn_x',       default_value='0.0'),
        DeclareLaunchArgument('spawn_y',       default_value='0.0'),
        DeclareLaunchArgument('spawn_z',       default_value='-20.0'),
    ]

    use_gui       = LaunchConfiguration('use_gui')
    use_rviz      = LaunchConfiguration('use_rviz')
    enable_slam   = LaunchConfiguration('enable_slam')
    enable_logger = LaunchConfiguration('enable_logger')
    world_file    = LaunchConfiguration('world_file')
    log_directory = LaunchConfiguration('log_directory')
    spawn_x       = LaunchConfiguration('spawn_x')
    spawn_y       = LaunchConfiguration('spawn_y')
    spawn_z       = LaunchConfiguration('spawn_z')

    # ── URDF via Xacro — FIX: wrap in ParameterValue ─────────────────
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([pkg_share, 'urdf', 'auv_surveillance.urdf.xacro']),
        ]),
        value_type=str
    )

    robot_description = {'robot_description': robot_description_content}

    # ── 1. Gazebo Classic (headless) ──────────────────────────────────
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             PathJoinSubstitution([pkg_share, 'worlds', world_file]),
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so'],
        output='screen',
        condition=UnlessCondition(use_gui)
    )

    # ── 1b. Gazebo Classic (with GUI) ─────────────────────────────────
    gazebo_gui = ExecuteProcess(
        cmd=['gazebo', '--verbose',
             PathJoinSubstitution([pkg_share, 'worlds', world_file]),
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so'],
        output='screen',
        condition=IfCondition(use_gui)
    )

    # ── 2. Joint State Publisher ──────────────────────────────────────
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # ── 3. Robot State Publisher ──────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    # ── 4. Static TF: map → odom ──────────────────────────────────────
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': True}]
    )

    # ── 5. Spawn AUV (wait 3s for Gazebo) ────────────────────────────
    spawn_auv = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_auv',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'auv_surveillance',
                    '-x', spawn_x,
                    '-y', spawn_y,
                    '-z', spawn_z,
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
            )
        ]
    )

    # ── 6. AUV Localisation node (DVL + IMU) ─────────────────────────
    auv_localization = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='auv_mine_surveillance',
                executable='auv_localization_node.py',
                name='auv_localization',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # ── 7. robot_localization EKF node ────────────────────────────────
    ekf_node = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    PathJoinSubstitution([pkg_share, 'config', 'ekf_config.yaml']),
                    {'use_sim_time': True},
                ],
                remappings=[('/odometry/filtered', '/auv/odom_ekf')]
            )
        ]
    )

    # ── 8. SLAM Toolbox ───────────────────────────────────────────────
    slam_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='sync_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                condition=IfCondition(enable_slam),
                parameters=[
                    PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox_config.yaml']),
                    {'use_sim_time': True},
                ],
                remappings=[
                    ('/scan', '/auv/sonar/scan'),
                    ('/odom', '/auv/odom'),
                ]
            )
        ]
    )

    # ── 9. Mission Logger ─────────────────────────────────────────────
    logger_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='auv_mine_surveillance',
                executable='auv_mission_logger.py',
                name='auv_mission_logger',
                output='screen',
                condition=IfCondition(enable_logger),
                parameters=[
                    {'use_sim_time': True},
                    {'log_directory': log_directory},
                    {'log_rate_hz': 5.0},
                    {'generate_excel': True},
                    {'mine_confidence_threshold': 0.70},
                    {'ship_confidence_threshold': 0.55},
                ],
            )
        ]
    )

    # ── 10. RViz2 (optional) ──────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}]
    )

    # ── Info banner ───────────────────────────────────────────────────
    info_msg = LogInfo(msg=[
        '\n╔══════════════════════════════════════════╗\n',
        '║  AUV Mine Surveillance Simulation        ║\n',
        '║  ROS2 Humble + Gazebo Classic            ║\n',
        '╚══════════════════════════════════════════╝\n',
        '  World:   minefield.world\n',
        '  AUV:     Torpedo-style, 4 thrusters\n',
        '  Sensors: IMU + DVL + Depth + FLS Sonar\n',
        '  EKF:     robot_localization (15-state)\n',
        '  SLAM:    slam_toolbox (synchronous)\n',
        '  Logger:  CSV + Excel output\n',
    ])

    return LaunchDescription(
        args + [
            info_msg,
            gazebo_server,
            gazebo_gui,
            joint_state_pub,
            robot_state_pub,
            static_tf_map_odom,
            spawn_auv,
            auv_localization,
            ekf_node,
            slam_node,
            logger_node,
            rviz_node,
        ]
    )