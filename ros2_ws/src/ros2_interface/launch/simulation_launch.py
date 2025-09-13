# ros2_interface/launch/simulation_launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ros2_interface'
    base_dir = os.path.join(os.getenv('ROS_PACKAGE_PATH', '/root/ros2_ws/src'), package_name)
    cfg_dir = os.path.join(base_dir, "config")

    # env variable (.env has BAG_DIR=/root/data/rosbag_logs）
    bag_dir_default = os.getenv('BAG_DIR', '/root/data/rosbag_logs')
    # data logs CSV/JSON data replay path
    log_dir_default = '/root/data/sample_inputs'

    # ---- add changeable parameters ----
    use_log_replay = DeclareLaunchArgument('use_log_replay', default_value='true',
        description='Y/N enable log_replay_node（CSV/JSON etc）')
    play_bag = DeclareLaunchArgument('play_bag', default_value='false',
        description='Y/N replay rosbag2')
    bag = DeclareLaunchArgument('bag', default_value=os.path.join(bag_dir_default, 'sample_run'),
        description='rosbag2 folder path（including metadata.yaml）')
    log_dir = DeclareLaunchArgument('log_dir', default_value=log_dir_default,
        description='log_replay_node default data folder')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false',
        description='Y/N use simulation time（if rosbag --clock, recommand true）')

    return LaunchDescription([
        use_log_replay, play_bag, bag, log_dir, use_sim_time,

        # replay files（only enable when use_log_replay=true ）
        Node(
            package=package_name,
            executable='log_replay_node',
            name='log_replay_node',
            output='screen',
            parameters=[
                os.path.join(cfg_dir, 'log_replay_params.yaml'),
                {'log_root': LaunchConfiguration('log_dir'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            condition=IfCondition(LaunchConfiguration('use_log_replay'))
        ),

        Node(
            package=package_name,
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                os.path.join(cfg_dir, 'ekf_params.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        Node(
            package=package_name,
            executable='ukf_node',
            name='ukf_node',
            output='screen',
            parameters=[
                os.path.join(cfg_dir, 'ukf_params.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),

        # rosbag replay（only activate when play_bag=true )
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play',
                 LaunchConfiguration('bag'),
                 '-l', '--clock'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('play_bag'))
        ),
    ])
