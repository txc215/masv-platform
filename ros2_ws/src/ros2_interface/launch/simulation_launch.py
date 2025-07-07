from launch import LaunchDescription
from launch_ros.actions import Node
import os




def generate_launch_description():

    package_name = 'ros2_interface'
    base_dir = os.path.join(os.getenv('ROS_PACKAGE_PATH', '/root/ros2_ws/src'), package_name)

    cfg_dir = os.path.join(base_dir, "config")

    return LaunchDescription([
        Node(
            package=package_name,
            executable='log_replay_node',
            name='log_replay_node',
            output='screen',
            parameters=[os.path.join(cfg_dir, 'log_replay_params.yaml')]
        ),
        Node(
            package=package_name,
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[os.path.join(cfg_dir, 'ekf_params.yaml')]
        ),
        Node(
            package=package_name,
            executable='ukf_node',
            name='ukf_node',
            output='screen',
            parameters=[os.path.join(cfg_dir, 'ukf_params.yaml')]
        )
    ])
