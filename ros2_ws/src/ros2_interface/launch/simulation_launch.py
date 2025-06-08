from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_interface',
            executable='imu_node',
            name='imu_node',
            parameters=['config/imu_params.yaml'],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='ros2_interface',
            executable='gnss_node',
            name='gnss_node',
            parameters=['config/gnss_params.yaml'],
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
