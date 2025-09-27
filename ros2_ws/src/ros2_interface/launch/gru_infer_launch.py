from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    cfg = os.path.join('/root/ros2_ws', 'src', 'ros2_interface', 'config', 'gru_params.yaml')
    return LaunchDescription([
        Node(
            package='ros2_interface',
            executable='gru_infer_node',
            name='gru_infer_node',
            output='screen',
            parameters=[cfg],
        )
    ])
