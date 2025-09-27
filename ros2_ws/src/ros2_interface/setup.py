from setuptools import setup
import os
import sys

#sys.path.insert(0, os.path.expanduser('/root/core_algorithms'))

package_name = 'ros2_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=['interface_nodes', 'ai_models_nodes'],
    data_files=[
        ('share/ros2_interface/launch', ['launch/simulation_launch.py', 'launch/gru_infer_launch.py']),
        ('share/ros2_interface/config', [
                              'config/gru_params.yaml', 
                              'config/ekf_params.yaml',
                              'config/ukf_params.yaml',
                              'config/log_replay_params.yaml'
                             ]),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Test',
    maintainer_email='test@email.com',
    description='ROS2 interface Python package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log_replay_node = interface_nodes.log_replay_node:main',
            'ekf_node = ai_models_nodes.ekf_node:main',
            'ukf_node = ai_models_nodes.ukf_node:main',
            'gru_infer_node = ai_models_nodes.gru_infer_node:main',
        ],
    },
)
