from setuptools import setup

package_name = 'ros2_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=['interface_nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS2 interface Python package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = interface_nodes.sensor_node:main',
        ],
    },
)
