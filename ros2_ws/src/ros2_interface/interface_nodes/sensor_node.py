import rclpy
from rclpy.node import Node
import sys
import os

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
sys.path.append(project_root)

from core_algorithms.EKF import SimpleHeadingEKF
from motion_models.vehicle.bicycle_model import BicycleModel


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.get_logger().info('SensorNode has started.')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
