import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Pose2D, PoseStamped
from EKF.BaseEKF import EKFAlgorithm
import numpy as np

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # init EKF
        x0 = np.array([0.0, 0.0, 0.0])
        P0 = np.eye(3) * 0.5
        self.ekf = EKFAlgorithm(x0, P0)
        self.ekf.set_observation_noise("gnss", np.eye(2) * 0.25)
        self.ekf.set_observation_noise("magnetometer", np.array([[0.01]]))

        self.prev_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(NavSatFix, "/gnss/fix", self.gnss_callback, 10)

        # Publisher
        self.state_pub = self.create_publisher(PoseStamped, "/ekf/state", 10)

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        v = 1.0  # 假設值或從其他來源取得
        yaw_rate = msg.angular_velocity.z
        u = np.array([v, yaw_rate])

        self.ekf.predict(u, dt, model="imu")
        self.publish_state()

    def gnss_callback(self, msg: NavSatFix):
        # Assume convert to local ENU over here
        x = msg.latitude * 111000  # simply calculate that every latitude is 111km
        y = msg.longitude * 111000
        z = np.array([x, y])

        self.ekf.update(z, "gnss")
        self.publish_state()

    def publish_state(self):
        state = self.ekf.get_state()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(state[0])
        msg.pose.position.y = float(state[1])
        msg.pose.orientation.z = float(np.sin(state[2]/2))
        msg.pose.orientation.w = float(np.cos(state[2]/2))
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()