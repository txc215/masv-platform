import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
import csv
import time
from rclpy.clock import ClockType
from rclpy.qos import QoSProfile
import pandas as pd

class LogReplayNode(Node):
    def __init__(self):
        super().__init__('log_replay_node')
        self.declare_parameter("log_path", "/root/data/sample_inputs/dummy_gnss_imu.csv")

        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("gnss_frequency", 5.0)
        self.declare_parameter("imu_frequency", 50.0)
        self.declare_parameter("gnss_position_covariance", [0.0]*9)
        self.declare_parameter("gnss_covariance_type", 0)

        self.log_path = self.get_parameter("log_path").value
        self.frame_id = self.get_parameter("frame_id").value
        self.gnss_freq = self.get_parameter("gnss_frequency").value
        self.imu_freq = self.get_parameter("imu_frequency").value
        self.gnss_cov = self.get_parameter("gnss_position_covariance").value
        self.gnss_cov_type = self.get_parameter("gnss_covariance_type").value

        self.imu_pub = self.create_publisher(Imu, '/imu/data', QoSProfile(depth=10))
        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', QoSProfile(depth=10))

        self.load_csv()
        self.index_gnss = 0
        self.index_imu = 0

        # Timers
        self.create_timer(1.0 / self.imu_freq, self.publish_imu)
        self.create_timer(1.0 / self.gnss_freq, self.publish_gnss)

    def load_csv(self):
        self.gnss_data = pd.read_csv(self.log_path, usecols=["timestamp_sec", "gnss_lat_deg", "gnss_lon_deg", "gnss_alt_m"])
        self.imu_data  = pd.read_csv(self.log_path, usecols=["timestamp_sec", "accel_x_mps2", "accel_y_mps2", "accel_z_mps2", \
                                                                              "gyro_x_radps", "gyro_y_radps", "gyro_z_radps", \
                                                                              "mag_x_uT", "mag_y_uT", "mag_z_uT"]) 
        self.get_logger().info(f"Loaded {len(self.gnss_data)} GNSS and IMU entries.")

    def publish_imu(self):
        if self.index_imu >= len(self.imu_data):
            return

        row = self.imu_data.iloc[self.index_imu]
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        imu_msg.angular_velocity.z = float(row['gyro_z_radps'])
        imu_msg.linear_acceleration.x = float(row['accel_x_mps2'])
        imu_msg.linear_acceleration.y = float(row['accel_y_mps2'])

        self.imu_pub.publish(imu_msg)
        self.index_imu += 1

    def publish_gnss(self):
        if self.index_gnss >= len(self.gnss_data):
            return

        row = self.gnss_data.iloc[self.index_gnss]
        gnss_msg = NavSatFix()
        gnss_msg.header.stamp = self.get_clock().now().to_msg()
        gnss_msg.header.frame_id = self.frame_id

        gnss_msg.latitude = float(row['gnss_lat_deg'])
        gnss_msg.longitude = float(row['gnss_lon_deg'])
        gnss_msg.altitude = float(row.get('gnss_alt_m', 0.0))

        gnss_msg.position_covariance = self.gnss_cov
        gnss_msg.position_covariance_type = self.gnss_cov_type

        self.gnss_pub.publish(gnss_msg)
        self.index_gnss += 1

def main(args=None):
    rclpy.init(args=args)
    node = LogReplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
