import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
import pandas as pd

class LogReplayNode(Node):
    def __init__(self):
        super().__init__('log_replay_node')

        # ----- Parameters -----
        self.declare_parameter("log_path", "/root/data/sample_inputs/dummy_gnss_imu.csv")
        self.declare_parameter("frame_id", "base_link")
        # self.declare_parameter("use_sim_time", True)
        self.declare_parameter("gnss_frequency", 5.0)
        self.declare_parameter("imu_frequency", 50.0)
        self.declare_parameter("mag_frequency", 10.0)
        # GNSS covariance setup
        self.declare_parameter("gnss_position_covariance", [0.0]*9)
        self.declare_parameter("gnss_covariance_type", 0)

        p = self.get_parameter
        self.log_path = p("log_path").value
        self.frame_id = p("frame_id").value
        self.gnss_freq = float(p("gnss_frequency").value)
        self.imu_freq  = float(p("imu_frequency").value)
        self.mag_freq  = float(p("mag_frequency").value)
        self.gnss_cov  = list(p("gnss_position_covariance").value)
        self.gnss_cov_type = int(p("gnss_covariance_type").value)

        # ----- QoS: SensorDataQoS -----
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.imu_pub  = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self.gnss_pub = self.create_publisher(NavSatFix, '/gnss/fix', sensor_qos)
        self.mag_pub  = self.create_publisher(MagneticField, '/mag/data', sensor_qos)

        # ----- Load CSV once -----
        df = pd.read_csv(self.log_path)

        # add column data if not exist
        def col(name, default=0.0):
            return df[name] if name in df.columns else pd.Series([default]*len(df))

        self.ts = col("timestamp_sec")
        # IMU
        self.ax = col("accel_x_mps2"); self.ay = col("accel_y_mps2"); self.az = col("accel_z_mps2")
        self.gx = col("gyro_x_radps"); self.gy = col("gyro_y_radps"); self.gz = col("gyro_z_radps")
        # MAG convert uT to T
        self.mx = col("mag_x_uT") * 1e-6
        self.my = col("mag_y_uT") * 1e-6
        self.mz = col("mag_z_uT") * 1e-6
        # GNSS
        self.lat = col("gnss_lat_deg")
        self.lon = col("gnss_lon_deg")
        self.alt = col("gnss_alt_m", 0.0)

        n = len(self.ts)
        self.get_logger().info(f"Loaded {n} rows from {self.log_path}")

        # index
        self.i_imu = 0; self.i_mag = 0; self.i_gnss = 0; self.N = n

        # Timers stamp msg time still based on CSV timestamp
        if self.imu_freq > 0:
            self.create_timer(1.0 / self.imu_freq, self.publish_imu)
        if self.mag_freq > 0:
            self.create_timer(1.0 / self.mag_freq, self.publish_mag)
        if self.gnss_freq > 0:
            self.create_timer(1.0 / self.gnss_freq, self.publish_gnss)

    # use float sec to builtin_interfaces/Time
    def _stamp_from_sec(self, tsec: float) -> RosTime:
        t = RosTime()
        t.sec = int(tsec)
        t.nanosec = int((tsec - t.sec) * 1e9)
        return t

    def publish_imu(self):
        if self.i_imu >= self.N:
            return
        i = self.i_imu
        msg = Imu()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self._stamp_from_sec(float(self.ts.iloc[i]))

        # if no orientation, covariance[0] = -1
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = float(self.gx.iloc[i])
        msg.angular_velocity.y = float(self.gy.iloc[i])
        msg.angular_velocity.z = float(self.gz.iloc[i])
        msg.linear_acceleration.x = float(self.ax.iloc[i])
        msg.linear_acceleration.y = float(self.ay.iloc[i])
        msg.linear_acceleration.z = float(self.az.iloc[i])

        self.imu_pub.publish(msg)
        self.i_imu += 1

    def publish_mag(self):
        if self.i_mag >= self.N:
            return
        i = self.i_mag
        msg = MagneticField()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self._stamp_from_sec(float(self.ts.iloc[i]))

        msg.magnetic_field.x = float(self.mx.iloc[i])
        msg.magnetic_field.y = float(self.my.iloc[i])
        msg.magnetic_field.z = float(self.mz.iloc[i])

        self.mag_pub.publish(msg)
        self.i_mag += 1

    def publish_gnss(self):
        if self.i_gnss >= self.N:
            return
        i = self.i_gnss
        msg = NavSatFix()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self._stamp_from_sec(float(self.ts.iloc[i]))

        msg.latitude  = float(self.lat.iloc[i])
        msg.longitude = float(self.lon.iloc[i])
        msg.altitude  = float(self.alt.iloc[i])

        # covariance（row-major 3x3）
        if len(self.gnss_cov) == 9:
            msg.position_covariance = self.gnss_cov
        msg.position_covariance_type = self.gnss_cov_type

        self.gnss_pub.publish(msg)
        self.i_gnss += 1

def main(args=None):
    rclpy.init(args=args)
    node = LogReplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
