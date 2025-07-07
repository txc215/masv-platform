import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from geometry_msgs.msg import Pose2D, PoseStamped
from UKF.ukf import UKFAlgorithm
import numpy as np



class UKFNode(Node):
        
    def __init__(self):

        super().__init__('ukf_node')

        # Load parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('dt_max', 0.1),
                ('dt_min', 1e-3),
                ('Q_diag', [1e-4, 1e-4]),
                ('R_mag', 0.05),
                ('R_gnss', 0.5),
                ('P0_diag', [0.1, 0.1]),
            ])

        self.dt_max = self.get_parameter('dt_max').value
        self.dt_min = self.get_parameter('dt_min').value
        
        Q_diag = self.get_parameter('Q_diag').value
        R_mag = self.get_parameter('R_mag').value
        R_gnss = self.get_parameter('R_gnss').value
        P0_diag = self.get_parameter('P0_diag').value

        # Create filter
        self.ukf = UKFAlgorithm(
            f=self.f,
            h_dict={"mag": self.h_mag, "gnss": self.h_gnss},
            Q=np.diag(Q_diag),
            R_dict={"mag": np.array([[R_mag]]), "gnss": np.array([[R_gnss]])}
        )

        # Subscribe to sensors
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(MagneticField, '/mag/data', self.mag_callback, 10)
        self.create_subscription(NavSatFix, '/gnss/fix', self.gnss_callback, 10)

        # Publisher
        self.state_pub = self.create_publisher(Pose2D, '/ukf/state', 10)

    def imu_callback(self, msg: Imu):
        # gyro.z as heading rate
        if not self.ukf.initialized:
            init_state = np.array([0.0, 0.0])  # [heading, heading_rate]
            P0 = np.diag([0.1, 0.1])
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.ukf.initialize(init_state, P0, timestamp)
        else:
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.ukf.state[1] = msg.angular_velocity.z  # set rate
            self.ukf.predict(timestamp)

    def mag_callback(self, msg: MagneticField):
        
        if self.ukf.initialized:
            mx_uT = msg.magnetic_field.x
            my_uT = msg.magnetic_field.y
            heading = np.arctan2(mx_uT, my_uT)
            self.ukf.update("mag", np.array([heading]))
            self.publish_state()


    def gnss_callback(self, msg: NavSatFix):
        if self.ukf.initialized:
            # Assume you have one function can transfer NavSatFix to heading
            heading = self.extract_heading_from_navsat(msg)
            self.ukf.update("gnss", np.array([heading]))
            self.publish_state()

    def extract_heading_from_navsat(self, msg: NavSatFix):
        # Difference with last GNSS position，calculate heading
        if not hasattr(self, 'last_gnss'):
            self.last_gnss = (msg.latitude, msg.longitude)
            return 0.0  # Can not calculate for just one data
        else:
            lat1, lon1 = self.last_gnss
            lat2, lon2 = msg.latitude, msg.longitude
            self.last_gnss = (lat2, lon2)

            dy = lat2 - lat1
            dx = lon2 - lon1
            heading_rad = np.arctan2(dx, dy)  # North= 0
            return heading_rad

    def publish_state(self):
        state, P_v = self.ukf.get_state()
        msg = Pose2D()
        msg.x = 0.0
        msg.y = 0.0
        msg.theta = float(state[0])
        self.state_pub.publish(msg)


    def f(self, x, dt):
        dt = max(min(dt, self.dt_max), self.dt_min)
        return np.array([x[0] + x[1]*dt, x[1]])

    def h_mag(self, x):
        # assume magnetometer give heading
        return np.array([x[0]])

    def h_gnss(self, x):
        # assum gnss provide position heading 
        return np.array([x[0]])

def main(args=None):
    rclpy.init(args=args)
    node = UKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()