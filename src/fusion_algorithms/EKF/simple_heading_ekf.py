import numpy as np
import pandas as pd
import os

curr_path = os.path.dirname(os.path.abspath(__file__))

__simple_data__ = curr_path + "/../../../data/sample_inputs/dummy_gnss_imu.csv"
__output_data__ = curr_path + "/../../../data/sample_outputs/dummy_gnss_imu_ekf.csv"


class SimpleHeadingEKF:
    def __init__(self):
        self.heading = 0.0  # init heading (rad)
        self.bias = 0.0     # Gyro bias
        self.P = np.eye(2)  # covariance matrix

        # Noise parameter
        self.Q = np.diag([0.01, 0.001])  # Process noise: heading, bias
        self.R = 0.05                    # Measurement noise (magnetometer or GNSS heading)

    def predict(self, gyro_z, dt):
        """Predict：use gyro data udpate heading"""
        u = gyro_z - self.bias
        self.heading += u * dt
        self.heading = self._normalize_angle(self.heading)

        # 狀態轉移矩陣 F, 控制矩陣 B
        F = np.array([[1.0, -dt],
                      [0.0, 1.0]])
        self.P = F @ self.P @ F.T + self.Q

    def update(self, heading_meas):
        """Update steps：Use magnetometer or GNSS heading adjust"""
        H = np.array([[1.0, 0.0]])
        y = self._normalize_angle(heading_meas - self.heading)  # Innovation
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T / S

        update = K.flatten() * y
        self.heading += update[0]
        self.bias   += update[1]
        self.heading = self._normalize_angle(self.heading)

        I = np.eye(2)
        self.P = (I - K @ H) @ self.P

    def _normalize_angle(self, angle):
        """Limit angle between -π to π"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def mag_to_heading(self, mag_x_uT, mag_y_uT):

        heading_rad = np.arctan2(-mag_y_uT, mag_x_uT)
        
        if heading_rad < 0:
            heading_rad += 2 * np.pi
        
        return heading_rad



if __name__ == '__main__':

    imu_df = pd.read_csv(__simple_data__, usecols=["timestamp_sec","gyro_z_radps", "mag_x_uT", "mag_y_uT"])

    ekf = SimpleHeadingEKF()

    data_len = len(imu_df["timestamp_sec"])
    output_data = {"timestamp_sec": [], "gyro_z_radps": [], "mag_heading_rad": []}
    update_ctrl = 10
    dt = 0.10101

    for t in range(data_len):
        output_data["timestamp_sec"].append(imu_df["timestamp_sec"][t])

        gyro_z = imu_df["gyro_z_radps"][t]  # simulate angular velocity（rad/s）

        ekf.predict(gyro_z, dt)

        output_data["gyro_z_radps"].append(gyro_z)

        if t % update_ctrl == 0:
            # every "update_ctrl" step update magnetometer heading (maybe noisy)
            # mag_heading = ekf.heading
            mag_heading = ekf.mag_to_heading(imu_df["mag_x_uT"][t], imu_df["mag_y_uT"][t])
            mag_heading += np.random.randn() * 0.1

            ekf.update(mag_heading)
        
        output_data["mag_heading_rad"].append(mag_heading)

        if t+1 < data_len:
            dt = imu_df["timestamp_sec"][t+1] - imu_df["timestamp_sec"][t]

        print(f"t={t*dt:.1f}s | Heading={np.degrees(ekf.heading):.2f} deg | Bias={ekf.bias:.4f}")

    output_df = pd.DataFrame(output_data)
    output_df.to_csv(__output_data__, sep='\t', encoding='utf-8', index=False, header=True)


