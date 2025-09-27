import numpy as np
import pandas as pd
import sys
sys.path.insert(0, '../../')
from motion_models.vehicle.bicycle_model import BicycleModel
from BaseEKF import EKFAlgorithm

# 1. Simulation data
dt = 0.1
steps = 120
true_states = []
measurements = []

x_true = np.array([0.0, 0.0, 0.0]) # x, y theta
v = 1.0  # m/s
delta = np.radians(10)  # constant steering
bicycle = BicycleModel(3.0)

for i in range(steps):
    x_true = bicycle.predict(x_true, (v, delta), dt)
    noise_gnss = x_true[:2] + np.random.normal(0, 0.5, 1)  # GNSS x,y
    noise_heading = x_true[2] + np.random.normal(0, 0.03)  # magnetometer
    true_states.append(x_true.copy())
    measurements.append({'gnss': noise_gnss, 'mag': np.array([noise_heading])})

# --- 2. init EKF ---
x0 = np.array([0.0, 0.0, 0.0])
P0 = np.eye(3) * 0.5
ekf = EKFAlgorithm(x0, P0)
ekf.set_observation_noise("gnss", np.eye(2) * 0.25)
ekf.set_observation_noise("magnetometer", np.array([[0.01]]))

# --- 3. process EKF Filter ---
estimates = {"timestamp_sec": [], "ENU_x_m": [], "ENU_y_m": [], "theta_heading_rad": []}
for i in range(steps):
    u = (v, delta)
    ekf.predict(u, dt, model="bicycle")  # try model="imu" to compare

    # update mag magnetometer every step
    # update GNSS every 2 step
    ekf.update(measurements[i]["mag"], "magnetometer")
    if i % 2 == 0:
        ekf.update(measurements[i]["gnss"], "gnss")

    x_m, y_m, theta_rad = ekf.get_state()
    estimates["timestamp_sec"].append(i*0.1)
    estimates["ENU_x_m"].append(x_m)
    estimates["ENU_y_m"].append(y_m)
    estimates["theta_heading_rad"].append(theta_rad)

# # --- 4. check result ---
output_df = pd.DataFrame(estimates)
output_df.to_csv("../../data/sample_outputs/dummy_data.csv", sep='\t', encoding='utf-8', index=False, header=True)
