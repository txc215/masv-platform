import pandas as pd
import numpy as np
import os

# setup data path
imu_path = "../data/rosbag_logs/utbm_robocar_dataset_20190110_noimage/imu_data.csv"
vel_path = "../data/rosbag_logs/utbm_robocar_dataset_20190110_noimage/velocity.csv"
output_dir = "../data/rosbag_logs/processed_dataset/utbm_robocar_20190110"
window_size = 20  # every window size len X 

# read .csv file
imu_df = pd.read_csv(imu_path)
vel_df = pd.read_csv(vel_path)


imu_df['timestamp'] = imu_df['header.stamp.secs'] + imu_df['header.stamp.nsecs'] * 1e-9

# get head of columns and give new name (base rostopic echo output columns)
imu_df = imu_df.rename(columns={
    'angular_velocity.x': 'gx_rps',
    'angular_velocity.y': 'gy_rps',
    'angular_velocity.z': 'gz_rps',
    'linear_acceleration.x': 'ax_mps2',
    'linear_acceleration.y': 'ay_mps2',
    'linear_acceleration.z': 'az_mps2',
})
imu_df = imu_df[['timestamp', 'ax_mps2', 'ay_mps2', 'az_mps2', 'gx_rps', 'gy_rps', 'gz_rps']].dropna()


vel_df['timestamp'] = vel_df['header.stamp.secs'] + vel_df['header.stamp.nsecs'] * 1e-9

vel_df = vel_df.rename(columns={
    'twist.linear.x': 'vx_mps',
    'twist.linear.y': 'vy_mps',
    'twist.angular.z': 'wz_mps',
})

vel_df = vel_df[['timestamp', 'vx_mps', 'vy_mps', 'wz_mps']].dropna()

# convert timesatmp into float（just in case not work）
imu_df['timestamp'] = imu_df['timestamp'].astype(float)
vel_df['timestamp'] = vel_df['timestamp'].astype(float)

# sync：according time to align IMU data and velocity data
# Use merge_asof（closest timestamp）with direction='nearest'
aligned_df = pd.merge_asof(imu_df.sort_values('timestamp'),
                           vel_df.sort_values('timestamp'),
                           on='timestamp',
                           direction='nearest',
                           tolerance=0.01)  # tolerance large time default 0.01s（may need adjust）

# remove un-aligned data
aligned_df = aligned_df.dropna()

# setup X, Y datasets（sliding window）
X_list = []
Y_list = []
for i in range(len(aligned_df) - window_size):
    X_seq = aligned_df.iloc[i:i+window_size][['ax_mps2', 'ay_mps2', 'az_mps2', 'gx_rps', 'gy_rps', 'gz_rps']].values
    Y = aligned_df.iloc[i+window_size][['vx_mps', 'vy_mps', 'wz_mps']].values
    X_list.append(X_seq)
    Y_list.append(Y)

X = np.array(X_list)
Y = np.array(Y_list)

print(f"- Finished: X shape = {X.shape}, Y shape = {Y.shape}")

# save as .npy file
os.makedirs(output_dir, exist_ok=True)
np.save(os.path.join(output_dir, "X_train.npy"), X)
np.save(os.path.join(output_dir, "Y_train.npy"), Y)

print(f"- Saved: {output_dir}/X_train.npy and Y_train.npy")
