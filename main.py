# main.py
import pandas as pd
from simulator_core.algorithms.ekf import run_ekf

if __name__ == "__main__":
    data_path = "data/sample_inputs/dummy_gnss_imu.csv"
    output_path = "data/sample_results/ekf_output.csv"

    df = pd.read_csv(data_path)
    result_df = run_ekf(df)
    result_df.to_csv(output_path, index=False)
    print(f"EKF completed. Results saved to {output_path}")
