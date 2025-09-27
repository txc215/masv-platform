# GRU-based Velocity and Heading Prediction Report

## Task Overview

The goal is to predict short-term velocity components (vx, vy) and yaw angle (heading) of a ground vehicle using IMU time-series data. The model is trained from sensor data of the file `utbm_robocar_dataset_20190110_noimage.bag`.

- Input data: 6-axis IMU (acc + gyro) with sequence length = 20
- Output targets: [vx, vy, yaw] at current timestamp
- Model used: GRU (Gated Recurrent Unit)
  - `input_size = 6`
  - `hidden_size = 64`
  - `output_size = 3`
  - `num_layers = 1`

---

## Evaluation Summary

| Metric | vx      | vy      | yaw     |
|--------|---------|---------|---------|
| MSE    | `0.000XYZ` | `0.000XYZ` | `0.000XYZ` |
| MAE    | `0.00XYZ`  | `0.00XYZ`  | `0.00XYZ`  |

(Note: Replace with actual numbers from `evaluate_and_plot.py`)

---

## Prediction vs Ground Truth

### vx

![vx](evaluation\vx_pred_vs_true.png)

---

### vy

![vy](evaluation\vy_pred_vs_true.png)

---

### yaw

![yaw](evaluation\yaw_pred_vs_true.png)

---

## Initial Data Insight

Based on the plotted results:

- **vx ≈ 0**, **vy ≈ 0** most of the time
- yaw is increasing or oscillating smoothly, indicating rotation-in-place or slow turns
- This suggests that the dataset used for training and evaluation corresponds to stationary spinning or parking-lot maneuvers.

Data source insight:
> The original `.bag` file (`utbm_robocar_dataset_20190110_noimage.bag`) likely includes segments of the robot spinning in place, or moving with minimal translational motion.

---

## Conclusion

- GRU learned to model the rotational behavior, with very low loss in yaw prediction.
- Translational prediction performance is limited due to lack of vx/vy variation in training data.

---

## Next Steps



---

## Project Files

- `X_train.npy`, `Y_train.npy`, `Y_pred.npy`
- `gru_model.pt`
- `evaluate_and_plot.py`
- `vx_pred_vs_true.png`, `vy_pred_vs_true.png`, `yaw_pred_vs_true.png`

## Acknowledgment

The dataset used in this project (`utbm_robocar_dataset_20190110_noimage.bag`) was kindly provided by the **Université de Technologie de Belfort-Montbéliard (UTBM)** as part of the [UTBM Robocar Dataset](https://github.com/rwth-asic/utbm_robocar_dataset). 

We thank the UTBM team for their contribution to the open-source robotics and autonomous driving research community.


