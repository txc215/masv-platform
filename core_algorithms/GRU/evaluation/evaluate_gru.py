# evaluate_gru.py

import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error, mean_absolute_error

import sys
sys.path.insert(0, '../')

from config import *
import os


MODEL_MODEL_PATH = os.path.join(MODEL_PATH, FILE_NAME)

# === input data ===
X = np.load(f"{DATA_PATH}/{X_FILE}")
Y_true = np.load(f"{DATA_PATH}/{Y_FILE}")
Y_pred = np.load(f"{DATA_PATH}/{Y_PRED_FILE}")

print(f" Loaded X: {X.shape}, Y: {Y_true.shape}, Y_pred: {Y_pred.shape}")


# === label name ===
labels = ['vx', 'vy', 'yaw']

# === display data and mean squared err and mean abs err ===
print("===== Error Metrics =====")
for i, label in enumerate(labels):
    mse = mean_squared_error(Y_true[:, i], Y_pred[:, i])
    mae = mean_absolute_error(Y_true[:, i], Y_pred[:, i])
    print(f"{label}: MSE={mse:.6f}, MAE={mae:.6f}")

# === compare and plot ===
for i, label in enumerate(labels):
    plt.figure(figsize=(10, 3))
    plt.plot(Y_true[:, i], label=f"True {label}")
    plt.plot(Y_pred[:, i], label=f"Pred {label}")
    plt.title(f"{label.upper()} Prediction vs Ground Truth")
    plt.xlabel("Sample index")
    plt.ylabel(label)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f"{label}_pred_vs_true.png")  # 儲存圖檔
    plt.show()
