# inference_gru.py

import os
import torch
import numpy as np
from train_gru import GRUNet
import sys
sys.path.insert(0, '../')
from config import *

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# ======== input test data ========
X_test = np.load(os.path.join(DATA_PATH, "X_train.npy"))  # shape: (N, T, 6)
X_tensor = torch.tensor(X_test, dtype=torch.float32).to(DEVICE)

# ======== init GRU model & input weight ========
model = GRUNet(input_size=INPUT_SIZE, hidden_size=HIDDEN_SIZE, output_size=3)  # output_size is heading + vx + vy
model.load_state_dict(torch.load(os.path.join(MODEL_PATH, FILE_NAME), weights_only=True))
model.to(DEVICE)
model.eval()

# ======== AI model inference ========
with torch.no_grad():
    Y_pred = model(X_tensor)  # shape: (N, 3)
    Y_pred = Y_pred.cpu().numpy()

# ======== save results ========
np.save(os.path.join(DATA_PATH, "Y_pred.npy"), Y_pred)
print(f"out shape: {Y_pred.shape}")
