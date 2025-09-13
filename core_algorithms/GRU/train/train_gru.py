import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import os
import matplotlib.pyplot as plt

# ======== Model ========
class GRUNet(nn.Module):
    def __init__(self, input_size=6, hidden_size=64, output_size=3, num_layers=1):
        super(GRUNet, self).__init__()
        self.gru = nn.GRU(input_size, hidden_size, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        _, h = self.gru(x)
        out = self.fc(h[-1])
        return out


def train_gru_model():
    # ======== Config ========
    DATA_DIR = "../../../data/rosbag_logs/processed_dataset/utbm_robocar_20190110"
    BATCH_SIZE = 64
    EPOCHS = 50
    LR = 1e-3
    HIDDEN_SIZE = 64
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    MODEL_PATH = "../models/gru_model.pt"
    # ======== Load Data ========
    X = np.load(os.path.join(DATA_DIR, "X_train.npy"))
    Y = np.load(os.path.join(DATA_DIR, "Y_train.npy"))
    print(f"Loaded X: {X.shape}, Y: {Y.shape}")

    X_tensor = torch.tensor(X, dtype=torch.float32)
    Y_tensor = torch.tensor(Y, dtype=torch.float32)

    dataset = TensorDataset(X_tensor, Y_tensor)
    train_loader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)

    # ======== Initialize ========
    model = GRUNet(hidden_size=HIDDEN_SIZE).to(DEVICE)
    optimizer = optim.Adam(model.parameters(), lr=LR)
    lost_func = nn.MSELoss()

    # ======== Training Loop ========

    losses = []
    for epoch in range(EPOCHS):
        epoch_loss = 0.0
        model.train()
        for X_batch, Y_batch in train_loader:
            X_batch = X_batch.to(DEVICE)
            Y_batch = Y_batch.to(DEVICE)

            optimizer.zero_grad()
            output = model(X_batch)
            loss = lost_func(output, Y_batch)
            loss.backward()
            optimizer.step()
            epoch_loss += loss.item() * X_batch.size(0)

        epoch_loss /= len(train_loader.dataset)
        losses.append(epoch_loss)
        print(f"Epoch {epoch+1}/{EPOCHS}, Loss: {epoch_loss:.6f}")

    # ======== Save Model ========
    torch.save(model.state_dict(), MODEL_PATH)
    print(f">> Saved model to {MODEL_PATH}")

    # ======== Plot Loss ========
    plt.plot(losses)
    plt.xlabel("Epoch")
    plt.ylabel("Loss (MSE)")
    plt.title("Training Loss")
    plt.grid()
    plt.savefig("../models/training_loss.png")
    print("[*] Saved training loss plot.")

if __name__ == "__main__":
    train_gru_model()
