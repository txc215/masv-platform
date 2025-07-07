import torch
import torch.nn as nn
from algorithms.base_algorithm import BaseAlgorithm

class PositionalEncoding(nn.Module):
    def __init__(self, d_model, max_len=500):
        super().__init__()
        pe = torch.zeros(max_len, d_model)
        pos = torch.arange(0, max_len).unsqueeze(1).float()
        div = torch.exp(torch.arange(0, d_model, 2).float() * (-torch.log(torch.tensor(10000.0)) / d_model))
        pe[:, 0::2] = torch.sin(pos * div)
        pe[:, 1::2] = torch.cos(pos * div)
        self.pe = pe.unsqueeze(0)

    def forward(self, x):
        return x + self.pe[:, :x.size(1)].to(x.device)

class TransformerFusionModel(nn.Module):
    def __init__(self, input_dim=12, output_dim=3, d_model=64, num_heads=4, num_layers=2):
        super().__init__()
        self.embedding = nn.Linear(input_dim, d_model)
        self.pos_encoder = PositionalEncoding(d_model)
        encoder_layer = nn.TransformerEncoderLayer(d_model=d_model, nhead=num_heads)
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)
        self.head = nn.Linear(d_model, output_dim)

    def forward(self, x):
        x = self.embedding(x)
        x = self.pos_encoder(x)
        x = self.transformer(x.transpose(0, 1)).transpose(0, 1)
        return self.head(x[:, -1, :])

class TransformerFusion(BaseAlgorithm):
    def __init__(self, model_path=None):
        super().__init__()
        self.model = TransformerFusionModel()
        self.model.eval()

        if model_path:
            self.model.load_state_dict(torch.load(model_path, map_location="cpu"))

        self.sequence = []  # Keep IMU + GNSS history sequence
        self.seq_len = 20   # squence length limit

    def initialize(self, initial_state):
        super().initialize(initial_state)
        self.sequence = []

    def update(self, sensor_data):
        """
        Args:
            sensor_data: dict with keys 'imu' and 'gnss' (1 data)
        Returns:
            dict: {'position': [x, y, z]}
        """
        imu_d = sensor_data["imu"]
        gnss_d = sensor_data["gnss"]

        imu_vals = [imu_d[k] for k in ["acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"]]
        gnss_vals = [gnss_d[k] for k in ["lat", "lon", "alt", "vel_n", "vel_e", "vel_d"]]
        combined = imu_vals + gnss_vals
        self.sequence.append(combined)

        if len(self.sequence) < self.seq_len:
            return {"position": [0.0, 0.0, 0.0]}  # if no enough data set

        # 保持固定長度
        self.sequence = self.sequence[-self.seq_len:]

        x = torch.tensor([self.sequence], dtype=torch.float32)  # [1, seq_len, 12]
        with torch.no_grad():
            out = self.model(x).squeeze(0)

        return {"position": out.tolist()}
