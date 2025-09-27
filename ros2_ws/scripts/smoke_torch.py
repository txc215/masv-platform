# ros2_ws/scripts/smoke_torch.py
import torch, sys

m = sys.argv[1] if len(sys.argv) > 1 else "/root/ai_models/gru_model_091025.pt"

model = torch.jit.load(m, map_location="cpu").eval()

print("TorchScript OK:", m)