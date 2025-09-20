# ros2_ws/scripts/smoke_onnx.py
import sys
import onnxruntime as ort

m = sys.argv[1] if len(sys.argv) > 1 else "/root/ai_models/gru_model_091025.onnx"

# try to load model
sess = ort.InferenceSession(m, providers=["CPUExecutionProvider"])

print("ONNX OK:", m)