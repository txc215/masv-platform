# plusgins/algos/gru_node.py

import collections
from typing import Optional, Dict, Any
import numpy as np
import onnxruntime as ort
from plugins.base_node import BaseNode

class GRUNode(BaseNode):
    """
    6 dimensions IMU（ax, ay, az, gx, gy, gz). Sliding window for ONNX inference
    """
    def __init__(self, onnx_path: str,
                 seq_len: int = 20, input_size: int = 6,
                 input_name: str = "", output_name: str = "",
                 map_imu = {"ax":"ax","ay":"ay","az":"az","gx":"gx","gy":"gy","gz":"gz"},
                 norm_mean: list = None, norm_std: list = None):
        self.sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.in_name  = input_name  or self.sess.get_inputs()[0].name
        self.out_name = output_name or self.sess.get_outputs()[0].name
        self.seq_len = seq_len
        self.input_size = input_size
        self.map = map_imu
        self.buf = collections.deque(maxlen=seq_len)
        self.mean = np.array(norm_mean, dtype=np.float32) if norm_mean else None
        self.std  = np.array(norm_std,  dtype=np.float32) if norm_std  else None

    def reset(self): 
        self.buf.clear()

    def on_event(self, ev: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if ev["_topic"] != "imu":
            return None
        feat = np.array([
            float(ev.get(self.map["ax"],0.0)),
            float(ev.get(self.map["ay"],0.0)),
            float(ev.get(self.map["az"],0.0)),
            float(ev.get(self.map["gx"],0.0)),
            float(ev.get(self.map["gy"],0.0)),
            float(ev.get(self.map["gz"],0.0)),
        ], dtype=np.float32)
        if self.mean is not None and self.std is not None:
            feat = (feat - self.mean) / (self.std + 1e-8)
        self.buf.append(feat)
        if len(self.buf) < self.seq_len:
            return None
        x = np.stack(list(self.buf), axis=0)[None, :, :]  # (1, T, 6)
        y = self.sess.run([self.out_name], {self.in_name: x})[0].astype(float).ravel()
        # Assume output [x, y, yaw]
        return {"timestamp": float(ev["timestamp"]),
                "x": float(y[0]) if len(y)>0 else 0.0,
                "y": float(y[1]) if len(y)>1 else 0.0,
                "yaw": float(y[2]) if len(y)>2 else 0.0}
