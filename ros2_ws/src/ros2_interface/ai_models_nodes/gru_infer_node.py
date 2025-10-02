#!/usr/bin/env python3
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from collections import deque

class OnnxRunner:
    def __init__(self, path: str, input_size: int):
        import onnxruntime as ort
        self.sess = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        self.input_name = self.sess.get_inputs()[0].name
        self.input_size = input_size

    def infer(self, x_1_L_F: np.ndarray) -> np.ndarray:
        y = self.sess.run(None, {self.input_name: x_1_L_F})[0]  # (1, L, D) or (1, D)
        if y.ndim == 3:
            y = y[:, -1, :]  # get last time step
        return np.squeeze(y).astype(np.float32)

class GRUInferNode(Node):
    def __init__(self):
        super().__init__("gru_infer_node")

        self.declare_parameter("model_path", "/root/ai_models/gru_model_091025.onnx")
        self.declare_parameter("seq_len", 20)
        self.declare_parameter("input_size", 6)
        self.declare_parameter("sub_topic", "/imu/data")
        self.declare_parameter("pub_topic", "/gru/pred")

        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.seq_len = self.get_parameter("seq_len").get_parameter_value().integer_value
        self.input_size = self.get_parameter("input_size").get_parameter_value().integer_value
        sub_topic = self.get_parameter("sub_topic").get_parameter_value().string_value
        pub_topic = self.get_parameter("pub_topic").get_parameter_value().string_value

        if not os.path.exists(model_path):
            raise FileNotFoundError(model_path)

        self.runner = OnnxRunner(model_path, self.input_size)
        self.buf = deque(maxlen=self.seq_len)

        qos = rclpy.qos.QoSProfile(depth=50)
        self.sub = self.create_subscription(Imu, sub_topic, self.on_imu, qos)
        self.pub = self.create_publisher(Float32MultiArray, pub_topic, qos)
        self.timer = self.create_timer(0.01, self.tick)  # 100Hz infer

        self.get_logger().info(f"GRU ONNX loaded: {model_path}")

    def on_imu(self, msg: Imu):
        f = np.array([
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ], dtype=np.float32)
        if f.shape[0] == self.input_size:
            self.buf.append(f)

    def tick(self):
        if len(self.buf) < self.seq_len:
            return
        x = np.stack(self.buf, axis=0)[None, :, :]  # (1, L, F)
        try:
            y = self.runner.infer(x)  # (D,)
            msg = Float32MultiArray(data=y.tolist())
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"infer error: {e}")

def main():
    rclpy.init()
    node = GRUInferNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
