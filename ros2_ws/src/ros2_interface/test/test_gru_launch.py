# ros2_interface/test/test_gru_launch.py
import os
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from launch import LaunchDescription
from launch_ros.actions import Node as RosNode
from launch_testing.markers import keep_alive
import time

MODEL = os.environ.get("GRU_MODEL", "/root/ai_models/gru_model_091025.onnx")

def generate_test_description():
    params = {
        "model_path": MODEL,
        "seq_len": 5,          # reduce waiting time
        "input_size": 6,
        "sub_topic": "/imu/data",
        "pub_topic": "/gru/pred",
    }
    gru = RosNode(
        package="ros2_interface",
        executable="gru_infer_node",
        name="gru_infer_node",
        output="screen",
        parameters=[params],
    )
    return LaunchDescription([gru]), {"params": params}

@keep_alive
def test_topic_emits_after_imu_input(params):
    rclpy.init()
    node = rclpy.create_node("tester")
    pred_msgs = []

    def on_pred(msg):
        pred_msgs.append(msg)

    sub = node.create_subscription(Float32MultiArray, params["pub_topic"], on_pred, 10)
    pub = node.create_publisher(Imu, params["sub_topic"], 10)

    # 發 50 筆 IMU（seq_len=5 所以很快會出結果）
    imu = Imu()
    imu.linear_acceleration.x = 0.1
    imu.linear_acceleration.y = 0.2
    imu.linear_acceleration.z = 9.8
    imu.angular_velocity.x = 0.01
    imu.angular_velocity.y = 0.02
    imu.angular_velocity.z = 0.03

    start = time.time()
    while time.time() - start < 5.0:  # 最多等 5 秒
        pub.publish(imu)
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred_msgs:
            break

    node.destroy_node()
    rclpy.shutdown()
    assert pred_msgs, "GRU node did not publish /gru/pred within timeout"
