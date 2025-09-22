# src/ros2_interface/test/test_gru_launch.py
import os
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu

from launch import LaunchDescription
from launch_ros.actions import Node as RosNode

import pytest
import launch_testing
from launch_testing.markers import keep_alive


@pytest.mark.launch_test
def generate_test_description():

    sub_topic = os.getenv("SUB_TOPIC", "/imu/data")
    pub_topic = os.getenv("PUB_TOPIC", "/gru/pred")
    model_path = os.getenv("GRU_MODEL", "/root/ai_models/gru_model.onnx")
    seq_len = int(os.getenv("SEQ_LEN", "5"))  # CI 測試用短序列

    # ROS2 node
    gru = RosNode(
        package="ros2_interface",
        executable="gru_infer_node",
        name="gru_infer_node",
        output="screen",
        parameters=[{
            "model_path": model_path,
            "input_size": 6,
            "seq_len": seq_len,
            "sub_topic": sub_topic,
            "pub_topic": pub_topic,
        }],
    )


    return LaunchDescription([gru]), {"topics": {"sub": sub_topic, "pub": pub_topic}}


@keep_alive
def test_topic_emits_after_imu_input(topics):
    
    rclpy.init()
    node = Node("tester")

    received = []

    def on_pred(msg):
        received.append(msg)

    pub_topic = topics["sub"]
    sub_topic = topics["pub"]

    node.create_subscription(Float32MultiArray, sub_topic, on_pred, 10)
    pub = node.create_publisher(Imu, pub_topic, 10)

    imu = Imu()
    imu.linear_acceleration.x = 0.1
    imu.linear_acceleration.y = 0.2
    imu.linear_acceleration.z = 9.8
    imu.angular_velocity.x = 0.01
    imu.angular_velocity.y = 0.02
    imu.angular_velocity.z = 0.03

    deadline = time.time() + 5.0
    while time.time() < deadline and not received:
        pub.publish(imu)
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    assert received, "GRU node did not publish to pub_topic within 5s"
