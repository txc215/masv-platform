#!/usr/bin/env python3
import os, time, collections, numpy as np, pandas as pd, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

class AnalysisNode(Node):
    def __init__(self):
        super().__init__('analysis_node')
        self.declare_parameter('sub_topic', '/imu/data')
        self.declare_parameter('out_dir', '/root/data/analysis')
        self.declare_parameter('window_sec', 10.0)

        self.topic = self.get_parameter('sub_topic').get_parameter_value().string_value
        self.out_dir = self.get_parameter('out_dir').get_parameter_value().string_value
        self.window_sec = self.get_parameter('window_sec').get_parameter_value().double_value
        os.makedirs(self.out_dir, exist_ok=True)

        self.buf = collections.deque()  # (t, ax, ay, az)
        self.t0 = None

        self.sub = self.create_subscription(Imu, self.topic, self.callback, 10)
        self.create_timer(2.0, self.dump_stats_and_plot)

        self.get_logger().info(f'Analyzing {self.topic}, out={self.out_dir}')

    def callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        t -= self.t0
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.buf.append((t, ax, ay, az))
        # sliding window
        while self.buf and (t - self.buf[0][0]) > self.window_sec:
            self.buf.popleft()

    def dump_stats_and_plot(self):
        if len(self.buf) < 10:
            return
        arr = np.array(self.buf)
        t, ax, ay, az = arr[:,0], arr[:,1], arr[:,2], arr[:,3]
        df = pd.DataFrame({'t':t,'ax':ax,'ay':ay,'az':az})
        stats = df[['ax','ay','az']].agg(['mean','std','min','max'])
        now = time.strftime('%Y%m%d_%H%M%S')

        # save data into CSV of the last window_sec
        csv_path = os.path.join(self.out_dir, f'imu_window_{now}.csv')
        df.to_csv(csv_path, index=False)

        # to PNG file
        png_path = os.path.join(self.out_dir, f'imu_window_{now}.png')
        plt.figure()
        plt.plot(t, ax, label='ax')
        plt.plot(t, ay, label='ay')
        plt.plot(t, az, label='az')
        plt.xlabel('t (s)'); plt.ylabel('linear_acc (m/s^2)'); plt.legend(); plt.tight_layout()
        plt.savefig(png_path); plt.close()

        # log statistic
        m = stats.loc['mean'].to_dict(); s = stats.loc['std'].to_dict()
        self.get_logger().info(f"window {self.window_sec}s stats: mean={m} std={s} -> saved {os.path.basename(csv_path)}, {os.path.basename(png_path)}")

def main():
    rclpy.init()
    node = AnalysisNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
