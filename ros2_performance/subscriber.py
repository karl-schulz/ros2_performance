#!/usr/bin/python3
from random import randint
from time import time

import psutil
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ros2_performance.util import Interval


def main(args=None):
    rclpy.init()
    node = rclpy.create_node(f"test_sub_{randint(0, 1000)}")
    num = node.declare_parameter("num", 1).value
    gap_time = node.declare_parameter("gap_time", 0.150).value
    proc = psutil.Process()

    t_last_log = time()
    t_last_msg = time()
    num_recv = 0
    max_gap = 0.0  # Not really latency: includes the publishing rate dt
    log_interval = Interval(2.0)

    def callback(msg: Image):
        nonlocal num_recv, max_gap, t_last_log, t_last_msg, log_interval
        num_recv += 1
        now = time()
        dt_msg = now - t_last_msg
        max_gap = max(max_gap, dt_msg)
        if dt_msg > gap_time:
            pass  # node.get_logger().warn(f"GAP DETECTED: {dt_msg:.3f}s, max is {gap_time:.3f}")
        if log_interval.tick():
            dt_interval = now - t_last_log
            node.get_logger().info(f"{num_recv / dt_interval:.2f} Hz reception, {proc.cpu_percent()}% CPU, max_gap={max_gap:.3f}")
            num_recv = 0
            max_gap = 0.0
            t_last_log = now
        t_last_msg = now

    # Create subs
    qos = qos_profile_sensor_data
    for i in range(num):
        node.create_subscription(Image, f"topic_{i}", callback=callback, qos_profile=qos)

    # Spin in the background
    executor = MultiThreadedExecutor(num_threads=num)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
