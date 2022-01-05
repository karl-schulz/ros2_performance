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
    proc = psutil.Process()

    last_log = time()
    num_recv = 0
    log_interval = Interval(2.0)

    def callback(msg: Image):
        nonlocal num_recv, last_log, log_interval
        num_recv += 1
        if log_interval.tick():
            now = time()
            dt = now - last_log
            print(f"{num_recv / dt:.2f} FPS reception, {proc.cpu_percent()}% CPU")
            num_recv = 0
            last_log = now

    # Create subs
    qos = qos_profile_sensor_data
    for i in range(num):
        node.create_subscription(Image, f"topic_{i}", callback=callback, qos_profile=qos)

    # Spin in the background
    executor = MultiThreadedExecutor(num_threads=num)
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
