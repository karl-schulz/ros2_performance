#!/usr/bin/python3
from random import randint
from threading import Thread, Event

import numpy as np
import psutil
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Image

from ros2_performance.util import Interval


def main(args=None):
    rclpy.init()
    node = rclpy.create_node(f"test_pub_{randint(0, 1000)}")
    qos_name = node.declare_parameter("qos", "sensor").value  # "sensor" or default
    num_pubs = node.declare_parameter("num", 1).value
    mb = node.declare_parameter("mb", 10.0).value
    hz = node.declare_parameter("hz", 10.0).value
    proc = psutil.Process()

    # Create pubs
    qos = qos_profile_sensor_data if qos_name == "sensor" else qos_profile_system_default
    length = round(mb * 1e6)
    msg = Image(data=np.empty(shape=length, dtype=np.uint8).tobytes())
    pubs = [node.create_publisher(Image, f"topic_{i}", qos, callback_group=ReentrantCallbackGroup()) for i in range(num_pubs)]

    node.get_logger().info(f"Images with {hz:.1f} hz of size {msg.data.buffer_info()[1] * msg.data.itemsize / 1024:.1f} kB, qos is {qos_name}")

    # Spin
    executor = MultiThreadedExecutor(num_threads=num_pubs)
    executor.add_node(node)
    stop_event = Event()

    def spin():
        while not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    thread = Thread(target=spin)
    thread.start()

    pub_interval = Interval(1 / hz)
    diag_interval = Interval(2.0)
    try:
        while thread.is_alive():
            if pub_interval.tick():
                for pub in pubs:
                    pub.publish(msg)
            else:
                pub_interval.wait()
            if diag_interval.tick():
                node.get_logger().info(f"{proc.cpu_percent()}%")
    except KeyboardInterrupt:
        stop_event.set()
        pass


if __name__ == '__main__':
    main()
