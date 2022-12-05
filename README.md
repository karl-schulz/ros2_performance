# Toolbox for Benchmarking ROS2 Performance

Deterministic and parametrizable publishers and subscribers for benchmarking ROS2 performance.

#### TL;DR:

Start publisher: `ros2 run ros2_performance publisher --ros-args -p mb:=10.00 -p hz:=10.0`

Start subscriber: `ros2 run ros2_performance subscriber`

#### Measures

 * CPU usage of each node (subscriber/publisher)
 * Message publishing throughput (publisher)
 * Message reception rate (subscriber)
 * Message gaps (subscriber lags)

#### Use Cases

* DDS tuning
* QOS tuning
* Network checking
* Measuring lag and throughput (roughly)

## Content

* Node `publisher`  
  Parameters
  * `mb` [float]: Package size
  * `hz` [float]: Rate of publishing
* Node `subscriber`

## Getting Started

* Clone to your `ros_ws`
* Build and source the workspace

### Usage Examples

1. Start some publishers
  * First publisher in one terminal:  
    `ros2 run ros2_performance publisher --ros-args -p mb:=10.00 -p hz:=10.0`
  * Second publisher in another terminal:  
    `ros2 run ros2_performance publisher --ros-args -p mb:=10.00 -p hz:=10.0`
2. Start some subscribers
  * First subscriber in one terminal:  
    `ros2 run ros2_performance subscriber`
  * Second subscriber in another terminal:  
    `ros2 run ros2_performance subscriber`
3. Observe output
4. Tune your DDS/QOS/Network settings and repeat. 

Happy tuning!

#### Example output:

Publisher
```
[INFO] [1670234709.474729087] [test_pub_815]: 9.0% - 9.99 Hz publishing
[INFO] [1670234711.478127708] [test_pub_815]: 9.0% - 9.98 Hz publishing
[INFO] [1670234713.481909428] [test_pub_815]: 8.5% - 9.98 Hz publishing
[INFO] [1670234715.485013885] [test_pub_815]: 9.0% - 9.99 Hz publishing
```

Subsciber
```
[INFO] [1670234704.964654462] [test_sub_563]: 18.80 Hz reception, 28.9% CPU, max_gap=0.105
[INFO] [1670234706.968516150] [test_sub_563]: 17.47 Hz reception, 25.9% CPU, max_gap=0.102
[INFO] [1670234708.972395378] [test_sub_563]: 18.96 Hz reception, 27.4% CPU, max_gap=0.104
[INFO] [1670234710.973045592] [test_sub_563]: 16.99 Hz reception, 25.0% CPU, max_gap=0.107
```


