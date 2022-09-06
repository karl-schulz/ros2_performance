from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    conf_mb = LaunchConfiguration("mb")
    conf_hz = LaunchConfiguration("hz")
    conf_qos = LaunchConfiguration("qos", default="sensor")

    pub = Node(
        name="pub",
        executable="publisher",
        package="ros2_performance",
        parameters=[{
            "mb": conf_mb,
            "hz": conf_hz,
            "qos": conf_qos,
        }],
        output="screen",
    )
    sub_0 = Node(
        name="sub_0",
        executable="subscriber",
        package="ros2_performance",
        parameters=[{
            "qos": conf_qos,
        }],
        output="screen",
    )
    sub_1 = Node(
        name="sub_1",
        executable="subscriber",
        package="ros2_performance",
        parameters=[{
            "qos": conf_qos,
        }],
        output="screen",
    )
    return LaunchDescription([
        pub,
        sub_0,
        sub_1,
    ])
