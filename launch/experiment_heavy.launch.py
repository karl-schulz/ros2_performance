from os.path import join

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            join(get_package_share_path("ros2_performance"), "launch/experiment.launch.py")
        ), launch_arguments=[
            ("mb", f"{640 * 360 * 5 / 1024 / 1024:.2f}"),  # Like a 630x360 RGBD image from the realsense (3 bytes RGB uint8, 2 bytes uint16 depth)
            ("hz", "30.0"),
        ])
    ])
