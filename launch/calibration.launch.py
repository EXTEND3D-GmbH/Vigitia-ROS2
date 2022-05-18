import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    azure_kinect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('azure_kinect_ros_driver'), 'launch'),
            '/driver.launch.py']), launch_arguments={'color_enabled': 'true',
                'color_resolution': '1080P',
                'depth_mode': 'NFOV_UNBINNED',
                'fps': '15',
                'point_cloud': 'false',
                'rgb_point_cloud': 'false'}.items()  # Change launch arguments in a specific file
    )

    return LaunchDescription([
        azure_kinect,
        Node(
            package='projector_display',
            executable='display',
            parameters=[
                {'window_origin.x': 3002}
            ],
        ),
        Node(
            package='calibration',
            executable='projector_calibration'
        ),
    ])