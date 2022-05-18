import os

from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    azure_kinect = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('azure_kinect_ros_driver'), 'launch'),
            '/driver.launch.py']), launch_arguments={'color_enabled': 'true',
                'color_resolution': '1080P',
                'depth_mode': 'NFOV_UNBINNED',
                'fps': '15',
                'point_cloud': 'true',
                'rgb_point_cloud': 'false'}.items()  # Change launch arguments in a specific file
    )

    return LaunchDescription([
        azure_kinect,
        DeclareLaunchArgument(
            'werklicht_extrinsic_path',
            default_value="c:/Projects/ros_ws/calib/extrinsics.xml",
            description="Path to werklich projector extrinsics"),   

        DeclareLaunchArgument(
            'werklicht_intrinsic_path',
            default_value="c:/Projects/ros_ws/calib/intrinsics.xml",
            description="Path to werklicht projector intrinsics"),
        Node(
            package='static_pose_injector',
            executable='broadcaster',
            output='screen',
            parameters=[
                {'werklicht_extrinsic_path': launch.substitutions.LaunchConfiguration(
                    'werklicht_extrinsic_path')} ]),
        Node(
            package='degginger_draw',
            executable='draw',
            output='screen',
            parameters=[]),
        Node(
            package='static_pose_injector',
            executable='camerainfo_projector',
            output='screen',
            parameters=[
                {'werklicht_intrinsic_path': launch.substitutions.LaunchConfiguration(
                    'werklicht_intrinsic_path')}]),
        Node(
            package='touch_tracker',
            executable='tracker',
            output='screen',
            parameters=[]),        
    ])