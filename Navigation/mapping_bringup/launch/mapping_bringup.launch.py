#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    FrontendLaunchDescriptionSource
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package paths
    rplidar_pkg = get_package_share_directory('rplidar_ros')
    leo_nav_pkg = get_package_share_directory('leo_nav')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_pkg, 'launch', 'rplidar_a2m12_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser'
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(leo_nav_pkg, 'launch', 'navigation.launch.xml')
        ),
        launch_arguments={
            'localization': 'false',   # false = SLAM mode
            'scan_topic': '/scan'
        }.items()
    )
    return LaunchDescription([
        lidar_launch,
        navigation_launch
    ])
