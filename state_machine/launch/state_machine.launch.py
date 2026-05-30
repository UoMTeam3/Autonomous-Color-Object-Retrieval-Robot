import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package paths
    mapping_pkg = get_package_share_directory('mapping_bringup')
    depth_cam_pkg = get_package_share_directory('realsense2_camera')
    state_machine_pkg = get_package_share_directory('state_machine')
    explore_pkg = get_package_share_directory('nav2_wfd')
    object_detection_pkg = get_package_share_directory('object_detection')

    mapping_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mapping_pkg, 'launch', 'mapping_bringup.launch.py')
        )
    )

    depth_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depth_cam_pkg, 'launch', 'rs_launch.py')
        ),
            launch_arguments={
            'align_depth.enable': 'true',  
            'enable_color': 'true', 
            'enable_depth': 'true'
        }.items()
    )

    state_machine_launch = Node(
        package= state_machine_pkg,
        executable='state_machine',
        name='state_machine',
        output='screen'
    )

    explore_launch = Node(
        package= explore_pkg,
        executable='explore',
        name='explore',
        output='screen'
    )

    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(object_detection_pkg, 'launch', 'object_detection_launch.py')
        )
    )

    return LaunchDescription([
        depth_cam_launch,
        mapping_bringup_launch,
        object_detection_launch,
        explore_launch,
        state_machine_launch
    ])
