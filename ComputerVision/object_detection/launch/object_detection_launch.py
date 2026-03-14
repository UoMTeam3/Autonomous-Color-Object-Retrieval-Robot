from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    object_detection_node = Node(
        output='screen',
        emulate_tty=True,
        package='object_detection',
        executable='object_publisher',
        name='object_detection_publisher'
    )

    tf2_transform_node = Node(
        output='screen',
        emulate_tty=True,
        package='object_detection',
        executable='tf2_transform',
        name='tf2_coordinate_transform'
    )

    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.1', '0.0', '0.0',      # x y z (adjust to your LiDAR mounting)
            '0.0', '0.0', '0.0',      # roll pitch yaw
            'base_link',
            'camera_link'
        ]
    )
    return LaunchDescription([
        object_detection_node,
        tf2_transform_node, 
        static_transform
    ])