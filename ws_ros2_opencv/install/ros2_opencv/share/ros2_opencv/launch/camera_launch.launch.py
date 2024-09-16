from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_opencv',
            executable='publisher_node',
            name='camera_publisher'
        ),
        Node(
            package='ros2_opencv',
            executable='subscriber_node',
            name='camera_subscriber'
        ),
    ])
