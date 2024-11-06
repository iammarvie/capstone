from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
             package='ros2_opencv',
            executable='publisher_node',
            name='image_publisher',
            #output='screen'
        ),
        Node(
            package='ros2_opencv',
            executable='object_detection_node',
            name='object_detection_node',
            #output='screen'
        ),
        Node(
            package='ros2_opencv',
            executable='image_display_node',
            name='image_display_node',
            #output='screen'
        ),
        Node(
            package='ros2_opencv',
            executable='driver_node',
            name='driving_node',
            #output='screen'
        ),
        Node(
            package='ros2_opencv',
            executable='stop_sign_detection',
            name='stop_sign_detection',
            output='screen'
        ),
        Node(
            package='ros2_opencv',
            executable='lane_detection_node',
            name='lane_detection',
            output='screen'        ),
        Node(
            package='ros2_opencv',
            executable='steering_node',
            name='steering',
            output='screen'        ),
        
        #Node(
         #   package='ros2_opencv',
          #  executable='obstacle_avoidance',
           # name='ultrasonic',
            #output='screen'        ),
    ])
