from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share/', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epue',
    maintainer_email='epuethapemi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts':['publisher_node=ros2_opencv.cameraPublisher:main','subscriber_node = ros2_opencv.subscriberimage:main',
        'object_detection_node=ros2_opencv.object_detection:main','image_display_node = ros2_opencv.detection_display:main',
        'driver_node=ros2_opencv.driving:main', 'stop_sign_detection=ros2_opencv.beginstop:main', 'lane_detection_node=ros2_opencv.road_detect:main',
        ],
    },
)
