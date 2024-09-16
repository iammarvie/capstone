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
        (os.path.join('share/' + package_name, 'launch'), glob('/*.camera_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epue',
    maintainer_email='epuethapemi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':['publisher_node=ros2_opencv.cameraPublisher:main','subscriber_node = ros2_opencv.subscriberimage:main',
        ],
    },
)
