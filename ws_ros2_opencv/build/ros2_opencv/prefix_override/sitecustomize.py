import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/epue/capstone.git/capstone/ws_ros2_opencv/install/ros2_opencv'
