import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/epue/capstone.git/capstone/ros2_sample/install/sample_pubsub'
