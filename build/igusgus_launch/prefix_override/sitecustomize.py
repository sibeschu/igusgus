import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amrl-linux24-04/ros2_ben/install/igusgus_launch'
