import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/senne/ros2_ws/install/lidar_mediapipe_senne_pkg'
