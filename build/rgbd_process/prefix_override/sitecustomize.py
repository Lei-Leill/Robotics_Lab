import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/homes/obluman/Robotics_Lab/install/rgbd_process'
