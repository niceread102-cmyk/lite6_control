import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/v/lite6_sim_ws/install/my_lite6_bringup'
