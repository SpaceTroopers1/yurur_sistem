import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/omer/GitHub/yurur_sistem/ros2_ws/install/motor_kontrol'
