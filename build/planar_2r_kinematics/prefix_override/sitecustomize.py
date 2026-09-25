import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gargi-das/Documents/robotics_playground/ros2_ws/install/planar_2r_kinematics'
