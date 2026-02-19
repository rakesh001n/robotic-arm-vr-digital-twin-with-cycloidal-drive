import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rocky/ros_robotic_arm_ws/src/install/robotic_arm'
