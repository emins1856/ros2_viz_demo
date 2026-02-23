import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/emin/ros2_ws/src/ros2_viz_demo/install/ros2_viz_demo'
