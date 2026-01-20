import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lorenzo/autonomy-ws-25-26/computer_vision/install/cv_ros_nodes'
