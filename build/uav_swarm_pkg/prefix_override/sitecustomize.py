import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ahmet/ros2_plane_workspace/install/uav_swarm_pkg'
