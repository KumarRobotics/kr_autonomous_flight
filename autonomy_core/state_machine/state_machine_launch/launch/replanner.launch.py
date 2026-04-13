# Deprecated launch stub.
#
# The original ROS1 path_replanner C++ node (src/path_replanner.cpp in
# state_machine_core) was dead code in master — it was commented out in
# the old CMakeLists and depended on a JPS stack that no longer exists in
# the tree. The ROS2 port dropped the source file entirely, so this
# launch file can no longer instantiate it.
#
# The launch file is kept as a stub (returning an empty LaunchDescription)
# so that any tmux / bringup script that still tries to `ros2 launch
# state_machine_launch replanner.launch.py` fails gracefully with an
# empty graph instead of crashing with "executable 'path_replanner' not
# found in package 'state_machine'".
#
# If someone needs a ROS2 path replanner node, port the original
# src/path_replanner.cpp from the historical ROS1 source and add the
# executable to state_machine_core's CMakeLists, then reinstate the Node()
# call below.

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
