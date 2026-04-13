#!/bin/bash

# ROS2: no rosmaster URI is needed; use ROS_DOMAIN_ID instead if you want to
# namespace multiple robots on the same network.
docker run -it --rm --network=host --name autonomy_control_ros2_it \
    kumarrobotics/autonomy:control-jazzy \
    bash
