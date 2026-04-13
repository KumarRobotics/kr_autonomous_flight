#!/bin/bash

# ROS2: no rosmaster URI is needed; use ROS_DOMAIN_ID instead if you want to
# namespace multiple robots on the same network.
docker run --gpus all -it --rm --network=host --name sim_ros2 \
    --gpus all \
    --privileged \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="$XAUTH" \
    kumarrobotics/autonomy:sim-jazzy \
    bash
