#!/bin/bash

docker run -it --rm --network=host --name autonomy_estimation_ros2_it \
    kumarrobotics/autonomy:estimation-jazzy \
    bash
