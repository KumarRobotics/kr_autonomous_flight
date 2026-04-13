#!/bin/bash

docker run -it --rm --network=host --name autonomy_map_plan_ros2_it \
    kumarrobotics/autonomy:map_plan-jazzy \
    bash
