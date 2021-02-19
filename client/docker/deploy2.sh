#!/bin/bash

xhost +local:root
docker run --gpus all -it \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --env="DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    --rm \
    --privileged \
    --network=host \
    --name client \
    kumarrobotics/autonomy:client \
    roslaunch client_launch client.launch
