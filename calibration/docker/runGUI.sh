#!/bin/bash

xhost +local:root
docker run -it \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --env="DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "${PWD}/calib_files:/root/calib_files:rw" \
    -v "${HOME}/bags:/root/bags:rw" \
    --rm \
    --privileged \
    --network=host \
    --name client \
    kumarrobotics/autonomy:calibration \
    bash
