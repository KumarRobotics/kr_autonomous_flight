#!/bin/bash

if [ $# -eq 0 ]; then
    ENV_FILE="../../env_files/emu1.sh"
else
    ENV_FILE=$1
fi
echo "env_file ${ENV_FILE}"

xhost +local:root
docker run --gpus all -it \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --env="DISPLAY=$DISPLAY" \
    --env-file="$ENV_FILE" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "${PWD}/calib_files:/root/calib_files:rw" \
    -v "${HOME}/bags:/root/bags:rw" \
    --rm \
    --privileged \
    --network=host \
    --name client \
    kumarrobotics/autonomy:calibration \
    bash
