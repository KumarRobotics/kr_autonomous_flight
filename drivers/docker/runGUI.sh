#!/bin/bash

if [ $# -eq 0 ]; then
    ENV_FILE="../../env_files/emu1.sh"
else
    ENV_FILE=$1
fi
echo "env_file ${ENV_FILE}"

xhost +local:root
docker run -it \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    --env-file="$ENV_FILE" \
    --env="DISPLAY=$DISPLAY" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/dev/serial:/dev/serial:rw" \
    -v "/home/drony/bags:/root/bags:rw" \
    --rm \
    --privileged \
    --network=host \
    --name client \
    kumarrobotics/autonomy:drivers \
    bash