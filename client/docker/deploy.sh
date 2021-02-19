#!/bin/bash

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
      touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# TODO: add command line argument for rosmaster uri
docker run --gpus all -it \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    --rm \
    --privileged \
    --network=host \
    --name client \
    kumarrobotics/autonomy:client \
    roslaunch client_launch client.launch
