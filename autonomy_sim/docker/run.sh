#!/bin/bash

# TODO: add command line argument for rosmaster uri
docker run --gpus all -it --rm --network=host --name state_machine \
    --gpus all \
    --privileged \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY="$XAUTH" \
    kumarrobotics/autonomy:sim \
    bash
