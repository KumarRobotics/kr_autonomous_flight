#!/bin/bash

docker run -it --rm --network=host --name autonomy_px4_interface_it \
    --privileged \
    --volume="/dev/pixhawk:/dev/pixhawk:rw" \
    --volume="/dev/serial:/dev/serial:rw" \
    kumarrobotics/autonomy:px4_interface \
    bash

