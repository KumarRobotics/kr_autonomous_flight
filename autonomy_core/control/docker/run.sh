#!/bin/bash

# TODO: add command line argument for rosmaster uri
docker run -it --rm --network=host --name autonomy_control_it \
    kumarrobotics/autonomy:control \
    bash
