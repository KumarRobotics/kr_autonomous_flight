#!/bin/bash

# TODO: add command line argument for rosmaster uri
docker run -it --rm --network=host --name sim \
    kumarrobotics/autonomy:sim \
    bash
