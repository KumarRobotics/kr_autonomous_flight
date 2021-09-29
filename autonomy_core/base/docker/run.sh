#!/bin/bash

# TODO: add command line argument for rosmaster uri
docker run -it --rm --network=host --name autonomy_base_it \
    kumarrobotics/autonomy:base \
    bash