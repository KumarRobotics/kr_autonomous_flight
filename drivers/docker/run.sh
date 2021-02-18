#!/bin/bash

# TODO: add command line argument for rosmaster uri
docker run -it --rm --network=host --name autonomy_drivers_it \
    kumarrobotics/autonomy:drivers \
    bash
