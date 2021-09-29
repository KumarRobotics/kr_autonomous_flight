#!/bin/bash

# TODO: add command line argument for rosmaster uri
docker run -it --rm --network=host --name state_machine \
    kumarrobotics/autonomy:state_machine \
    bash
