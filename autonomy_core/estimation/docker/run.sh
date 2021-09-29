#!/bin/bash

docker run -it --rm --network=host --name autonomy_estimation_it \
    kumarrobotics/autonomy:estimation \
    bash
