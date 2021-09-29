#!/bin/bash

docker run -it --rm --network=host --name autonomy_map_plan_it \
    kumarrobotics/autonomy:map_plan \
    bash
