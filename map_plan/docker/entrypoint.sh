#!/usr/bin/env bash

if [ -f /root/map_plan_ws/devel/setup.bash ]; then
    echo "Setting up map_plan_ws"
    echo "source /root/map_plan_ws/devel/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/map_plan_ws/devel/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  

