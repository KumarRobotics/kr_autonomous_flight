#!/usr/bin/env bash

if [ -f /root/sim_ws/install/setup.bash ]; then
    echo "Setting up sim"
    echo "source /root/sim_ws/install/setup.bash" >> /"$HOME"/.bashrc
    source /"$HOME"/.bashrc
fi

source /root/sim_ws/install/setup.bash

if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi
