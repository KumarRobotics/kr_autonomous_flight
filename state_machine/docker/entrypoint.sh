#!/usr/bin/env bash

if [ -f /root/state_machine_ws/devel/setup.bash ]; then
    echo "Setting up state_machine_ws"
    echo "source /root/state_machine_ws/devel/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/state_machine_ws/devel/setup.bash

if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi  

