#!/bin/bash

if [ -f /root/control_ws/install/setup.bash ]; then
    echo "Setting up control_ws"
    echo "source /root/control_ws/install/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/control_ws/install/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  

