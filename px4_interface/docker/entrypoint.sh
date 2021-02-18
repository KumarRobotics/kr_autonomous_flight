#!/usr/bin/env bash

if [ -f /root/px4_interface_ws/devel/setup.bash ]; then
    echo "Setting up px4_interface_ws"
    echo "source /root/px4_interface_ws/devel/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/px4_interface_ws/devel/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  

