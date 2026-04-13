#!/bin/bash

if [ -f /root/estimation_ws/install/setup.bash ]; then
    echo "Setting up estimation_ws"
    echo "source /root/estimation_ws/install/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/estimation_ws/install/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  

