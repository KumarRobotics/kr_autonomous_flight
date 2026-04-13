#!/usr/bin/env bash

if [ -f /root/client_ws/install/setup.bash ]; then
    echo "Setting up client_ws"
    echo "source /root/client_ws/install/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/client_ws/install/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  

