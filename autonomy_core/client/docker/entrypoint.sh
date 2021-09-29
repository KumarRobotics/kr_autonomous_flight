#!/usr/bin/env bash

if [ -f /root/client_ws/devel/setup.bash ]; then
    echo "Setting up client_ws"
    echo "source /root/client_ws/devel/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/client_ws/devel/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  

