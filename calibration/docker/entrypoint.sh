#!/usr/bin/env bash

if [ -f /root/calibration_ws/devel/setup.bash ]; then
    echo "Setting up calibration_ws"
    echo "source /root/calibration_ws/devel/setup.bash" >> /$HOME/.bashrc
    source /$HOME/.bashrc
fi

source /root/calibration_ws/devel/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  

