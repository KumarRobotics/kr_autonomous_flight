#!/usr/bin/env bash

# if [ -f /root/drivers_ws/install/setup.bash ]; then
#     echo "Setting up drivers_ws"
#     echo "source /root/drivers_ws/install/setup.bash" >> /$HOME/.bashrc
#     echo "setserial $SYNC_IMU_PORT low_latency" >> /$HOME/.bashrc 
#     source /$HOME/.bashrc
# fi

# source /root/drivers_ws/install/setup.bash

# if [ "$#" -eq 0 ]; then
#     exec bash
# else
#   exec "$@"
# fi  


if [ -f /root/drivers_ws/devel/setup.bash ]; then
    echo "Setting up drivers_ws"
    echo "source /root/drivers_ws/devel/setup.bash" >> /$HOME/.bashrc
    echo "setserial $SYNC_IMU_PORT low_latency" >> /$HOME/.bashrc 
    source /$HOME/.bashrc
fi

source /root/drivers_ws/devel/setup.bash

if [ "$#" -eq 0 ]; then
    exec bash
else
  exec "$@"
fi  