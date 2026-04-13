#!/usr/bin/env bash

if [ -f /root/state_machine_ws/install/setup.bash ]; then
  echo "Setting up sim_ws"
  echo "source /root/sim_ws/install/setup.bash" >>/$HOME/.bashrc
  source /$HOME/.bashrc
fi

source /root/sim_ws/install/setup.bash

if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi
