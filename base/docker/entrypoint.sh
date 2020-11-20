#!/usr/bin/env bash
set -e

# setup ros environment
echo "Setting up ros"
echo "source /opt/ros/melodic/setup.bash" >> /$HOME/.bashrc
source /$HOME/.bashrc

source /opt/ros/melodic/setup.bash

if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi  
