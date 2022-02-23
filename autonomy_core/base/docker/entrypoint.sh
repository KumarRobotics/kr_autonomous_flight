#!/bin/bash
set -e

# setup ros environment
echo "Setting up ros"
echo "source /opt/ros/noetic/setup.bash" >> /$HOME/.bashrc
source /$HOME/.bashrc

source /opt/ros/noetic/setup.bash

if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi  
