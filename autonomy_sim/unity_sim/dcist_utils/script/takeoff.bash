#!/bin/bash

if [ $# -eq 0 ]; then
    ROBOT="quadrotor"
else
    ROBOT=$robot
fi
echo "robot name ${ROBOT}"

# echo "Enable motors..."
# rosservice call /$ROBOT/mav_services/motors true
# sleep 1

# echo "Takeoff..."
# rosservice call /$ROBOT/mav_services/takeoff
# sleep 1

rosrun arl_unity_ros_air rosflight_offboard.py __ns:=${ROBOT}

# read -p "Press [Enter] to go to [1, 1, 1]"
# rosservice call /$ROBOT/mav_services/goTo "goal: [1.0, 1.0, 1.0, 0.0]"
# sleep 1
