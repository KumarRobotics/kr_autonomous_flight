#!/bin/bash

if [ $# -eq 0 ]; then
    ROBOT="quadrotor"
else
    ROBOT=$robot
fi
echo "robot name ${ROBOT}"

# echo "Enable motors..."
# ros2 service call /$ROBOT/mav_services/motors std_srvs/srv/SetBool "{data: true}"
# sleep 1

# echo "Takeoff..."
# ros2 service call /$ROBOT/mav_services/takeoff std_srvs/srv/Trigger "{}"
# sleep 1

ros2 run arl_unity_ros_air rosflight_offboard --ros-args -r __ns:=${ROBOT}

# read -p "Press [Enter] to go to [1, 1, 1]"
# ros2 service call /$ROBOT/mav_services/goTo kr_mav_msgs/srv/Vec4 "{goal: [1.0, 1.0, 1.0, 0.0]}"
# sleep 1
