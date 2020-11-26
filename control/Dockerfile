FROM kumarrobotics/autonomy:base

RUN apt update \
    && apt install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-msgs \
    ros-melodic-mavros-extras

RUN mkdir -p /root/control_ws/src 
WORKDIR /root/control_ws
COPY pkgs src/pkgs
RUN catkin init && catkin config --extend /opt/ros/melodic \ 
    && catkin build -j4 -DCMAKE_BUILD_TYPE=RelWithDebInfo 

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

