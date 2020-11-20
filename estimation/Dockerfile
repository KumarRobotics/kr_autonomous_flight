FROM kumarrobotics/autonomy:base

RUN apt update \
    && apt install -y \
    ros-melodic-random-numbers \
    libsuitesparse-dev 

RUN mkdir -p /root/estimation_ws/src 
WORKDIR /root/estimation_ws

COPY pkgs src/pkgs
RUN catkin init && catkin config --extend /opt/ros/melodic \
    && catkin build -j4 -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
