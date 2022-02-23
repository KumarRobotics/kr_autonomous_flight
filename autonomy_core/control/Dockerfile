FROM kumarrobotics/autonomy:base

RUN apt-get update \
    && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-mavros-extras

RUN mkdir -p /root/control_ws/src
WORKDIR /root/control_ws
COPY autonomy_core/control/ src/
RUN catkin init && catkin config --extend /opt/ros/noetic
RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=Release

COPY autonomy_core/control/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

