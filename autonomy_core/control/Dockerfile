FROM kumarrobotics/autonomy:base

RUN apt update \
    && apt install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-msgs \
    ros-melodic-mavros-extras

RUN mkdir -p /root/control_ws/src
WORKDIR /root/control_ws
COPY control/ src/
RUN catkin init && catkin config --extend /opt/ros/melodic
RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=Release

COPY autonomy_core/control/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

