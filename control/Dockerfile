FROM kumarrobotics/autonomy:base

RUN apt update \
    && apt install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-msgs \
    ros-melodic-mavros-extras

RUN mkdir -p /root/control_ws/src
WORKDIR /root/control_ws
COPY control/pkgs src/pkgs
RUN catkin init && catkin config --extend /opt/ros/melodic

# FIXME: We don't run vcs import as quadrotor_control is still a private repo. We have
# to clone it before building with github actions
# RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j4 -DCMAKE_BUILD_TYPE=Release

COPY control/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

