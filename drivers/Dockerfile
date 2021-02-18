FROM kumarrobotics/autonomy:base as base

RUN apt update \
    && apt install -y \
    ros-melodic-mavros \
    ros-melodic-mavros-msgs \
    ros-melodic-mavros-extras \ 
    ros-melodic-camera-info-manager\
    setserial

RUN mkdir -p /root/drivers_ws/src 
WORKDIR /root/drivers_ws
# RUN catkin init && catkin config --extend /opt/ros/melodic --install
RUN catkin init && catkin config --extend /opt/ros/melodic


# build
FROM base as build

WORKDIR /root/drivers_ws
COPY external-repos.yaml src/
COPY pkgs src/pkgs

RUN vcs import --input src/external-repos.yaml src/ \
    && catkin build -j4 -DCMAKE_BUILD_TYPE=RelWithDebInfo 

# install
# FROM base
# WORKDIR /root/drivers_ws
# COPY --from=build /root/drivers_ws/install /root/drivers_ws/install
RUN usermod -aG dialout $(whoami)

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

