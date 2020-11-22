FROM kumarrobotics/autonomy:base

# From these packages, libqt4 and image-geometry are required for supplementary
# packages of kr-utils. These may be removed if we don't compile the whole
# library
RUN apt update \
    && apt install -y \
    ros-melodic-random-numbers \
    libsuitesparse-dev \
    libqt4-dev \
    ros-melodic-image-geometry

RUN mkdir -p /root/estimation_ws/src
WORKDIR /root/estimation_ws

COPY estimation/external-repos.yaml src/
COPY estimation/pkgs src/pkgs
RUN catkin init && catkin config --extend /opt/ros/melodic
RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j4 -DCMAKE_BUILD_TYPE=Release


COPY estimation/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
