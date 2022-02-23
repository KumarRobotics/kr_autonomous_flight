FROM kumarrobotics/autonomy:base

RUN apt-get update \
    && apt-get install -y \
    ros-noetic-random-numbers \
    libsuitesparse-dev

RUN mkdir -p /root/estimation_ws/src
WORKDIR /root/estimation_ws

COPY autonomy_core/estimation/ src/
RUN catkin init && catkin config --extend /opt/ros/noetic
RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=Release

COPY autonomy_core/estimation/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
