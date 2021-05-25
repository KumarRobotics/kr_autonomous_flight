FROM kumarrobotics/autonomy:base

RUN apt update \
    && apt install -y \
    ros-melodic-random-numbers \
    libsuitesparse-dev

RUN mkdir -p /root/estimation_ws/src
WORKDIR /root/estimation_ws

COPY estimation/ src/
RUN catkin init && catkin config --extend /opt/ros/melodic
RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=Release

COPY estimation/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
