FROM kumarrobotics/autonomy:base

RUN apt update \
    && apt install -y \
    ros-melodic-random-numbers \
    libsuitesparse-dev

RUN mkdir -p /root/estimation_ws/src
WORKDIR /root/estimation_ws

COPY estimation/external-repos.yaml src/
COPY estimation/pkgs src/pkgs
RUN catkin init && catkin config --extend /opt/ros/melodic
RUN vcs import --input src/external-repos.yaml src/ \
    && catkin build -j4 -DCMAKE_BUILD_TYPE=RelWithDebInfo 


COPY estimation/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
