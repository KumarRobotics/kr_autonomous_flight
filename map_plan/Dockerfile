FROM kumarrobotics/autonomy:control

RUN apt update \
    && apt install -y \
    libsdl1.2-dev  \
    libsdl-image1.2-dev \
    ros-melodic-image-pipeline \
    ros-melodic-image-geometry \
    libqt5widgets5 \
    libnlopt-dev

RUN mkdir -p /root/map_plan_ws/src
WORKDIR /root/map_plan_ws
COPY map_plan/pkgs src/pkgs
COPY map_plan/external-repos.yaml src/

RUN git clone https://github.com/catkin/catkin_simple.git src/catkin_simple
RUN catkin init && catkin config --extend /root/control_ws/devel
RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY map_plan/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
