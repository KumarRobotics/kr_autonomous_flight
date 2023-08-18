FROM kumarrobotics/autonomy:control

RUN apt-get update && \
    apt-get install -y \
    ros-noetic-image-pipeline \
    ros-noetic-image-geometry \
    libqt5widgets5 \
    libnlopt-dev \
    libsdl-dev \
    libsdl-image1.2-dev \
    ros-noetic-image-geometry \
    ros-noetic-depth-image-proc

RUN mkdir -p /root/map_plan_ws/src
WORKDIR /root/map_plan_ws
COPY autonomy_core/map_plan/ src/

RUN catkin init && catkin config --extend /root/control_ws/devel
RUN vcs import --input src/external-repos.yaml src/
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY autonomy_core/map_plan/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
