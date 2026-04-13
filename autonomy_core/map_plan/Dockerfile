FROM kumarrobotics/autonomy:control

RUN apt-get update \
    && apt-get install -y \
    ros-jazzy-image-pipeline \
    ros-jazzy-image-geometry \
    libqt5widgets5 \
    libnlopt-dev

RUN mkdir -p /root/map_plan_ws/src
WORKDIR /root/map_plan_ws
COPY autonomy_core/map_plan/ src/

RUN vcs import --input src/external-repos.yaml src/
RUN . /root/control_ws/install/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY autonomy_core/map_plan/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
