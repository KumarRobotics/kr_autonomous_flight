FROM kumarrobotics/autonomy:base

RUN apt-get update \
    && apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-mavros-extras

RUN mkdir -p /root/control_ws/src
WORKDIR /root/control_ws
COPY autonomy_core/control/ src/
RUN vcs import --input src/external-repos.yaml src/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY autonomy_core/control/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
