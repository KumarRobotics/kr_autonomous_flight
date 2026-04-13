FROM kumarrobotics/autonomy:base

RUN apt-get update \
    && apt-get install -y \
    ros-jazzy-random-numbers \
    libsuitesparse-dev

RUN mkdir -p /root/estimation_ws/src
WORKDIR /root/estimation_ws

COPY autonomy_core/estimation/ src/
RUN vcs import --input src/external-repos.yaml src/
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY autonomy_core/estimation/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
