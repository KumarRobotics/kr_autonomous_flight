FROM kumarrobotics/autonomy:map_plan

RUN mkdir -p /root/client_ws/src
WORKDIR /root/client_ws
COPY autonomy_core/client/ src/
RUN . /root/map_plan_ws/install/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY autonomy_core/client/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
