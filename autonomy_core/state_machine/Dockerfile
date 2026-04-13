FROM kumarrobotics/autonomy:map_plan

RUN apt-get update \
    && apt-get install -y \
    python3-yaml

RUN pip3 install --break-system-packages rospkg

RUN mkdir -p /root/state_machine_ws/src
WORKDIR /root/state_machine_ws
COPY autonomy_core/state_machine/ src/
RUN . /root/map_plan_ws/install/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY autonomy_core/state_machine/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
