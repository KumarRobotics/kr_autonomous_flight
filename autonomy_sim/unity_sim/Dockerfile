FROM kumarrobotics/autonomy:state_machine

RUN mkdir -p /root/sim_ws/src
WORKDIR /root/sim_ws
COPY pkgs src/pkgs
RUN rm -f src/pkgs/rosflight_msgs/COLCON_IGNORE
RUN . /root/state_machine_ws/install/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
