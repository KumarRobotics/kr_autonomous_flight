FROM kumarrobotics/autonomy:state_machine

# Gazebo classic is EOL on Jazzy; we install the ros-gz Harmonic bindings instead.
# hector-gazebo has no ROS2/Harmonic drop-in, so the sensor plugins it used to
# supply will need to be replaced in a follow-up (left as TODO).
# TODO: replace hector-gazebo sensor plugins with ros-gz equivalents.
RUN apt update \
    && apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge

RUN mkdir -p /root/sim_ws/src
WORKDIR /root/sim_ws
COPY autonomy_sim/gazebo_sim/ src/
COPY autonomy_sim/external.yaml src/
COPY autonomy_core/client/ src/
RUN vcs import --input src/external.yaml src/
RUN . /root/state_machine_ws/install/setup.sh \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY autonomy_sim/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
