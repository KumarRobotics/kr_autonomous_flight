FROM kumarrobotics/autonomy:state_machine

RUN apt update \
    && apt install -y \
    ros-noetic-gazebo-ros \
    ros-noetic-hector-gazebo

RUN mkdir -p /root/sim_ws/src
WORKDIR /root/sim_ws
COPY autonomy_sim/gazebo_sim/ src/
COPY autonomy_sim/external.yaml src/
COPY autonomy_core/client/ src/
RUN catkin init && catkin config --extend /root/state_machine_ws/devel
RUN vcs import --input src/external.yaml src/
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY autonomy_sim/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

