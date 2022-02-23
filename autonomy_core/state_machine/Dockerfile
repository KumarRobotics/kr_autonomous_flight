FROM kumarrobotics/autonomy:map_plan

RUN apt-get update \
    && apt-get install -y \
    python3-yaml

RUN pip3 install rospkg

RUN mkdir -p /root/state_machine_ws/src 
WORKDIR /root/state_machine_ws
COPY autonomy_core/state_machine/ src/
RUN catkin init && catkin config --extend /root/map_plan_ws/devel
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY autonomy_core/state_machine/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

