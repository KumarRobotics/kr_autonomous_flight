FROM kumarrobotics/autonomy:map_plan

RUN mkdir -p /root/client_ws/src
WORKDIR /root/client_ws
COPY autonomy_core/client/ src/
RUN catkin init && catkin config --extend /root/map_plan_ws/devel
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=Release

COPY autonomy_core/client/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
