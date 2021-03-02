FROM kumarrobotics/autonomy:map_plan

RUN mkdir -p /root/client_ws/src
WORKDIR /root/client_ws
COPY pkgs src/pkgs
RUN catkin init && catkin config --extend /root/map_plan_ws/devel
RUN catkin build -j2 --no-status -DCMAKE_BUILD_TYPE=Release

COPY client/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
