FROM kumarrobotics/autonomy:map_plan

RUN mkdir -p /root/client_ws/src 
WORKDIR /root/client_ws
COPY pkgs src/pkgs
RUN catkin init && catkin config --extend /root/map_plan_ws/devel \
    && catkin build -j8 -DCMAKE_BUILD_TYPE=RelWithDebInfo 

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

