FROM kumarrobotics/autonomy:state_machine

RUN mkdir -p /root/sim_ws/src 
WORKDIR /root/sim_ws
COPY pkgs src/pkgs
RUN rm src/pkgs/rosflight_msgs/CATKIN_IGNORE
RUN catkin init && catkin config --extend /root/state_machine_ws/devel \
    && catkin build -j8 -DCMAKE_BUILD_TYPE=RelWithDebInfo 

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

