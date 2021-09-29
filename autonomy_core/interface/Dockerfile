FROM kumarrobotics/autonomy:control

RUN mkdir -p /root/px4_interface_ws/src
WORKDIR /root/px4_interface_ws
COPY pkgs src/pkgs 
COPY scripts scripts

RUN catkin init && catkin config --extend /root/control_ws/devel \
    && catkin build -j4 -DCMAKE_BUILD_TYPE=RelWithDebInfo 

WORKDIR scripts
RUN chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh

RUN usermod -aG dialout $(whoami)

COPY autonomy_core/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

