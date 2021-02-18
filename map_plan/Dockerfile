FROM kumarrobotics/autonomy:control

RUN apt update \
    && apt install -y \
    libsdl1.2-dev  \
    libsdl-image1.2-dev \
    ros-melodic-image-pipeline \
    ros-melodic-image-geometry \
    libqt5widgets5 \
    libnlopt-dev

RUN mkdir -p /root/map_plan_ws/src 
WORKDIR /root/map_plan_ws
COPY pkgs src/pkgs

RUN git clone https://github.com/catkin/catkin_simple.git src/catkin_simple \
    && catkin init && catkin config --extend /root/control_ws/devel \
    && catkin build -j4 -DCMAKE_BUILD_TYPE=RelWithDebInfo

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
