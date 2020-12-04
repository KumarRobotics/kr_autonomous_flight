FROM kumarrobotics/autonomy:base

RUN apt update \
    && rosdep update \
    && apt install -y python-setuptools \
    python-rosinstall \
    ipython \
    libeigen3-dev \
    libboost-all-dev \
    doxygen \
    libopencv-dev \
    ros-melodic-vision-opencv \
    ros-melodic-image-transport-plugins \
    ros-melodic-cmake-modules \
    software-properties-common \
    libpoco-dev \
    python-matplotlib \
    python-scipy \
    python-git \
    python-pip \
    ipython \
    libtbb-dev \
    libblas-dev \
    liblapack-dev \
    python-catkin-tools \
    libv4l-dev \
    python-igraph \ 
    libceres-dev \
    python3-pip \
    && pip3 install -U pyyaml 

RUN mkdir -p /root/calibration_ws/src 
WORKDIR /root/calibration_ws
COPY pkgs src/pkg
COPY external-repos.yaml src/
RUN vcs import --input src/external-repos.yaml src/ \
    && catkin init && catkin config --extend /opt/ros/melodic && catkin build -j16 -DCMAKE_BUILD_TYPE=RelWithDebInfo 


RUN usermod -aG dialout $(whoami)
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]