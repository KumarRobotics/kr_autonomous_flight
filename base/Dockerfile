FROM nvidia/cudagl:10.2-base-ubuntu18.04

RUN apt update \
    && apt install -y \
    build-essential \
    cmake \
    cppcheck \
    gdb \
    git \
    lsb-release \
    software-properties-common \
    sudo \
    vim \
    wget \
    net-tools \
    iputils-ping \
    tmux \
    && apt clean

RUN export DEBIAN_FRONTEND=noninteractive \
    && sudo apt update \
    && sudo -E apt install -y tzdata \
    && sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
    && sudo dpkg-reconfigure --frontend noninteractive tzdata \
    && sudo apt clean

RUN sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && sudo apt update \
    && sudo apt install -y \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-vcstool \
    ros-melodic-catkin \
    ros-melodic-rosbash \
    ros-melodic-desktop \
    ros-melodic-pcl-ros \
    ros-melodic-tf2-geometry-msgs \
    && sudo rosdep init \
    && sudo apt clean \
    && rosdep update


COPY base/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
