FROM nvidia/cudagl:10.2-base-ubuntu18.04

RUN apt-get update \
    && apt-get install -y \
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
    python3-pip \
    && apt-get clean

RUN export DEBIAN_FRONTEND=noninteractive \
    && sudo apt-get update \
    && sudo -E apt-get install -y tzdata curl \
    && sudo ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
    && sudo dpkg-reconfigure --frontend noninteractive tzdata \
    && sudo apt-get clean

RUN sudo /bin/sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -  \
    && sudo apt-get update \
    && sudo apt-get install -y \
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
    && rosdep update

COPY autonomy_core/base/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
