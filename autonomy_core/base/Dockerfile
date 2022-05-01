FROM nvidia/cudagl:11.4.2-base-ubuntu20.04

RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub

#Run the frontend first so it doesn't throw an error later
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update \
 && export TZ="America/New_York" \
 && apt-get install -y keyboard-configuration tzdata \
 && ln -fs "/usr/share/zoneinfo/$TZ" /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt-get clean

# General tools
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
      neovim \
      wget \
      net-tools \
      iputils-ping \
      tmux \
      locales \
      python3-pip \
      curl \
    && apt-get clean

# Set locales
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en

# ROS Setup
RUN sudo apt-get update \
    && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && sudo apt-get update \
    && sudo apt-get install -y \
      python3-catkin-tools \
      python3-rosdep \
      python3-rosinstall \
      python3-vcstool \
      ros-noetic-catkin \
      ros-noetic-rosbash \
      ros-noetic-desktop \
      ros-noetic-pcl-ros \
      ros-noetic-tf2-geometry-msgs \
    && sudo rosdep init \
    && rosdep update

COPY autonomy_core/base/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
