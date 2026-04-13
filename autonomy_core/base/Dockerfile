# TODO: restore CUDA runtime support (original used nvidia/cudagl:11.4.2-base-ubuntu20.04).
# The nvidia/cudagl lineage was EOL'd and there is no drop-in Jazzy + CUDA image.
# osrf/ros:jazzy-desktop-full covers graphics needs (rviz2, gazebo harmonic) but
# compute-oriented CUDA workloads (e.g. yolov8 inference) will need a follow-up
# that either layers CUDA onto this image or switches to nvidia/cuda:12.x-runtime-ubuntu24.04.
FROM osrf/ros:jazzy-desktop-full

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

# ROS2 Jazzy tools. The osrf/ros:jazzy-desktop-full base already ships the ROS2
# apt source, colcon, rosdep, and the desktop-full metapackage, so we only need
# to pull in the extra packages the autonomy stack depends on.
RUN sudo apt-get update \
    && sudo apt-get install -y \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-jazzy-pcl-ros \
      ros-jazzy-tf2-geometry-msgs \
      ros-jazzy-tf2-ros \
    && (sudo rosdep init || true) \
    && rosdep update

COPY autonomy_core/base/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
