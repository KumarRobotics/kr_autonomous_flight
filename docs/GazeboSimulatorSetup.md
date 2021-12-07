# Gazebo Simulation Setup
1. Make sure you have Gazebo and ROS (ubuntu 18 + ROS melodic OR ubuntu 20 + ROS noetic) intalled.
2. **Build the autonomy stack**:
    1. **Download third party dependencies**: We use [vcstool](https://github.com/dirk-thomas/vcstool) to clone thrid-party repos. You may install it with apt `sudo apt-get install python3-vcstool`.
        ```
        mkdir -p /path_to_your_workspace/src
        cd /path_to_your_workspace/src
        git clone git@github.com:KumarRobotics/autonomy_stack.git
        vcs import < /path_to_your_workspace/src/autonomy_stack/external.yaml
        vcs pull
        ```
    2. **Install dependencies and build everything**:
        ```
        sudo apt install libnlopt-dev
        sudo apt install ros-"your ros distro"-hector-gazebo-plugins
        sudo apt install libsdl-image1.2-dev
        cd /path_to_your_workspace/src
        catkin build -DCMAKE_BUILD_TYPE=Release
        ```

3. **Run the simulation with autonomy stack**:

        ```
        source /path_to_your_workspace/devel/setup.bash
        roslaunch gazebo_utils full_sim.launch
        ```
