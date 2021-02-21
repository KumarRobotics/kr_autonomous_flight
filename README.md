# Autonomy Stack

This is the autonomy stack for quadrotor simulation and hardware.

This readme consists of two parts: (1) [use without docker (i.e. build from source)]() and (2)[use with docker](https://github.com/KumarRobotics/autonomy_stack/blob/update_docs/README.md#part2-use-with-docker) .

If you are a user who will not make changes to this code stack and is comfortable with docker, (1) is recommended; Otherwise, (2) is recommended.


# Part1: Use without docker (i.e. build from source)
1. Follow the [build from source instructions](https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros#building-from-source) to build arl-unity-ros simulator.
### Troubleshooting
1. For any errors related to rosfligt, try the following:
```
cd ~/arl-unity-ros (your arl-unity-ros workspace folder)
cd src/external/rosflight
git checkout tags/1.3.1
cd ../../..
catkin clean
catkin build -DCMAKE_BUILD_TYPE=Release 
```
If this still does not work, download the rosflight.zip file from [here](https://drive.google.com/drive/folders/1pbBVoYd5NfYJhfe-Of0q2kD70HWSEeNK?usp=sharing), unzip it and use the extracted folder to replace the /src/external/rosflight folder in your workspace, then
```
cd ~/arl-unity-ros (your arl-unity-ros workspace folder)
catkin clean
catkin build -DCMAKE_BUILD_TYPE=Release 
```


## Step 2: Build autonomy stack

Firstly, clone the following repo:
```
git clone https://github.com/KumarRobotics/autonomy_stack
git clone https://github.com/KumarRobotics/autonomy_simulation
git clone https://github.com/catkin/catkin_simple.git
git clone --branch catkin https://github.com/KumarRobotics/motion_primitive_library.git
git clone --branch catkin https://github.com/KumarRobotics/msckf_vio
git clone --branch catkin https://github.com/KumarRobotics/kr_mav_control.git
git clone --branch catkin https://github.com/KumarRobotics/jps3d
git clone --branch catkin https://github.com/KumarRobotics/DecompUtil
git clone --branch catkin https://github.com/KumarRobotics/DecompROS.git
```

Secondly, go to arl-unity-ros, create symbolic links of the repos you clone:
```
cd ~/arl-unity-ros (or your arl-unity-ros workspace folder)
cd src
git clone https://github.com/catkin/catkin_simple
sudo apt install libnlopt-dev
sudo apt install libsdl-image1.2-dev
ln -s ~/autonomy_stack/client/
ln -s ~/autonomy_stack/control/
ln -s ~/autonomy_stack/map_plan/
ln -s ~/autonomy_stack/state_machine/
ln -s ~/autonomy_stack/sim/
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release 
```

### Troubleshooting
None

## Step 3: Run the simulation with autonomy stack
Open four terminals, in **each** terminal source arl-unity-ros workspace you just built:
```
source ~/arl-unity-ros/devel/setup.bash
```

Then, following the instructions in **one of the following scenarios** according to your application:

### Scenario 1: Run in DCIST prebuilt environments + use ground truth pose
In the 1st terminal:
```
roslaunch dcist_utils sim_quad.launch
```
**Wait unitl everything finishes, you should see a UAV generated in the simulation environment**

In the 2nd terminal:
```
roslaunch client_launch client.launch 
```
Then, click “motors on” in the GUI.

In the 3rd terminal:
```
roslaunch state_machine_launch system.launch
```

In the 4th terminal:
```
rosrun dcist_utils takeoff.bash
```
**Wait unitl finishes, i.e., nothing is being printed in the terminal**
Then, click “take off” in the GUI, and you should see the UAV take off. 



### Scenario 2 Customize and run in your own environments + use ground truth pose
**PLease go to this link with SEAS account for the manual, download the script file and use the script and Unity Editor according to the instructions. This manual contains: 1. How to install the UnityHub and configure the correct version of Unity Editor and arl-unity-robotics repository, 2. How to customize your Unity simulation environments (finding suitable objects and placing them; saving the dynamic map as static scene) And the script is used for massively placing objects.
https://drive.google.com/drive/folders/1VbmOorFlEIpqCw3hX1xXsYKw7Pj8fMpS?usp=sharing. **

Click "Play" in Unity Editor.

In the 1st terminal:
```
roslaunch dcist_utils sim_quad.launch launch_unity:=false
```
For the other 3 terminals, do the same as in Scenario 1.

**Wait unitl everything finishes, you should see a UAV generated in the simulation environment**

### Scenario 3: Customize and run in your own environments + use onboard VIO pose
Follow the steps in Scenario 2, only difference is that, in the 1st terminal, run:
```
roslaunch dcist_utils sim_quad_onboard_vision_only.launch
```

### Troubleshooting
1. No UAV generated in simulation:
Redo the steps for the 1st terminal.

2. UAV not taking off:
Try clicking “motors on“ again, and then click “take off“. If does not work, kill all 4 terminals, and redo all procedures in this step.

3. Cannot find packages or launch files:
Remember to **always** source the workspace in each newly opened terminal.





# Part2: Use with docker

## Visit [arl-unity-ros](https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros) for pre-built docker images for simulation:

### Download and rename docker image
```
docker login registry.gitlab.sitcore.net
docker pull registry.gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros:master
docker tag registry.gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros:master arl-unity-ros:latest
```

### Download “run.sh” and run the following
```
run.sh arl-unity-ros:latest
```

### Run Unity simulator with Husky and RVIZ (for checking simulator)
```
git clone https://gitlab.sitcore.net/arl/robotics-simulation/arl-unity-ros.git
roslaunch arl_unity_ros_ground simulator_with_husky.launch rviz:=true
```

## Visit [dcist-platform-supplemental](https://gitlab.sitcore.net/dcist/dcist-platform-supplemental) for running robots:

### Requirements
1. nvidia-docker2 is assumed
2. Modify `/etc/docker/daemon.json`
```
{
    "default-runtime": "nvidia",
    "runtimes": {
      "nvidia": {
        "path": "nvidia-container-runtime",
        "runtimeArgs": []
       }
     }
}
```
3. Access to Xserver
```
xhost +
```
4. Login to Sitcore with the following port:
```
docker login registry.gitlab.sitcore.net:443
```
### Troubleshooting 
* If login fails (e.g. timeout exceeded), set DNS server to 8.8.8.8 and reconnect the internet

## Usage
1. docker-compose the yaml file
```
git clone https://gitlab.sitcore.net/dcist/dcist-platform-supplemental.git
cd dcist-platform-supplemental
git checkout feature/penn-quad
docker-compose -f falcon_lejeune.yml down
docker-compose -f falcon_lejeune.yml up
```

2. Open a separate terminal and run 
```
docker attach dcist-platform-supplemental_falcon_gui_1 
roslaunch client_launch client.launch 
```
In the gui, click `motors on`

3. Open a separate terminal and run 
```
docker attach dcist-platform-supplemental_falcon_bash_1 
source install/setup.bash 
rosrun arl_unity_ros_air rosflight_offboard.py __ns:=falcon
```
4. Set waypoints in rviz and click `short range`

### Troubleshooting 
* If you see many error messages, go to `dcist-platform-supplemental` folder and open `falcon_lejeune.yml`
* Go to line 123 and change into `sleep 30` (longer time)



# General troubleshooting:
1. If semantic segmentation does not work correctly (giving black colored mask instead of the correct color you specify) in Unity simulator, this is usually because you add multi-shader or multi-material objects to the scene. Try downloading the two files [here](https://drive.google.com/drive/folders/1Rp1OCEIkL-VwSSPR4qVG91J4yWGqYDOD?usp=sharing) and replace the corresponding two files in arl-unity-robotics/Assets/scripts with those two.


