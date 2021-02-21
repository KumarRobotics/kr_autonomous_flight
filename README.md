# Autonomy Stack

This is the autonomus flight code stack. Together our supplementary repository for real_robot [KumarRobotics/autonomy_real_robot](https://github.com/KumarRobotics/autonomy_real_robot) or Unity simulation [KumarRobotics/autonomy_simulation](https://github.com/KumarRobotics/autonomy_simulation), you can run experiments either on real robots or in Unity simulation. 

The following is instruction on how to run Unity simulation.

This readme consists of two parts: (1) [use without docker (i.e. build from source)](https://github.com/KumarRobotics/autonomy_stack/blob/update_docs/README.md#part1-use-without-docker-ie-build-from-source) and (2)[use with docker](https://github.com/KumarRobotics/autonomy_stack/blob/update_docs/README.md#part2-use-with-docker) .

If you are a developer who will make changes to this code stack, (1) is recommended; Otherwise, if you are a user who will NOT make changes to this code stack and is comfortable with docker, (2) is recommended.


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

Firstly, create a new folder (your_code_directory), clone the following repo into it:
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
ln -s ~/path_to_your_code_directory/* .
```
after this, you should be able to see symbolic links of all repos that you just cloned in this folder.

Finally, install dependencies, and build everything:
```
sudo apt install libnlopt-dev
sudo apt install libsdl-image1.2-dev
cd ..
catkin build -DCMAKE_BUILD_TYPE=Release 
```

#### Troubleshooting
If you run into any dependency issues, try installing the corresponding package using apt-get install.

## Step 3: Run the simulation with autonomy stack
Open four terminals, in **each** terminal source arl-unity-ros workspace you just built:
```
source ~/arl-unity-ros/devel/setup.bash
```

Then, following the instructions in **one of the following scenarios** according to your application:

### Scenario 1: Run in DCIST prebuilt environments + use ground truth pose
```
roslaunch dcist_utils full_sim.launch
```
Click motors on after you get commands (i.e., when the commands in rqt GUI window become non-zero).
Wait for several seconds, click “take off” in rqt GUI, and you should see the UAV take off. 

#### Troubleshooting
If the robot does not take off. You don't have to relaunch. Instead, you can try clicking the rqt GUI again in the following order:
```
motors off -> motors on -> take off
```
If the robot still does not take off, kill everything and relaunch.


### Scenario 2 Customize and run in your own environments + use ground truth pose
**PLease go to this link with SEAS account for the manual, download the script file and use the script and Unity Editor according to the instructions. This manual contains: 1. How to install the UnityHub and configure the correct version of Unity Editor and arl-unity-robotics repository, 2. How to customize your Unity simulation environments (finding suitable objects and placing them; saving the dynamic map as static scene) And the script is used for massively placing objects.
https://drive.google.com/drive/folders/1VbmOorFlEIpqCw3hX1xXsYKw7Pj8fMpS?usp=sharing. **

Click "Play" in Unity Editor.

```
roslaunch dcist_utils full_sim_onboard.launch (TODO!)
```



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
#### Troubleshooting 
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

#### Troubleshooting 
* If you see many error messages, go to `dcist-platform-supplemental` folder and open `falcon_lejeune.yml`
* Go to line 123 and change into `sleep 30` (longer time)



# General troubleshooting:
1. If semantic segmentation does not work correctly (giving black colored mask instead of the correct color you specify) in Unity simulator, this is usually because you add multi-shader or multi-material objects to the scene. Try downloading the two files [here](https://drive.google.com/drive/folders/1Rp1OCEIkL-VwSSPR4qVG91J4yWGqYDOD?usp=sharing) and replace the corresponding two files in arl-unity-robotics/Assets/scripts with those two.


