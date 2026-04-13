


Install dependencies:

$ sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool ros-jazzy-tf2-sensor-msgs ros-jazzy-twist-mux \
   ros-jazzy-vision-msgs python3-yaml python3-pycryptodome python3-gnupg libsuitesparse-dev libv4l-dev libceres-dev \
   ros-jazzy-random-numbers ros-jazzy-mavros-msgs libsdl-dev libsdl-image1.2-dev

$ sudo apt-get update -y

$ sudo apt-get install libpcl-dev -y

$ sudo apt-get install -y libeigen3-dev libtbb-dev libgtest-dev

Clone Repo: (Make sure you have ssh key setup)

git clone git@github.com:ljarin/kr_autonomous_flight.git

git checkout ros2_dev

vcs import < external_all.yaml

vcs import < motion_primitives/deps_ssh.repos # (or deps_https.ssh)

rosdep install --from-paths src --ignore-src -r -y

sudo apt install libspdlog-dev

sudo apt-get install ros-jazzy-ompl

sudo apt install python3-pcl

pip3 install pandas tqdm

give path to the json file in tracker_params_mp.yaml : dispersion/graph_file: FILL IN PATH

To change planner, change `tracker_params_mp.yaml`, to run planner quickly without running quad, set `use_tracker_client` to false


- one terminal run :

```
ros2 launch map_plan_launch run_in_sim.launch.py
```


- one terminal run:


```
ros2 run rqt_mav_manager rqt_mav_manager

```

click "motors on" and "take off"



- one terminal run:

```
ros2 run action_planner evaluate_traj_exp.py
```


To change map back to image one, see `run_in_sim.launch.py`


For experiment, directly run `run_in_exp.launch.py` to change quadrotor name
