To change planner, change `tracker_params_mp.yaml`, to run planner quickly without running quad, set `use_tracker_client` to false


- one terminal run :

```
roslaunch map_plan_launch run_in_sim.launch 
```


- one terminal run:


```
rosrun rqt_mav_manager rqt_mav_manager

```

click "motors on" and "take off"



- one terminal run: 

```
rosrun action_planner evaluate_traj_exp.py
```


To change map back to image one, see `run_in_sim.launch `


For experiment, directly run `run_in_exp.launch ` to change quadrotor name
