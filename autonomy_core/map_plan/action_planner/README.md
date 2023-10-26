# Action Planner Package
[See more in the private documentation](https://github.com/KumarRobotics/Documentation/tree/master/kr_autonomous_flight/autonomy_core/map_plan/action_planner)
## Maintainer: 
[Laura](mailto:laurajar@seas.upenn.edu)

## Overview
This package bring together different planners into one place and launches a [action server](http://wiki.ros.org/actionlib) for motion planning. 

## Slides
Add a link to the slides here

## Installation
See the kr_autonomous_flight for installation instructions, some of the dependencies are from Laura's fork

## Running
`autonomy_core/map_plan/map_plan_launch/launch/planner_standalone.launch` launches the planner by itself. It will launch the action server and a rviz node for visualization. Additionally `rosrun action_planner publish_plantwopointaction.py` can be used to publish a start and goal to the action server.

JPS and GCOPTER running togeher can crash.


## Adjusting Configuration
Most configs can be adjusted in autonomy_core/control/control_launch/config/tracker_params_mp.yaml
The most important parameters are:
* `search_planner_type`: The search planner to use that mostly rely on search to find a path. 0 [MPL](https://github.com/sikang/motion_primitive_library), 1: [dispersion planner](https://github.com/ljarin/dispersion_motion_planning), 2: [JPS (geometric)](https://github.com/KumarRobotics/jps3d)
* `opt_planner_type`: 0: [Double Description](https://github.com/ZJU-FAST-Lab/large_scale_traj_optimizer) 1: [gcopter](https://github.com/yuwei-wu/GCOPTER)

Many of the planners are from outside the lab, if that is the case, we usually fork the repo and do development on there, and add as a depency to this package using [vcs](https://github.com/dirk-thomas/vcstool). 




