Steps to run:

copy FloodedGrounds.unity into arl-unity-robotics Unity project assets folder, i.e., arl-unity-robotics/Assets

start the simulation from Unity Editor (detail refer to readme in the autonomy stack repo)

source arl-unity-ros workspace

launch sim_quad_onboard_rgbd_visualize_3d_detections.launch in this repo instead of sim_quad.launch in arl-unity-ros

launch dcist_utils system_vis_car_detection.launch use the following command:
```roslaunch dcist_utils system_vis_car_detection.launch```

do the rest as usual (launch client_launch client.launch, rosrun takeoff.bash, click motors on, etc.)

run object_localization.py

in rviz, add and visualize the car bounding box MarkerArray topic ("/quadrotor/car_bbox")
