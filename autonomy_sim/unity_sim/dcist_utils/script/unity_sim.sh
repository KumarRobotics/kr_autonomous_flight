#!/bin/bash

# Performs the same functions as ros2 launch dcist_utils full_sim.launch.py, 
# but separates out the launches in tmux for more convenient usage.

MAV_NAME=quadrotor
ODOM_TOPIC=/unity_command/ground_truth/${MAV_NAME}/odom
MIN_DISPERSION_PLANNER=false

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=sim_unity

CURRENT_DISPLAY=${DISPLAY}
if [ -z "${DISPLAY}" ]; then
  echo "DISPLAY is not set"
  CURRENT_DISPLAY=:0
fi

if [ -z "${TMUX}" ]; then
  tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Make mouse useful in copy mode
tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Core/Client"
tmux send-keys -t $SESSION_NAME "echo "ROS2: no roscore needed"" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; export DISPLAY=${CURRENT_DISPLAY}; ros2 launch client_launch client.launch.py robot:=${MAV_NAME}" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "Sim"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; export DISPLAY=${CURRENT_DISPLAY}; ros2 launch dcist_utils sim_quad.launch.py robot:=${MAV_NAME} odom:=${ODOM_TOPIC}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 15; export DISPLAY=${CURRENT_DISPLAY}; export ROS_NAMESPACE=${MAV_NAME}; ros2 run arl_unity_ros_air rosflight_offboard --ros-args -r __node:=robot" Enter
tmux select-layout -t $SESSION_NAME tiled

tmux new-window -t $SESSION_NAME -n "SM/Planner"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; export DISPLAY=${CURRENT_DISPLAY}; ros2 launch state_machine_launch system_mp.launch.py robot:=${MAV_NAME} min_dispersion_planner:=${MIN_DISPERSION_PLANNER}" Enter
if ${MIN_DISPERSION_PLANNER}; then
  tmux split-window -t $SESSION_NAME
  tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 1; export DISPLAY=${CURRENT_DISPLAY}; ros2 launch motion_primitives cpp_action_server.launch.py robot:=${MAV_NAME}" Enter
fi
tmux select-layout -t $SESSION_NAME tiled

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
