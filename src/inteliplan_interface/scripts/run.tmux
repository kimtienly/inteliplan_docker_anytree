#!/bin/bash

# Tmux session name
SESSION=inteliplan_experiment
WS_NAME=catkin_ws

# Command setup
CLEAR="clear"
PREFIX="rsource; sim_mode; export GAZEBO_MODEL_PATH=/catkin_ws/src/inteliplan_interface/configs/models:/catkin_ws/src/gazebo_models_worlds_collection/models"

# Create tmux session
tmux -2 new-session -d -s $SESSION

# Create and name windows
tmux rename-window -t $SESSION:0 'robot'
tmux new-window -t $SESSION:1 -n 'vision-manipulation'
tmux new-window -t $SESSION:2 -n 'inteliplan'
tmux new-window -t $SESSION:3 -n 'random-test'

# Helper function
run_tmux_command() {
    local pane="$1"
    local command="$2"
    tmux select-pane -t "$pane"
    tmux send-keys "$PREFIX" C-m
    tmux send-keys "$CLEAR" C-m
    tmux send-keys "$command"
}

# ==================== Robot window ====================
tmux select-window -t $SESSION:robot
# run_tmux_command 0 "roslaunch inteliplan_interface simulation.launch"
run_tmux_command 0 "rosrun procman_ros sheriff src/inteliplan_interface/procman_configs/start_simulation_control.pmd --lone-ranger --start-roscore"
tmux split-window -h
run_tmux_command 1 "roslaunch position_controller position_controller.launch"
tmux split-window -v
run_tmux_command 2 "roslaunch anytree_motion_planner standalone_arm_motion_planner.launch"

# ==================== Vision-manipulation ====================
tmux select-window -t $SESSION:vision-manipulation
run_tmux_command 0 "roslaunch orion_recognition recognition.launch"
tmux split-window -h
run_tmux_command 1 "roslaunch anytree_manipulation manipulation_servers.launch"

# ==================== Inteliplan ====================
tmux select-window -t $SESSION:inteliplan
run_tmux_command 0 "rosrun tf static_transform_publisher 0.8 0 0.72 0 0 0 map surface 1"

# ==================== Random test ====================
tmux select-window -t $SESSION:random-test
run_tmux_command 0 "rosrun anytree_manipulation pick_up_client.py apple_0"
tmux split-window -h
run_tmux_command 1 "rostopic pub -1 /z1_gazebo/reset_arm_pose std_msgs/Bool 'data: true'"

# Attach
tmux select-window -t $SESSION:robot
tmux -2 attach-session -t $SESSION
