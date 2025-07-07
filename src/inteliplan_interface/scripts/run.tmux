s#!/bin/bash

# Tmux session name
SESSION=inteliplan_experiment
WS_NAME=catkin_ws
# Set this variable in order to source ORIon ROS workspace
WS=/$WS_NAME/devel/setup.bash

CLEAR_PANE='tmux -u send-keys "clear" Enter'
_SRC_ENV="tmux send-keys source Space $WS C-m "

PREFIX="rsource; sim_mode; export GAZEBO_MODEL_PATH=/catkin_ws/src/inteliplan_interface/configs/models:/catkin_ws/src/gazebo_models_worlds_collection/models;"

tmux -2 new-session -d -s $SESSION

tmux rename-window -t $SESSION:0 'robot'
tmux new-window -t $SESSION:1 -n 'vision-manipulation'
tmux new-window -t $SESSION:2 -n 'inteliplan'
tmux new-window -t $SESSION:3 -n 'random_test'

# ######################################################################

tmux select-window -t $SESSION:robot

[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX roslaunch inteliplan_interface simulation.launch"

tmux split-window -h
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX roslaunch anytree_control standalone_arm_control.launch"

tmux split-window -v
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX roslaunch anytree_motion_planner standalone_arm_motion_planner.launch"


# ######################################################################

tmux select-window -t $SESSION:vision-manipulation

[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX roslaunch orion_recognition recognition.launch"


tmux split-window -h
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX roslaunch manipulation manipulation_servers_no_grasp_synthesis.launch"

# ######################################################################

tmux select-window -t $SESSION:inteliplan

[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX rosrun tf static_transform_publisher 0.8 0 0.72 0 0 0 map surface 1"


# ######################################################################

tmux select-window -t $SESSION:random_test

[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX rosrun anytree_manipulation pick_up_client.py apple_0"

tmux split-window -h
[ -f $DEVELOPMENT_WS_ROBOT ] && `$_SRC_ENV` && `$CLEAR_PANE`
tmux send-keys "$PREFIX rostopic pub -1 /z1_gazebo/reset_arm_pose std_msgs/Bool 'data: true"


# ######################################################################

# Set default window
tmux select-window -t $SESSION:robot

# Attach to session
tmux -2 attach-session -t $SESSION