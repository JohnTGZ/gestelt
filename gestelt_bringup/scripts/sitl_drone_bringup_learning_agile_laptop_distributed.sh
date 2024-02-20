#!/bin/bash

SESSION="gz_sim_single_uav"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
gestelt_bringup_DIR="$SCRIPT_DIR/.."
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../PX4-Autopilot"

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../../devel/setup.bash &&
"
# export ROS_MASTER_URI (for distributed simulation)
# drone's side ROS_MASTER_URI should be the laptop
EXPORT_ROS_MASTER_URI="
export ROS_IP=172.20.10.4 
export ROS_HOSTNAME=172.20.10.4 &&
export ROS_MASTER_URI=http://172.20.10.4:11311
"
# PX4 v1.13.0
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/sitl_gazebo &&
"
#####
# Commands
#####
# Start Gazebo and PX4 SITL instances
CMD_0="
roslaunch gestelt_bringup sitl_drone.launch 
"

# Start up drone commander (Handles taking off, execution of mission and landing etc.)
CMD_1="
roslaunch trajectory_server trajectory_server_node.launch rviz_config:=gz_sim
"

# Start up minimum snap trajectory planner and sampler 
CMD_2="
rosbag record -o ~/gestelt_ws/src/gestelt/gestelt_bringup/data/ /learning_agile_agent/callback_runtime
"

# Start up script to send commands
CMD_3="roslaunch gestelt_bringup learning_agile_mission.launch platform:='drone' LAUNCH_DRONE_NODE:=true"

# disarm drone
# CMD_4="rosservice call /drone_commander/disarm"
# CMD_4="rosrun mavros mavparam set COM_RCL_EXCEPT 4"
if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $CMD_0" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS " #C-m 
    sleep 10
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
