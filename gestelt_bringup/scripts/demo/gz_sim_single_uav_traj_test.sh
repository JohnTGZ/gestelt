#!/bin/bash

SESSION="gz_sim_single_uav"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
gestelt_bringup_DIR="$SCRIPT_DIR/.."
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../../PX4-Autopilot"

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../../../devel/setup.bash &&
"
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic &&
"

#####
# Commands
#####
CMD_0="roslaunch gestelt_bringup gazebo.launch world_name:=$SCRIPT_DIR/../simulation/worlds/empty.world "

CMD_1="
roslaunch gestelt_bringup single_uav_sim.launch drone_id:=0 init_x:=0 init_y:=0
"

CMD_2="
roslaunch gestelt_bringup sitl_central.launch rviz_config:=gz_sim cloud_topic_downsample_in:=camera/depth/points_fake
"


CMD_3="
roslaunch gestelt_bringup trajectory_server.launch
"

CMD_4="roslaunch gestelt_bringup square_mission.launch"

CMD_5="roslaunch gestelt_bringup traj_tracking.launch"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    tmux split-window -t $SESSION:0.3 -h
    tmux split-window -t $SESSION:0.2 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $CMD_0" C-m 
    sleep 3
    tmux send-keys -t $SESSION:0.1 "$SOURCE_PX4_AUTOPILOT $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
    tmux send-keys -t $SESSION:0.4 "$SOURCE_WS $CMD_4" C-m
    tmux send-keys -t $SESSION:0.5 "$SOURCE_WS $CMD_5" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
