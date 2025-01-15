#!/bin/bash
# Starts up the turtlebot house sim

SESSION=turtlebot_house_sim
POLICY_MODE=$1
MONGO_CONNECTION_STRING=$2
WIRE_FILE=$3
WIRE_FILE_INDEX=$4


tmux -2 new-session -d -s $SESSION
tmux rename-window -t $SESSION 'gazebo'
tmux new-window -t $SESSION -n 'navigation'
tmux new-window -t $SESSION -n 'wire-manager'
tmux new-window -t $SESSION -n 'policy-executor'

tmux select-window -t $SESSION:gazebo
tmux send-keys "ros2 launch turtlebot_house_sim turtlebot_house.launch.py" C-m

tmux select-window -t $SESSION:navigation
tmux send-keys "ros2 launch turtlebot_house_sim navigator.launch.py" C-m

tmux select-window -t $SESSION:wire-manager
if [ -z $WIRE_FILE ] && [ -z $WIRE_FILE_INDEX ] # If both not set, don't pass in
then
    tmux send-keys "ros2 launch turtlebot_house_sim wire.launch.py" C-m
else
    tmux send-keys "ros2 launch turtlebot_house_sim wire.launch.py wire_status_file:=$WIRE_FILE wire_status_index:=$WIRE_FILE_INDEX" C-m
fi

tmux select-window -t $SESSION:policy-executor
tmux send-keys "ros2 launch turtlebot_house_sim policy_executor.launch.py db_collection:=house-$POLICY_MODE mode:=$POLICY_MODE db_connection_string:=$MONGO_CONNECTION_STRING" C-m