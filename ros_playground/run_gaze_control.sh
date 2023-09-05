#!/bin/bash

# Define cmds
source_cmd="source install/setup.bash"
run_gaze_node="ros2 run gazebaes_package gazebaes_node"
run_decision_node="ros2 run motor_control_package decision_node"
run_motor_cmd_subs="ros2 run motor_control_package motor_cmd_subscriber"

colcon build

# Open the first terminal and run the first command
gnome-terminal --geometry=80x12+0 -- bash -c "$source_cmd; $run_gaze_node; exec bash"

# Open the second terminal and run the second command
gnome-terminal --geometry=80x12+80 -- bash -c "$source_cmd; $run_decision_node; exec bash"

# Open the third terminal and run the third command
gnome-terminal --geometry=80x12+160 -- bash -c "$source_cmd; $run_motor_cmd_subs; exec bash"
