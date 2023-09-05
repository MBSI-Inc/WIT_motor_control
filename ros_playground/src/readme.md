## Dependencies

This package requires the following Python dependencies:

- subprocess




## folder structure:
.
├── bci_package
├── gazebaes_package
├── motor_control_package
└── readme.md


bci_package: 
    bci_node: 
        call BCI API and send msg to error_potential and motor_imagery topics

gazebaes_package: 
    gaze_node: 
        call GAZE API and send msg to gaze_tracking topic

motor_control_package: 
    Decision Node: 
        listens to error_potential, motor_imagery and gaze_tracking topics.
        Upon receiving new messages, decide how to move the wheelchair and publish msg to motor_cmd topic
    motor_cmd_subscriber_node:
        listens to motor_cmd topic. Upon receiving new message, translate to motor cmd and send through serial port


## How to run each package:
Open a new terminal, nagivate to project root dir, run the following script:
    colcon build 
    source install/setup.bash
    ros2 run <package_name> <node_name>
    e.g.:
        ros2 run motor_control_package decision_node
        ros2 run motor_control_package motor_cmd_subscriber
        ros2 run gazebaes_package gazebaes_node 
        ros2 run bci_package bci_node
    
A launch file is provided to launch decision_node and motor_cmd_subscriber in one line:
    ros2 launch motor_control_package multi_node_launch.py
