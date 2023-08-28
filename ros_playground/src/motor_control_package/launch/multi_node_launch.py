import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='motor_control_package',
            executable='decision_node',
            name='decision_node'
        ),
        Node(
            package='motor_control_package',
            executable='motor_cmd_subscriber',
            name='motor_cmd_subscriber'
        )
    ])
