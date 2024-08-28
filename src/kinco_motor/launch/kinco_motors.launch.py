from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'kinco_motor',
            namespace = 'expansion',
            executable = 'kinco_motor_node',
            name = 'expansion_motor',
            parameters = [
                {"node_id": 1},
            ],
            respawn = True,
            respawn_delay = 3,
        ),
        Node(
            package = 'kinco_motor',
            namespace = 'push_pull',
            executable = 'kinco_motor_node',
            name = 'push_pull_motor',
            parameters = [
                {"node_id": 2},
            ],
            respawn = True,
            respawn_delay = 3,
        ),
        Node(
            package = 'kinco_motor',
            namespace = 'rotation',
            executable = 'kinco_motor_node',
            name = 'rotation_motor',
            parameters = [
                {"node_id": 3},
            ],
            respawn = True,
            respawn_delay = 3,
        ),
    ])