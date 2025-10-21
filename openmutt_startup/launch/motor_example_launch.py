from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package = 'odrive_can',
            namespace = 'odrive_axis0',
            executable = 'odrive_can_node',
            name = 'can_node',
            parameters = [
                {'node_id' , 0},
                {'interface' , 'can0'}
            ]
        ),

        Node(
            package = 'openmutt_startup', 
            namespace = 'joint_sub1',
            executable = 'joint_state_sub',
            name = 'joint_sub_node',
        )

    ])

