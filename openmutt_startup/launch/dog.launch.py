from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    can0_axes = [0,1,2,3,4,5]
    can1_axes = [6,7,8,9,10,11]

    can0_nodes = [
        Node(
            package='odrive_can',
            namespace=f'odrive_axis{i}',
            executable='odrive_can_node',
            name='can_node',
            parameters=[{'node_id': i, 'interface': 'can0'}]
        )
        for i in can0_axes
    ]

    can1_nodes = [
        Node(
            package='odrive_can',
            namespace=f'odrive_axis{j}',
            executable='odrive_can_node',
            name='can_node',
            parameters=[{'node_id': j, 'interface': 'can1'}]
        )
        for j in can1_axes
    ]

    odrive_publisher = Node(
        package='openmutt_startup',
        namespace='joint_sub',
        executable='joint2odrive_pub',  # make sure console_scripts matches
        name='quarter_joint2odrive',
    
        )

    # FLAT list — use unpacking so there’s zero chance of nesting mistakes
    return LaunchDescription([*can0_nodes, *can1_nodes, odrive_publisher])
