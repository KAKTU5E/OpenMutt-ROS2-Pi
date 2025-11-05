from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    can0_axes = [0, 1, 2]

    can0_nodes = []

    for i in can0_axes:
        can0_nodes.append(    
            Node(
                package = 'odrive_can',
                namespace = f'odrive_axis{i}',
                executable = 'odrive_can_node',
                name = 'can_node',
                parameters = [{
                    'node_id' : i,
                    'interface' : 'can0'
                        }]
                )
            )


    motor_publisher = Node(
        package = 'openmutt_startup', 
        namespace = 'quarter_joint2odrive_pub',
        executable = 'quarter_joint2odrive_pub',
        name = 'quarter_model_publisher',
    )

    

    return LaunchDescription([can0_nodes, motor_publisher])