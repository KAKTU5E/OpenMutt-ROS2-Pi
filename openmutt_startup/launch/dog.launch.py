from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    
    can0_axes = [0, 1, 2, 3, 4, 5]
    can1_axes = [6, 7, 8, 9, 10, 11]

    can0_nodes = []
    can1_nodes = []

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
        
    for j in can1_axes:
        can1_nodes.append(    
            Node(
                package = 'odrive_can',
                namespace = f'odrive_axis{j}',
                executable = 'odrive_can_node',
                name = 'can_node',
                parameters = [{
                    'node_id' : j,
                    'interface' : 'can1'
                    }]
                )
            )
        
    joint_subscriber = Node(
        package='openmutt_startup',
        namespace='joint_sub',
        executable='quarter_joint2odrive',
        name='quarter_joint2odrive',
        parameters=[{
            'motor_map':  [0,1,2,3,4,5,6,7,8,9,10,11],
            'gear_ratio': [13.0]*12,
            'sign':       [1.0]*12,
            'control_mode': 3,
            'input_mode':   1,
            'namespace_format':'/odrive_axis{}/control'
        }]
    )

    

    return LaunchDescription([can0_nodes, can1_nodes, joint_subscriber])