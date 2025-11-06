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


    joint_subscriber = Node(
        package='openmutt_startup',
        namespace='joint_sub',
        executable='quarter_joint2odrive',
        name='quarter_joint2odrive',
        parameters=[{
            'motor_map':  [0,1,2],
            'gear_ratio': [13.0]*3,
            'sign':       [1.0]*3,
            'control_mode': 3,
            'input_mode':   1,
            'namespace_fmt':'/odrive_axis{}/control',
        }],
)


    

    return LaunchDescription([can0_nodes, joint_subscriber])