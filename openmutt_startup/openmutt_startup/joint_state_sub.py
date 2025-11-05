import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from std_srvs.srv import Empty
from ros_odrive.odrive_node.msg import ControlMessage, ControllerStatus, ODriveStatus
import numpy as np


class AxisPublisher(Node): 
    def __init__(self):
        super().__init__('axis_publisher')
        self.declare_parameter('motor_remap', [0,1,2,3,4,5,6,7,8,9,10,11,12])
        self.motor_order = self.get_parameter('motor_remap').get_parameter_value().integer_array_value
        
        self.joint_zeros = [0.0]*12
        self.current_pos = [0.0]*12

        self.enable = False

        self.state_sub = self.create_subscription(

            JointState,
            '/joint_states',
            self.state_listener_callback,
            10
        )

        self.command_pub = self.create_publisher

def main(args=None):
    rclpy.init(args=args)

    #node = 

    #try:
    #    rclpy.spin(node)
    #except KeyboardInterrupt:
    #    pass
    #finally:
    #    node.destroy_node()
    #    rclpy.shutdown()

    return None    

if __name__ == "__main__":
    main()