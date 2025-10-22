import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node): # create Joint_State_Subscriber class based on the class, Node

    def __init__(self):
        super().__init__('joint_state_sub') 

        self.state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.state_listener_callback,
            10
        )
        self.state_sub

    def subscriber_callback(self, msg): 
        self.get_logger().info(f"Received\n{msg}") 


def main(args=None):
    rclpy.init(args=args)

    node = JointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return None    
        
if __name__ == "__main__":
    main()