# openmutt_startup/joint_state_sub.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
import time


TWOPI = 2.0 * math.pi

class Joint2Odrive(Node):
    def __init__(self):
        super().__init__('joint_to_odrive')

        # ---- Parameters (edit defaults or pass via launch) ----
        self.declare_parameter('axis_indices', list(range(12)))   # which 12 axes exist (0..11)
        self.declare_parameter('motor_map',    list(range(12)))   # joint index -> axis index mapping
        self.declare_parameter('gear_ratio',   [13.0]*12)          # motor_rot / joint_rot
        self.declare_parameter('sign',         [1.0]*12)          # +1 or -1 per axis
        self.declare_parameter('zero_rad',     [0.0]*12)          # joint zero offsets (rad)
        self.declare_parameter('offset_rotation',   [0.0]*12)          # constant motor rotation offsets
        self.declare_parameter('namespace_format','/odrive_axis{}/control_message')
        self.declare_parameter('control_mode', 3)                 # POSITION_CONTROL (verify)
        self.declare_parameter('input_mode',   1)                 # PASSTHROUGH (verify)
        self.declare_parameter('axis_state', [8]*12)              # sets all 12 motors to closed loop control mode

        self.axis_indices = list(self.get_parameter('axis_indices').value)
        self.motor_map    = list(self.get_parameter('motor_map').value)
        self.gear_ratio   = list(self.get_parameter('gear_ratio').value)
        self.sign         = list(self.get_parameter('sign').value)
        self.zero_rad     = list(self.get_parameter('zero_rad').value)
        self.offset_rot   = list(self.get_parameter('offset_rotation').value)
        self.namespace_format = self.get_parameter('namespace_format').value
        self.control_mode    = int(self.get_parameter('control_mode').value)
        self.input_mode      = int(self.get_parameter('input_mode').value)
        self.axis_state   = list(self.get_parameter('axis_state').value)

        # Publishers: one per axis

        self.pubs = []
        for axis in self.axis_indices:
            topic = self.namespace_format.format(axis)
            self.pubs.append(self.create_publisher(ControlMessage, topic, 10))
            self.get_logger().info(f'Publishing to {topic}')

        # create a client per axis so we can call services without recreating clients each time

        
        # Subscription
        self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)

        self.get_logger().info('JointToOdrive ready.')

    def cb_joint_states(self, js: JointState):
        # Safety guard
        if not js.position:
            return
        else:
            
            # For each joint index j, find its target axis k
            for j, k in enumerate(self.motor_map):
                if j >= len(js.position):
                    break

                # Convert radians -> rotations
                q = float(js.position[j])
                rot = self.sign[k] * ((q - self.zero_rad[k]) * (self.gear_ratio[k] / TWOPI)) + self.offset_rot[k]

                msg = ControlMessage()
                msg.control_mode = self.control_mode
                msg.input_mode   = self.input_mode
                msg.input_pos    = float(rot)
                # leave vel/torque unset (0.0) in position mode
                self.pubs[k].publish(msg)
            
                for axis in self.axis_indices:
                    service = f'/odrive_axis{axis}/request_axis_state'
                    self.srvs[axis] = self.create_client(AxisState, service)
                    self.get_logger().info(f'Axis state client for {service}')

                # send axis state request asynchronously (non-blocking)
                # build request and call the client previously created
                if k in self.srvs and self.srvs[k].service_is_ready():
                    
                    req = AxisState.Request()
                    req.axis_requested_state = 8
                    future = self.srvs[k].call_async(req)
                    # optional: add a callback to handle the response
                    future.add_done_callback(lambda fut, axis=k: self._on_axis_state_response(fut, axis))

                
        return future

    def _on_axis_state_response(self, future, axis):
        try:
            res = future.result()
            self.get_logger().info(f'Axis {axis} state response: {res}')
        except Exception as e:
            self.get_logger().warn(f'Axis {axis} state call failed: {e}')
        

def main():

    rclpy.init()
    node = Joint2Odrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
