# openmutt_startup/joint_state_sub.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from odrive_can.msg import ControlMessage  # from your CLI output

TWOPI = 2.0 * math.pi

class Quarter_Joint2Odrive(Node):
    def __init__(self):
        super().__init__('joint_to_odrive')

        # ---- Parameters (edit defaults or pass via launch) ----
        self.declare_parameter('axis_indices', list(range(3)))   # which 12 axes exist (0..11)
        self.declare_parameter('motor_map',    list(range(3)))   # joint index -> axis index mapping
        self.declare_parameter('gear_ratio',   [13.0]*3)          # motor_rot / joint_rot
        self.declare_parameter('sign',         [1.0]*3)          # +1 or -1 per axis
        self.declare_parameter('zero_rad',     [0.0]*3)          # joint zero offsets (rad)
        self.declare_parameter('offset_rot',   [0.0]*3)          # constant motor rotation offsets
        self.declare_parameter('namespace_format','/odrive_axis{}/control')
        self.declare_parameter('control_mode', 3)                 # POSITION_CONTROL (verify)
        self.declare_parameter('input_mode',   1)                 # PASSTHROUGH (verify)

        self.axis_indices = list(self.get_parameter('axis_indices').value)
        self.motor_map    = list(self.get_parameter('motor_map').value)
        self.gear_ratio   = list(self.get_parameter('gear_ratio').value)
        self.sign         = list(self.get_parameter('sign').value)
        self.zero_rad     = list(self.get_parameter('zero_rad').value)
        self.offset_rot   = list(self.get_parameter('offset_rot').value)
        self.namespace_format = self.get_parameter('namespace_format').value
        self.ctrl_mode    = int(self.get_parameter('control_mode').value)
        self.in_mode      = int(self.get_parameter('input_mode').value)

        # Publishers: one per axis
        self.pubs = []
        for axis in self.axis_indices:
            topic = self.namespace_format.format(axis)
            self.pubs.append(self.create_publisher(ControlMessage, topic, 10))
            self.get_logger().info(f'Publishing to {topic}')

        # Subscription
        self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)

        self.get_logger().info('Quarter_JointToOdrive ready.')

    def cb_joint_states(self, js: JointState):
        # Safety guard
        if not js.position:
            return

        # For each joint index j, find its target axis k
        for j, k in enumerate(self.motor_map):
            if j >= len(js.position):
                break

            # Convert radians -> rotations
            q = float(js.position[j])
            rot = self.sign[k] * ((q - self.zero_rad[k]) * (self.gear_ratio[k] / TWOPI)) + self.offset_rot[k]

            msg = ControlMessage()
            msg.control_mode = self.ctrl_mode
            msg.input_mode   = self.in_mode
            msg.input_pos    = float(rot)
            # leave vel/torque unset (0.0) in position mode
            self.pubs[k].publish(msg)

def main():
    rclpy.init()
    node = Quarter_Joint2Odrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
