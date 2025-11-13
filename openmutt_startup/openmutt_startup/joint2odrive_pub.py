# openmutt_startup/joint_state_sub.py
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState

TWOPI = 2.0 * math.pi

class Joint2Odrive(Node):
    def __init__(self):
        super().__init__('joint_to_odrive')

        # ---- Parameters (edit defaults or pass via launch) ----
        self.declare_parameter('axis_indices', list(range(12)))   # which 12 axes exist (0..11)
        self.declare_parameter('motor_map',    list(range(12)))   # joint index -> axis index mapping
        self.declare_parameter('gear_ratio',   [13.0]*12)         # motor_rot / joint_rot
        self.declare_parameter('sign',         [1.0]*12)          # +1 or -1 per axis
        self.declare_parameter('zero_rad',     [0.0]*12)          # joint zero offsets (rad)
        self.declare_parameter('offset_rotation', [0.0]*12)       # constant motor rotation offsets
        self.declare_parameter('namespace_format','/odrive_axis{}/control_message')
        self.declare_parameter('control_mode', 3)                 # POSITION_CONTROL (verify)
        self.declare_parameter('input_mode',   1)                 # PASSTHROUGH (verify)
        
        self.axis_indices = list(self.get_parameter('axis_indices').value)
        self.motor_map    = list(self.get_parameter('motor_map').value)
        self.gear_ratio   = list(self.get_parameter('gear_ratio').value)
        self.sign         = list(self.get_parameter('sign').value)
        self.zero_rad     = list(self.get_parameter('zero_rad').value)
        self.offset_rot   = list(self.get_parameter('offset_rotation').value)
        self.namespace_format = self.get_parameter('namespace_format').value
        self.control_mode = int(self.get_parameter('control_mode').value)
        self.input_mode   = int(self.get_parameter('input_mode').value)
        self.axis_state   = list(self.get_parameter('axis_state').value)

        # ---- Publishers: one per axis ----
        self.pubs = []
        for axis in self.axis_indices:
            topic = self.namespace_format.format(axis)
            pub = self.create_publisher(ControlMessage, topic, 10)
            self.pubs.append(pub)
            self.get_logger().info(f'Publishing to {topic}')

        # ---- Service clients: one per axis ----
        self.cli = {}
        self.axis_state_sent = {}
        for axis in self.axis_indices:
            service = f'/odrive_axis{axis}/request_axis_state'
            self.cli[axis] = self.create_client(AxisState, service)
            self.axis_state_sent[axis] = False
            self.get_logger().info(f'Axis state client for {service}')

        # Timer to attempt setting axis states until all have succeeded
        self.axis_state_timer = self.create_timer(0.5, self._tick_axis_state_requests)

        # ---- Subscription ----
        self.create_subscription(JointState, '/joint_states', self.cb_joint_states, 10)

        self.get_logger().info('JointToOdrive ready.')

    def _tick_axis_state_requests(self):
        """Periodically try to send axis_state=... to each axis when the service is ready."""
        all_done = True

        for axis in self.axis_indices:
            if self.axis_state_sent[axis]:
                continue  # already sent successfully

            client = self.cli[axis]
            if not client.service_is_ready():
                all_done = False
                self.get_logger().debug(f'Axis {axis} service not ready yet')
                continue

            req = AxisState.Request()
            # index into axis_state param by axis (or clamp if needed)
            state_code = self.axis_state[axis] if axis < len(self.axis_state) else 8
            req.axis_requested_state = int(state_code)

            future = client.call_async(req)
            # capture axis value in default arg
            future.add_done_callback(lambda fut, ax=axis: self._on_axis_state_response(fut, ax))

            self.axis_state_sent[axis] = True
            self.get_logger().info(f'Sent axis_state={state_code} to axis {axis}')

        # If everyone got a request, stop the timer
        for axis in self.axis_indices:
            if not self.axis_state_sent[axis]:
                all_done = False
                break

        if all_done:
            self.axis_state_timer.cancel()
            self.get_logger().info('All axis_state requests sent; stopping timer.')

    def cb_joint_states(self, js: JointState):
        # Safety guard
        if not js.position:
            return

        # For each joint index j, find its target axis k
        for j, k in enumerate(self.motor_map):
            if j >= len(js.position):
                break

            # Convert radians -> rotations of the motor
            q = float(js.position[j])
            rot = self.sign[k] * ((q - self.zero_rad[k]) * (self.gear_ratio[k] / TWOPI)) + self.offset_rot[k]

            msg = ControlMessage()
            msg.control_mode = self.control_mode
            msg.input_mode   = self.input_mode
            msg.input_pos    = float(rot)

            # Assuming motor_map entries line up with pub indices
            self.pubs[k].publish(msg)

    def _on_axis_state_response(self, future, axis):
        try:
            res = future.result()
            self.get_logger().info(f'Axis {axis} state response: {res}')
        except Exception as e:
            self.get_logger().warn(f'Axis {axis} state call failed: {e}')
            # Mark as not sent so timer can retry
            self.axis_state_sent[axis] = False


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
