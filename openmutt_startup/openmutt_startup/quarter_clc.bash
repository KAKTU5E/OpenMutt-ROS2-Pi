#!/usr/bin/bash

ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}"
ros2 service call /odrive_axis1/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}"
ros2 service call /odrive_axis2/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 3}"

ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"
ros2 service call /odrive_axis1/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"
ros2 service call /odrive_axis2/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"