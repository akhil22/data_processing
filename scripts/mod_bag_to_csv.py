#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float64
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, SystemRptFloat, VehicleSpeedRpt

import csv

dynamics_data = open('mod_dynamics_data.csv', 'w')
field_names = ['vx','pedal_c','brake_c', 'v_out']
data_writer = csv.DictWriter(dynamics_data,field_names)
data_writer.writeheader()
prev_steering = 0
prev_vel = 0
prev_pedal_cmd = 0
prev_brake_cmd = 0
prev_steering_cmd = 0
prev_accel = 0
del_t = 0.02
got_first_msg = False
def fill_prev_data(vel_msg, pedal_command_msg,brake_cmd_msg):
    global got_first_msg, prev_vel, prev_pedal_cmd, prev_brake_cmd
    prev_vel = vel_msg.vehicle_speed
    prev_pedal_cmd = pedal_command_msg.command
    prev_brake_cmd = brake_cmd_msg.command

def callback(vel_msg, pedal_command_msg, brake_cmd_msg):
    global got_first_msg, prev_vel, prev_pedal_cmd, prev_brake_cmd
    if not got_first_msg:
        got_first_msg = True
        fill_prev_data(vel_msg, pedal_command_msg, brake_cmd_msg)
        return
    else:
       #print((steer_msg.steering_wheel_angle - prev_steering)/del_t)
       #prev_steering = steer_msg.steering_wheel_angle
       data_writer.writerow({'vx': prev_vel, 'pedal_c': prev_pedal_cmd, 'brake_c': prev_brake_cmd, 'v_out': vel_msg.vehicle_speed})
       fill_prev_data(vel_msg, pedal_command_msg, brake_cmd_msg)
       print("filled_data")



vel_sub = message_filters.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt)
pedal_command_sub= message_filters.Subscriber('/pacmod/parsed_tx/accel_rpt', SystemRptFloat)
brake_sub= message_filters.Subscriber('/pacmod/parsed_tx/brake_rpt', SystemRptFloat)

ts = message_filters.ApproximateTimeSynchronizer([vel_sub, pedal_command_sub, brake_sub] , 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.init_node('data_synchronizer')
rospy.spin()
