#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from dbw_mkz_msgs.msg import SteeringReport, ThrottleReport, BrakeReport

import csv

dynamics_data = open('gazebo_dynamics_data.csv', 'w')
field_names = ['vx','pedal_c','brake_c','steer_val','accel','steer_vel_out','steer_cmd','v_out','omega_out','accel_out']
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
def fill_prev_data(steer_msg, pedal_command_msg, vel_msg, brake_cmd_msg, accel_msg):
    global got_first_msg, prev_steering, prev_vel, prev_pedal_cmd, prev_steering_cmd, prev_brake_cmd, prev_accel
    prev_steering = steer_msg.steering_wheel_angle
    prev_vel = vel_msg.twist.linear.x
    prev_pedal_cmd = pedal_command_msg.pedal_cmd
    prev_steering_cmd = steer_msg.steering_wheel_angle_cmd
    prev_brake_cmd = brake_cmd_msg.pedal_cmd
    prev_accel = accel_msg.data

def callback(vel_msg, pedal_command_msg, accel_msg, steer_msg, brake_cmd_msg):
    global got_first_msg, prev_steering, prev_vel, prev_vel_cmd, prev_steering_cmd, del_t, prev_accel
    if not got_first_msg:
        got_first_msg = True
        fill_prev_data(steer_msg, pedal_command_msg,  vel_msg, brake_cmd_msg, accel_msg)
        return
    else:
       #print((steer_msg.steering_wheel_angle - prev_steering)/del_t)
       #prev_steering = steer_msg.steering_wheel_angle
       data_writer.writerow({'vx': prev_vel, 'pedal_c': prev_pedal_cmd, 'brake_c': prev_brake_cmd, 'steer_val': prev_steering,'accel':prev_accel, 'steer_vel_out': (steer_msg.steering_wheel_angle - prev_steering)/del_t, 'steer_cmd': prev_steering_cmd,'v_out': vel_msg.twist.linear.x, 'omega_out': vel_msg.twist.angular.z, 'accel_out': accel_msg.data})
       fill_prev_data(steer_msg, pedal_command_msg, vel_msg, brake_cmd_msg, accel_msg)
       print("filled_data")



vel_sub = message_filters.Subscriber('/vehicle/twist', TwistStamped)
pedal_command_sub= message_filters.Subscriber('/vehicle/throttle_report', ThrottleReport)
accel_sub= message_filters.Subscriber('/vehicle/filtered_accel', Float64)
steer_sub= message_filters.Subscriber('/vehicle/steering_report', SteeringReport)
brake_sub= message_filters.Subscriber('/vehicle/brake_report', BrakeReport)

ts = message_filters.ApproximateTimeSynchronizer([vel_sub, pedal_command_sub, accel_sub, steer_sub, brake_sub] , 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.init_node('data_synchronizer')
rospy.spin()
