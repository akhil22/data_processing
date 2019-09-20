#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import csv

dynamics_data = open('warthog_dynamics_data.csv', 'a')
field_names = ['v_c','w_c','v', 'w','v_out', 'w_out', 'dt']
data_writer = csv.DictWriter(dynamics_data,field_names)
data_writer.writeheader()
prev_vel = 0
prev_omega = 0
prev_vel_cmd = 0
prev_omega_cmd = 0
got_first_msg = False
prev_time = 0
def fill_prev_data(cmdvel_msg, odom_msg):
    global got_first_msg, prev_vel, prev_omega, prev_vel_cmd, prev_omega_cmd, prev_time
    prev_vel = odom_msg.twist.twist.linear.x
    prev_omega = odom_msg.twist.twist.angular.z
    prev_vel_cmd = cmdvel_msg.linear.x
    prev_omega_cmd = cmdvel_msg.angular.z
    prev_time = odom_msg.header.stamp

def callback(cmdvel_msg, odom_msg):
    global got_first_msg, prev_vel, prev_omega, prev_vel_cmd, prev_omega_cmd, prev_time
    if not got_first_msg:
        got_first_msg = True
        fill_prev_data(cmdvel_msg, odom_msg)
        return
    else:
       #print((steer_msg.steering_wheel_angle - prev_steering)/del_t)
       #prev_steering = steer_msg.steering_wheel_angle
       v_out = odom_msg.twist.twist.linear.x
       w_out = odom_msg.twist.twist.angular.z
       dt = (odom_msg.header.stamp - prev_time).to_sec()
       data_writer.writerow({'v_c': prev_vel_cmd, 'w_c': prev_omega_cmd, 'v': prev_vel, 'w': prev_omega, 'v_out': v_out, 'w_out': w_out, 'dt': dt})
       fill_prev_data(cmdvel_msg, odom_msg)
       print("filled_data")



cmdvel_sub = message_filters.Subscriber('/warthog_velocity_controller/cmd_vel', Twist)
odom_sub= message_filters.Subscriber('/warthog_velocity_controller/odom', Odometry)

ts = message_filters.ApproximateTimeSynchronizer([cmdvel_sub, odom_sub] , 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.init_node('data_synchronizer')
rospy.spin()
