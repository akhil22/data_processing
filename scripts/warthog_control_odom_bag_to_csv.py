#!/usr/bin/env python
import rospy
import message_filters
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import csv

dynamics_data = open('warthog_dynamics_data_odom.csv', 'a')
field_names = ['v_c','w_c','v', 'w','v_out', 'w_out', 'dt', 'dt_msg']
data_writer = csv.DictWriter(dynamics_data,field_names)
data_writer.writeheader()
prev_vel = 0
prev_omega = 0
prev_vel_cmd = 0
prev_omega_cmd = 0
got_first_msg = False
prev_time = 0
msg_num = 0
def fill_prev_data(cmdvel_msg, odom_msg):
    global got_first_msg, prev_vel, prev_omega, prev_vel_cmd, prev_omega_cmd, prev_time
    prev_vel = odom_msg.twist.twist.linear.x
    prev_omega = odom_msg.twist.twist.angular.z
    prev_vel_cmd = cmdvel_msg.twist.twist.linear.x
    prev_omega_cmd = cmdvel_msg.twist.twist.angular.z
    prev_time = odom_msg.header.stamp

def callback(cmdvel_msg, odom_msg):
    global got_first_msg, prev_vel, prev_omega, prev_vel_cmd, prev_omega_cmd, prev_time, msg_num
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
       dt_msg = (odom_msg.header.stamp - cmdvel_msg.header.stamp).to_sec()
       data_writer.writerow({'v_c': prev_vel_cmd, 'w_c': prev_omega_cmd, 'v': prev_vel, 'w': prev_omega, 'v_out': v_out, 'w_out': w_out, 'dt': dt, 'dt_msg': dt_msg})
       fill_prev_data(cmdvel_msg, odom_msg)
       print("filled_data_odom",msg_num)
       msg_num+=1



cmdvel_sub = message_filters.Subscriber('/odom_cmd', Odometry)
odom_sub= message_filters.Subscriber('/warthog_velocity_controller/odom', Odometry)

ts = message_filters.ApproximateTimeSynchronizer([cmdvel_sub, odom_sub] , 1000, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.init_node('data_synchronizer_odom')
rospy.spin()
