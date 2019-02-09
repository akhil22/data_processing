#!/usr/bin/env python
import rospy
import message_filters
from vn300.msg import sensors, gps
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, SystemRptFloat, VehicleSpeedRpt
from std_msgs.msg import Float64
import csv

dynamics_data = open('dynamics_data.csv', 'w')
field_names = ['x','y','theta','vel','accel','steer','omega']
data_writer = csv.DictWriter(dynamics_data,field_names)
data_writer.writeheader()
def callback(imu_msg, gps_msg, accel_msg, break_msg, steer_msg, vel_msg):
    print('got both messages\n')
    data_writer.writerow({'x': gps_msg.LLA.x, 'y': gps_msg.LLA.y, 'theta': 0, 'vel': vel_msg.vehicle_speed, 'accel': accel_msg.command, 'steer': steer_msg.command, 'omega': imu_msg.Accel.z})


imu_sub = message_filters.Subscriber('/vectornav/imu', sensors)
gps_sub= message_filters.Subscriber('/vectornav/gps', gps)
accel_sub= message_filters.Subscriber('/pacmod/parsed_tx/accel_rpt', SystemRptFloat)
break_sub= message_filters.Subscriber('/pacmod/parsed_tx/brake_rpt', SystemRptFloat)
steer_sub= message_filters.Subscriber('/pacmod/parsed_tx/steer_rpt', SystemRptFloat)
vel_sub= message_filters.Subscriber('/pacmod/parsed_tx/vehicle_speed_rpt', VehicleSpeedRpt)

ts = message_filters.ApproximateTimeSynchronizer([imu_sub, gps_sub, accel_sub, break_sub, steer_sub, vel_sub], 100, 0.1, allow_headerless=True)
ts.registerCallback(callback)
rospy.init_node('data_synchronizer')
rospy.spin()
