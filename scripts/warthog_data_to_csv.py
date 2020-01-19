import rospy
from nav_msgs.msg import Odometry
import time
import csv

class DataLog:
	def __init__(self):
		rospy.Subscriber('/warthog_velocity_controller/odom', Odometry, self.odomcb)
		rospy.Subscriber('/odom_cmd', Odometry, self.cmdcb)
		self.odom_data_file = open('warthog_odom.csv', 'w')
                self.cmd_data_file = open('warthog_cmd.csv', 'w')
		self.field_odom = ['v','w','t']
		self.field_cmd = ['v_c','w_c','t_c']
		self.odom_writer = csv.DictWriter(self.odom_data_file, self.field_odom)
		self.cmd_writer = csv.DictWriter(self.cmd_data_file, self.field_cmd)
                self.odom_writer.writeheader()
                self.cmd_writer.writeheader()
                self.init_time = rospy.Time.now()
		self.odom_num = 0
		self.cmd_num = 0
        def odomcb(self, msg):
		v = msg.twist.twist.linear.x
		w = msg.twist.twist.angular.z
		dt = (msg.header.stamp - self.init_time).to_sec()
		self.odom_writer.writerow({'v': v, 'w': w, 't': dt})
		self.odom_num+=1
		print('odom_msg ', self.odom_num)
        def cmdcb(self, msg):
		v = msg.twist.twist.linear.x
		w = msg.twist.twist.angular.z
		dt = (msg.header.stamp - self.init_time).to_sec()
		self.cmd_writer.writerow({'v_c': v, 'w_c': w, 't_c': dt})
		self.cmd_num+=1
		print('cmd_msg ', self.cmd_num)

def main():
	rospy.init_node('data_logger')
	datalog = DataLog()
	rospy.spin()

if __name__=='__main__':
	main()
