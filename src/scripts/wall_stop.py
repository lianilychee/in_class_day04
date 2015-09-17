#!/usr/bin/env python

""" Have bot move, then stop according to user-input distance. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
# import proportional_linear

class Controller:
	def __init__(self):
		rospy.init_node('wall_stop')
		# rospy.Subscriber('/odom', Odometry, self.react_odom, queue_size=1)
		rospy.Subscriber('/scan', LaserScan, self.react_laser, queue_size=1)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.command = Twist()
		self.target_distance = rospy.get_param('~target_distance') # user-input distance from wall
		self.stop()

	def forward(self, x):
		self.command.linear.x = x
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = 0	

	def react_laser(self, scan):
		''' callback function for laser '''
		factor = 1
		error = scan.ranges[0] - self.target_distance
		speed = error * factor
		if speed > 1:
			speed = 1
		# self.forward(speed)

		print scan.ranges[0]
		if error < 0.01:
			self.stop()
		else:
			self.forward(speed)

	def stop(self):
		''' stop bot motion '''
		self.command.linear.x = 0
		self.pub.publish(self.command)

	def drive(self):
		self.pub.publish(self.command)

controller = Controller()

while not rospy.is_shutdown():
	controller.drive()