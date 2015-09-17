#!/usr/bin/env python

""" Have bot move forward 1m """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import LaserScan

class Controller:
	def __init__(self):
		rospy.init_node('proportional_control')
		rospy.Subscriber('/odom', Odometry, self.react_odom, queue_size=1)
		# rospy.Subscriber('/scan', LaserScan, self.react_laser, queue_size=1)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.command = Twist()
		self.target_distance = rospy.get_param('~target_distance')
		self.stop()

	def forward(self, x):
		self.command.linear.x = x
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = 0	

	def react_odom(self, odom):
		''' callback function: react to odom info '''
		factor = 1
		error = self.target_distance - odom.pose.pose.position.x 
		speed = error * factor
		if speed > 1:
			speed = 1
		print speed
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