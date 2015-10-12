#!/usr/bin/env python

import rospy
from geometry_msgs.msgs import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class gazebomask(object):

	def __init__(self):
		rospy.init_node('gazebomask') #start the ROS node
		self.rate = rospy.Rate(10) # set the rate to 10 hz
		self.pub_Nav = rospy.Publisher('/navsat/enu', Odometry, queue_size = 1) #publish enu pos
		