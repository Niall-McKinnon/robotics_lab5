#!/usr/bin/env python3
import rospy
import numpy as np
from robot_vision_lectures.msg import XYZarray
# from geometry_msgs.msg import Point

coords = XYZarray()
received = False

# Callback function to get ball data from publisher:
def get_ball_data(data):
	
	global coords
	global received
	
	coords = data
	
	received = True

if __name__ == '__main__':
	
	# Initialize the node:
	rospy.init_node('sphere_fit', anonymous = True)
	
	# Add a subscriber for the XYZ data:
	rospy.Subscriber('xyz_cropped_ball', XYZarray, get_ball_data)
	
	# Set loop frequency:
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
	
		if received:
			x = coords.points[0].x
			print(x)
		
