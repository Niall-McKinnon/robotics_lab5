#!/usr/bin/env python3
import math
import rospy
import numpy as np
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams

# Define global variables, these are used to build A and B matricies:
Ax = []
Ay = []
Az = []
b = []
# A = []

received = False

# Callback function to get ball data from publisher:
def get_ball_data(data):
	
	# Access global variables:
	global received
	global Ax
	global Ay
	global Az
	global b
	global A
	
	# Redefine lists as empty for each new dataset:
	Ax = []
	Ay = []
	Az = []
	b = []
	# A = []
	
	# Build individual columns of A and B from subscriber data:
	for point in data.points:
		
		b.append((point.x)**2 + (point.y)**2 + (point.z)**2)
		
		# A.append([2*point.x, 2*point.y, 2*point.z, 1])
		
		Ax.append(2*point.x)
		Ay.append(2*point.y)
		Az.append(2*point.z)
		
		
	if not(len(Ax) == 0 or len(Ay) == 0 or len(Az) == 0 or len(b) == 0):
	
		received = True

if __name__ == '__main__':
	
	# Initialize the node:
	rospy.init_node('sphere_fit', anonymous = True)
	
	# Add a subscriber for the XYZ data:
	rospy.Subscriber('xyz_cropped_ball', XYZarray, get_ball_data)
	
	# Define publisher:
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	# Set loop frequency:
	rate = rospy.Rate(5)
	
	counter = 0
	
	while not rospy.is_shutdown():
	
		if received:
			
			#if len(Ax) == len(Ay) and len(Ay) == len(Az):
			
			#print("lenAx: {} lenAy: {} lenAz: {}".format(Ax.shape, len(Ay), len(Az)))
			
			# Define the A matrix:
			A = np.vstack([Ax, Ay, Az, np.ones(len(Ax))]).T
			
			# A = np.array(A).T
			# Define the B matrix:
			B = np.array([b]).T
			
			if len(A) == len(B):
			
				P = np.linalg.lstsq(A, B, rcond=None)[0]
			
				
				# Get sphere params from P:
				xc = P[0]
				yc = P[1]
				zc = P[2]
				r = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
				
				# Declare variable for publisher:
				sphere_data = SphereParams()
				
				# Add sphere params to publisher:
				sphere_data.xc = xc
				sphere_data.yc = yc
				sphere_data.zc = zc
				sphere_data.radius = r
				
				# Publish messge:
				sphere_pub.publish(sphere_data)
				
				# Test statement:
				counter += 1
				print('check', counter)
