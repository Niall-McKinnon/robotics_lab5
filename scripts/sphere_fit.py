#!/usr/bin/env python3
import math
import rospy
import numpy as np
from robot_vision_lectures.msg import XYZarray
from robot_vision_lectures.msg import SphereParams
import matplotlib.pyplot as plt

# Define global variables, these are used to build A and B matricies:
Ax = []
Ay = []
Az = []
b = []

received = False

# Callback function to get ball data from publisher:
def get_ball_data(data):
	
	# Access global variables:
	global received
	global Ax
	global Ay
	global Az
	global b
	
	# Redefine lists as empty for each new dataset:
	Ax = []
	Ay = []
	Az = []
	b = []
	
	# Build individual columns of A and B from subscriber data:
	for point in data.points:
		
		b.append((point.x)**2 + (point.y)**2 + (point.z)**2)
		
		Ax.append(2*point.x)
		Ay.append(2*point.y)
		Az.append(2*point.z)
	
	received = True

if __name__ == '__main__':
	
	# Initialize the node:
	rospy.init_node('sphere_fit', anonymous = True)
	
	# Add a subscriber for the XYZ data:
	rospy.Subscriber('xyz_cropped_ball', XYZarray, get_ball_data)
	
	# Define publisher:
	sphere_pub = rospy.Publisher('/sphere_params', SphereParams, queue_size = 1)
	
	# Set loop frequency:
	rate = rospy.Rate(10)
	
	while not rospy.is_shutdown():
	
		if received:
			
			if len(Ax) == len(Ay) and len(Ay) == len(Az):
				# Test statement:
				# print('A: {} B: {} C: {}'.format(len(Ax), len(Ay), len(Az)))
				
				# Define the A matrix:
				A = np.vstack([Ax, Ay, Az, np.ones(len(Ax))]).T
		
				# Define the B matrix:
				B = np.array([b]).T
				
				# Calculate the product of A^T and A:
				ATA = np.matmul(A.T, A)
				
				# Calculate the product of A^T and B:
				ATB = np.matmul(A.T, B)
				
				# Calculate P:
				P = np.matmul(np.linalg.inv(ATA), ATB)
				
				# Get sphere params from P:
				xc = P[0]
				yc = P[1]
				zc = P[2]
				r = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
				
				# Get sphere params from P:
				sphere_pub.xc = P[0]
				sphere_pub.yc = P[1]
				sphere_pub.zc = P[2]
				sphere_pub.radius = math.sqrt(P[3] + xc**2 + yc**2 + zc**2)
				
				# Publish messge:
				sphere_pub.publish()
				
				# Test statement:
				print('check')
