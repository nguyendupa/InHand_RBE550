import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

def forward_kinematics(f1,f2,f3,k1,k2):
	b_c  = 0.25  # bend constant of flexible finger 
	fl_1 = 77.82 # finger length 1: unit mm
	fl_2 = 45.55 # finger length 2
	
	offset_f1 = [40,30.75] # f1 joint position xy
	offset_f2 = [40,-30.75] # f1 joint position xy
	offset_f3 = [-40,0] # f1 joint position xy
	# Finger f1:
	lx_f1 = fl_1*math.cos(f1)+fl_2*math.cos(f1+f1*b_c)
	ly_f1 = fl_1*math.sin(f1)+fl_2*math.sin(f1+f1*b_c)
	x1 = lx_f1*math.cos(k1 - math.pi/8) + offset_f1[0]
	y1 = lx_f1*math.sin(k1 - math.pi/8) + offset_f1[1]
	z1 = ly_f1 

	# Finger f2:
	lx_f2 = fl_1*math.cos(f2)+fl_2*math.cos(f2+f1*b_c)
	ly_f2 = fl_1*math.sin(f2)+fl_2*math.sin(f2+f1*b_c)
	x2 = lx_f2*math.cos(math.pi/8 - k1) + offset_f2[0]
	y2 = lx_f2*math.sin(math.pi/8 - k1) + offset_f2[1]
	z2 = ly_f2

	# Finger f3:
	lx_f3 = fl_1*math.cos(f1)+fl_2*math.cos(f1+f1*b_c)
	ly_f3 = fl_1*math.sin(f1)+fl_2*math.sin(f1+f1*b_c)
	x3 = -lx_f3*math.cos(math.pi/4-k2) + offset_f3[0]
	y3 = lx_f3*math.sin(math.pi/4-k2) + offset_f3[1]
	z3 = ly_f3


	x = [x1,x2,x3]
	y = [y1,y2,y3]
	z = [z1,z2,z3]
	return [x,y,z]
global rotateXPoints, testPose,numstep
rotateXPoints = [\
[0.785, 0.785, -0.0000, 0.000 + math.pi/8, -0.000 + math.pi/4], \
[0.978, 0.785, 1.68473, 0.040 + math.pi/8, -0.005 + math.pi/4],\
[0.978, 0.785, 1.68473, 0.513 + math.pi/8, -0.005 + math.pi/4],\
[0.978, 0.785, 1.37846, 0.513 + math.pi/8, -0.005 + math.pi/4]]

testPose = [[0.0,0.0,0.0,math.pi/8,math.pi/4],[1.0,1.0,1.0,math.pi/8,math.pi/4]]
numstep = 5
def generatePath(listPose,numstep):
	allpose=[]
	for i in range(0,len(listPose)-1):
		#print "i: ", i
		stepsize_f1 = (listPose[i+1][0]-listPose[i][0])/numstep
		stepsize_f2 = (listPose[i+1][1]-listPose[i][1])/numstep
		stepsize_f3 = (listPose[i+1][2]-listPose[i][2])/numstep
		stepsize_p1 = (listPose[i+1][3]-listPose[i][3])/numstep
		stepsize_p2 = (listPose[i+1][4]-listPose[i][4])/numstep
		for j in range(0,numstep):
			pose = [listPose[i][0]+j*stepsize_f1,listPose[i][1]+j*stepsize_f2,\
			listPose[i][2]+j*stepsize_f3, listPose[i][3]+j*stepsize_p1,\
			listPose[i][4]+j*stepsize_p2]
			allpose.append(pose)
	allpose.append(listPose[len(listPose)-1])
	#print allpose
	return allpose

def poseToPoint(poses):
	allpoint = []
	for pose in poses:
		fw_ki = forward_kinematics(pose[0],pose[1],pose[2],pose[3],pose[4])
		allpoint.append(fw_ki)
	return allpoint

def RRTWithoutConstraint(start,goal)
	return 0

def RRTWithConstraint(start,goal)
	return 0
	

if __name__ == "__main__":
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	# Test 1
	# test = forward_kinematics(0.0,0.0,0.0,math.pi/4,math.pi/4)
	# test1 = forward_kinematics(1.0,1.0,1.0,math.pi/4,math.pi/4)
	# test2 = forward_kinematics(0.0,0.0,0.0,math.pi/8,math.pi/2)
	# test3 = forward_kinematics(1.0,1.0,1.0,math.pi/8,0)
	# poses = []
	# poses.append(test)
	# poses.append(test1)
	# poses.append(test2)
	# poses.append(test3)

	# Test 2
	test_all_pose = generatePath(testPose,numstep)
	test_all_point = poseToPoint(test_all_pose)


	x = []
	y = []
	z = []
	for point in test_all_point:
		x.append(point[0])
		y.append(point[1])
		z.append(point[2])	
	ax.scatter(x, y, z ,c='r',marker='o')

	test_all_pose = generatePath(rotateXPoints,numstep)
	test_all_point = poseToPoint(test_all_pose)


	x = []
	y = []
	z = []
	for point in test_all_point:
		x.append(point[0])
		y.append(point[1])
		z.append(point[2])	
	ax.scatter(x, y, z ,c='b',marker='^')

	ax.set_xlabel('X axis')
	ax.set_ylabel('Y axis')
	ax.set_zlabel('Z axis')
	plt.show()
