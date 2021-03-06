# The object movement will only result from the movements of the fingers
# The configuration space of object is noted CSObject. In these case a cube
# Hand Config characterized bt the joint parameters of its n finger:
# 		- In these case 3 finger
# 		- Each finger has 2 joints
# 		- Each finger i has mi joints. 
# 		- O(i,k) is value of kth joint of the ith finger.
# 		- Config vector for each finger is ai = (O(i,1),O(i,2),...O(i,mi)) from S^mi (where S is circle)
# 		- Config vector of the whole hand is a = (a1,a2, ..., an)
# 		- Config space is CShand = {S^m1 x S^m2 x ...}
# Config space CS = CShand x CSObject
# Assume all contects to be point contact and the fingertips is sharp enough to neglect
# the effects of rolling on contact pos

# Grasping finger: finger that participates in the object
# Independent finger: does not

# CSfree is set of all config that do not lead to collision 
# Grasp Subspace GSk, subspace of all the configurations q with k grasping finger

# Elementary manipulation subtask: Grasp- Reconfiration and Object displacement
# The goal of dexterous manipulation planner is then to find such a sequence connecting
# two given configuration while ensuring the grasp stability all along the path sequence

# Purpose reconstructure path from contact points of finger in CS space. 


# Import
import copy
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random
import time

# Some predefine Object
obs_1 = [[40.0,40.0],[40.0,-40.0],[-40.0,-40.0],[-40.0,40.0]]
obs_1_hex = [[-15.0,26.0],[15.0,26.0],[26.0,0.0],[15.0,-26.0],[-15.0,-26.0],[-26.0,0.0]]
obs_1_hex_2 = [[-12.5,21.64],[12.5,21.64],[21.64,0.0],[12.5,-21.64],[-12.5,-21.64],[-21.64,0.0]]
obs_1_tri = [[-20.0,20.0],[-20.0,-20.0],[20.0,0.0]]
obs_1_tri_2 = [[-30.5,-17.60],[30.5,-17.60],[0.0,35.218]]
obs_1_rect = [[25.0,20.0],[25.0,-20.0],[-25.0,-20.0],[-25.0,20.0]]
obs_1_rect_2 = [[24.0,16.0],[24.0,-16.0],[-24.0,-16.0],[-24.0,16.0]]

obs_1_square = [[20.0,20.0],[20.0,-20.0],[-20.0,-20.0],[-20.0,20.0]]
obs_2 = [[20.0,0],[0.0,-20.0],[-20.0,0],[0.0,20.0]]
obs_3 = [[25.0,0],[5.0,-20.0],[-25.0,0],[5.0,20.0]]
# Pos of finger for ref
num_finger = 3
offset = []
offset_f1 = [ 40.0 , 30.75] # f1 joint position xy
offset_f2 = [ 40.0 ,-30.75] # f2 joint position xy
offset_f3 = [-40.0 , 0.00 ] # f3 joint position xy
offset.append(offset_f1)
offset.append(offset_f2)
offset.append(offset_f3)

# Angle offset for eq tri
off_30dg = 0.0
# RRT Stuff
NUMNODES = 2000

# for square
# EPSILON = 1.0 # 7
# RADIUS=3.0 #15
# for rect
EPSILON = 1.0 # 7
RADIUS=3.0 #15

# Global CS
global CS1,CS2,CS3

global totalCost,totalNode
# distance between two points
def dist(p1,p2):
	return math.sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

# find angle with x axis
def angleWithX(p1,p2):
	return math.atan2(p2[1]-p1[1],p2[0]-p1[0])

# find center point of obs
def findCenterPoint(obsGeo):
	x = []
	y = []
	for points in obsGeo:
		x.append(points[0])
		y.append(points[1])
	xarr = np.array(x)
	yarr = np.array(y)
	return [xarr.mean(),yarr.mean()]

# Find closest point from extended set to 3 pre defined point
def findClosestPoint(extGeo,comparedPoints):
	# find closest points to each start position of finger
	index_list = []
	min_list = []
	for i in range(0,len(comparedPoints)):
		dist_list = []
		for points in extGeo:
			dist_list.append(dist(points,comparedPoints[i]))
		darr = np.array(dist_list)
		index_list.append(darr.argmin())
		min_list.append(darr.min())
		# if (i == 2):
		# 	print dist_list
		#print "Min: ", darr.min(), "Index: ", darr.argmin()
	return index_list, min_list

# Plan the original Grasp
# obsGeo is list of (x,y) position, corner of the object
def planStartGrasp(obsGeo):
	# get all points from extend geometry
	obs_points = extendGeometry(obsGeo,10)
	
	# find closest points to each start position of finger
	(index_list, min_list) = findClosestPoint(obs_points,offset)
	#print "Index: ", index_list
	# First cloest Points for debuging purpose
	first_closest_points = [obs_points[index_list[0]],obs_points[index_list[1]],obs_points[index_list[2]]]
	
	# find min of min to choose the keep point
	minarr = np.array(min_list)
	pivot_index = index_list[minarr.argmin()]
	pivot_point = obs_points[pivot_index]

	# Center point of Obs
	center_point = findCenterPoint(obsGeo)
	#print "Center Point: ",center_point
	# Equ Tri Vertices
	eq_points = findEqTriPoint(pivot_point,center_point)
	# Find new cloest Point
	(index_list, min_list) = findClosestPoint(obs_points,eq_points)
	

	# for i in range(0,num_finger):
	# 	for j in range(0,len(obs_points)):
	# 		if (min_dummy > dist(points,offset[i])):
	# 			min_dummy = dist(points,offset[i])
	closest_points = [obs_points[index_list[0]],obs_points[index_list[1]],obs_points[index_list[2]]]
	
	return closest_points, index_list

# plan rotate point of obs an angle
def rotateObs(obsGeo, rotateAngleDegree):
	rotateAngleRad = np.radians(rotateAngleDegree)
	newObsGeo = []
	for point in obsGeo:
		# Rotation matrix
		x_prime = math.cos(rotateAngleRad)*point[0] - math.sin(rotateAngleRad)*point[1]
		y_prime = math.sin(rotateAngleRad)*point[0] + math.cos(rotateAngleRad)*point[1]
		newObsGeo.append([x_prime,y_prime])
	return newObsGeo

# Find the vertices of eq tri from 1 point and center
def findEqTriPoint(verPoint, centerPoint):
	sideLength = math.sqrt(3)*dist(verPoint,centerPoint)
	angleWX = angleWithX(verPoint,centerPoint)
	angle_offset_1 = angleWX + off_30dg
	angle_offset_2 = angleWX - off_30dg
	p2 = [verPoint[0] + sideLength*math.cos(angle_offset_1),\
		verPoint[1] + sideLength*math.sin(angle_offset_1)]
	p3 = [verPoint[0] + sideLength*math.cos(angle_offset_2),\
		verPoint[1] + sideLength*math.sin(angle_offset_2)]

	return [verPoint,p2,p3]
# Extend the point of contact from 4 to whatever to limit size of search
def extendGeometry(obsGeo,numstep):
	allpoints=[]
	dummyGeo = copy.deepcopy(obsGeo)
	dummyGeo.append(dummyGeo[0])
	for i in range(0,len(dummyGeo)-1):
		#print "i: ", i
		stepsize_f1 = (dummyGeo[i+1][0]-dummyGeo[i][0])/numstep
		stepsize_f2 = (dummyGeo[i+1][1]-dummyGeo[i][1])/numstep
		for j in range(0,numstep):
			point = [dummyGeo[i][0]+j*stepsize_f1,dummyGeo[i][1]+j*stepsize_f2]
			allpoints.append(point)
	#print allpoints
	return allpoints



# For RRT Stuff
# Cost of node
class Cost:
	x = 0
	y = 0
	cost=0  
	parent=None
	def __init__(self,xcoord, ycoord):
		 self.x = xcoord
		 self.y = ycoord

# Node class used in algorithm
class Node:
	x = 0
	y = 0
	cost=0  
	parent=None
	def __init__(self,xcoord, ycoord):
		 self.x = xcoord
		 self.y = ycoord

# return node within radius
def step_from_to(p1,p2):
	if dist(p1,p2) < EPSILON:
		return p2
	else:
		theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
		return p1[0] + EPSILON*math.cos(theta), p1[1] + EPSILON*math.sin(theta)
def ccw(A,B,C):
	return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

# Area Function for Intersect checking
def area(A,B,C):
	return abs((A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0);

# Check if a point is inside a rectangle or not:
def checkOutside(A,B,C,D,E):
	inst_check = area(A,B,C) + area(A,D,C)
	inst1 = area(E,A,B)
	inst2 = area(E,B,C)
	inst3 = area(E,C,D)
	inst4 = area(E,D,A) 
	#print "start: ", inst_check
	#print "debug: ", (inst1+inst2+inst3+inst4)
	return (inst_check == (inst1+inst2+inst3+inst4))

# Return true if line segments AB and CD intersect
def checkIntersect(nodeA,nodeB,OBS):
	A=(nodeA.x,nodeA.y)
	B=(nodeB.x,nodeB.y)
	C=OBS[0]
	D=OBS[1]
	E=OBS[2]
	if (len(OBS) == 4):
		F=OBS[3]
		inst1 = intersect(A,B,C,D)
		inst2 = intersect(A,B,D,E)
		inst3 = intersect(A,B,E,F)
		inst4 = intersect(A,B,F,C)
	elif (len(OBS) == 3):
		inst1 = intersect(A,B,C,D)
		inst2 = intersect(A,B,D,E)
		inst3 = intersect(A,B,E,C)
		inst4 = False
	else:
		F=OBS[3]
		G=OBS[4]
		H=OBS[4]
		inst1 = intersect(A,B,C,D) and intersect(A,B,D,E) and intersect(A,B,E,F) 
		inst2 = intersect(A,B,F,G) and intersect(A,B,G,H) and intersect(A,B,H,C)
		inst3 = False
		inst4 = False
		
	# inst1 = intersect(A,B,C,D)
	# inst2 = intersect(A,B,D,E)
	# inst3 = intersect(A,B,E,F)
	# inst4 = intersect(A,B,F,C)
	# inst5 = checkOutside(C,D,E,F,A)
	# inst6 = checkOutside(C,D,E,F,B)
	if inst1==False and inst2==False and inst3==False and inst4==False: #and \
	# inst5==False and inst6==False:
		return True  
	return False
	

# assign parent of node
def chooseParent(nn,newnode,nodes,OBS):
		for p in nodes:
			if checkIntersect(p,newnode,OBS) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
				nn = p
		newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
		newnode.parent=nn
		return newnode,nn
# re wire process for RRT*
def reWire(nodes,newnode,OBS):
		for i in range(len(nodes)):
			p = nodes[i]
			if checkIntersect(p,newnode,OBS) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
				p.parent = newnode
				p.cost=newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) 
				nodes[i]=p
		return nodes
# draw solution
def getSolutionPath(start,goal,nodes):
	path = []
	nn = nodes[0]
	for p in nodes:
	   if dist([p.x,p.y],[goal.x,goal.y]) < dist([nn.x,nn.y],[goal.x,goal.y]):
		  nn = p
	while nn!=start:
		path.append(nn)
		nn=nn.parent
	return path

def rrt(startAPoint,endAPoint,OBS):
	#initialize and prepare screen
	#a=checkIntersect()
	#print(a)
	nodes = []
	XDIM = 160
	YDIM = 160
	#nodes.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the center
	nodes.append(Node(startAPoint[0],startAPoint[1])) 
	start=nodes[0]
	goal=Node(endAPoint[0],endAPoint[1])
	for i in range(NUMNODES):
		rand = Node(random.random()*XDIM, random.random()*YDIM)
		nn = nodes[0]
		for p in nodes:
			if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
				nn = p
		interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
	
		newnode = Node(interpolatedNode[0],interpolatedNode[1])
		if checkIntersect(nn,rand,OBS):
			newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
			newnode.parent=nn
			nodes.append(newnode)
			# Do not re wire to become RRT
			#nodes=reWire(nodes,newnode,pygame,screen)
			# Terminate condition
			if ((dist([newnode.x,newnode.y],[goal.x,goal.y]) < RADIUS) and (checkIntersect(goal,newnode,OBS))):

				[goal,newnode]=chooseParent(newnode,goal,nodes,OBS);
				nodes.append(goal)
				nodes=reWire(nodes,goal,OBS)
				#print("RRT finished")
				break
	return getSolutionPath(start,goal,nodes)

# Bi directional RRT* 
def bidirectionalRRT(startAPoint,endAPoint,OBS):
	#initialize and prepare screen
	global totalNode,totalCost
	nodes = []
	XDIM = 160.0
	YDIM = 160.0
	#nodes.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the center
	nodes.append(Node(startAPoint[0],startAPoint[1]))
	start=nodes[0]
	goal=Node(endAPoint[0],endAPoint[1])

	# Second tree start from goal
	nodes_goal = []
	nodes_goal.append(goal)
	for i in range(NUMNODES):
		# Start tree
		rand = Node(random.random()*XDIM, random.random()*YDIM)
		
		nn = nodes[0]
		
		# find cloest node to rand node
		for p in nodes:
			if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
				nn = p
		# find the interpolated node
		interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
	
		newnode = Node(interpolatedNode[0],interpolatedNode[1])
		# Assign this newnode1 for the goal tree
		newnode1 = Node(interpolatedNode[0],interpolatedNode[1])
		newnode_goal = None
		if checkIntersect(nn,rand,OBS):
		  
			[newnode,nn]=chooseParent(nn,newnode,nodes,OBS);
	   
			nodes.append(newnode)
			nodes=reWire(nodes,newnode,OBS)
			
			
			# Goal tree: Similar to start tree but for goal tree, goal tree start when we found new node
			rand_goal = newnode1
			nn_goal = nodes_goal[0]

			for p in nodes_goal:
				if dist([p.x,p.y],[rand_goal.x,rand_goal.y]) < dist([nn_goal.x,nn_goal.y],[rand_goal.x,rand_goal.y]):
					nn_goal = p

			interpolatedNode_goal= step_from_to([nn_goal.x,nn_goal.y],[rand_goal.x,rand_goal.y])
			newnode_goal = Node(interpolatedNode_goal[0],interpolatedNode_goal[1])
			# check if there is an obstacle between new node and the closest node from start tree
			if checkIntersect(nn_goal, rand_goal, OBS):
				# if yes then use connect type
				[newnode_goal,nn_goal]=chooseParent(nn_goal,newnode_goal,nodes_goal,OBS);

				nodes_goal.append(newnode_goal)
				nodes_goal=reWire(nodes_goal, newnode_goal,OBS)
				
				# if find goal within radius and the new node then exist
				if (dist([nn_goal.x,nn_goal.y],[rand_goal.x,rand_goal.y]) < RADIUS):

					# start tree end with new node, and goal tree end with newnode_goal
					totalNode = totalNode + i
					#print("Bidirectional RRT star")
					break
			else:
				# if no then random again for extend type
				rand_goal = Node(random.random()*XDIM, random.random()*YDIM)
				nn_goal = nodes_goal[0]

				for p in nodes_goal:
					if dist([p.x,p.y],[rand_goal.x,rand_goal.y]) < dist([nn_goal.x,nn_goal.y],[rand_goal.x,rand_goal.y]):
						nn_goal = p

				interpolatedNode_goal= step_from_to([nn_goal.x,nn_goal.y],[rand_goal.x,rand_goal.y])
				newnode_goal = Node(interpolatedNode_goal[0],interpolatedNode_goal[1])
				if checkIntersect(nn_goal, rand_goal, OBS):
					[newnode_goal,nn_goal]=chooseParent(nn_goal,newnode_goal,nodes_goal,OBS);

					nodes_goal.append(newnode_goal)
					nodes_goal=reWire(nodes_goal, newnode_goal,OBS)
					
		if (newnode_goal == None):
			newnode_goal = newnode
			path = []
			path.append(start)
			path.append(goal)
			#totalNode = totalNode + i
			return path
	# draw from the previous newnode and newnode_goal when we break out of for loop
	#print("Path legnth: " + str(newnode.cost+newnode_goal.cost))	# print out cost length
	totalCost = totalCost + newnode.cost+newnode_goal.cost
	return getSolutionPath(start,newnode,nodes) + getSolutionPath(goal,newnode_goal,nodes_goal)
def nodeToList(nodes):
	listofnode = []
	for node in nodes:
		listofnode.append([node.x,node.y])
	return listofnode
# Regrasp Planning
# start & end: 3 p (x,y)
# obs current obs config 4 p (x,y)
def regrasp(start,end,obs,check):
	#test_rrt_node = bidirectionalRRT(start,end,obs)
	#print "Star Regrasp"
	resultPose = []
	list_extra_poses = []

	startNode = copy.deepcopy(start[check-1])
	test_start_node = copy.deepcopy(start[check-1])
	list_extra_poses.append(test_start_node)
	startNode[0] = round(startNode[0]+np.sign(startNode[0])*3,1)
	startNode[1] = round(startNode[1]+np.sign(startNode[1])*3,1)
	#print startNode
	endNode = copy.deepcopy(end[check-1])
	endNode[0] = round(endNode[0] + np.sign(endNode[0])*3,1)
	endNode[1] = round(endNode[1] + np.sign(endNode[1])*3,1)
	#print endNode
	
	list_extra_poses = nodeToList(bidirectionalRRT(startNode,endNode,obs))
	test_end_node = copy.deepcopy(end[check-1])
	#print test_end_node
	list_extra_poses.append(test_end_node)

	for j in range(0,len(list_extra_poses)):
		if (check-1) == 0:
			resultPose.append([list_extra_poses[j],start[1],start[2]])
		elif (check-1) == 1:
			resultPose.append([start[0],list_extra_poses[j],start[2]])
		elif (check-1) == 2:
			resultPose.append([start[0],start[1],list_extra_poses[j]])

	#print list_extra_poses
	#print resultPose
	return resultPose

def detectRegrasp(current,last,obs,stepnum_side):
	inst1 = [int(last[0]/stepnum_side),int(last[1]/stepnum_side),int(last[2]/stepnum_side)]
	inst2 = [int(current[0]/stepnum_side),int(current[1]/stepnum_side),int(current[2]/stepnum_side)]
	#print current
	#print last
	#print inst1
	#print inst2
	for i in range(0,len(inst1)):
		if not ((inst1[i] == inst2[i]) or (current[i]%stepnum_side == 0) \
			or (last[i]%stepnum_side == 0)):
			#print "Detect Regrasp at Finger: ", (i+1)
			return (i+1)
	return 0


def forward_kinematics(f1,k1,f2,k2,f3,k3):
	b_c  = 0.25  # bend constant of flexible finger 
	fl_1 = 77.82 # finger length 1: unit mm
	fl_2 = 45.55 # finger length 2
	
	offset_f1 = [40,30.75] # f1 joint position xy
	offset_f2 = [40,-30.75] # f1 joint position xy
	offset_f3 = [-40,0] # f1 joint position xy
	# Finger f1:
	lx_f1 = fl_1*math.cos(f1)+fl_2*math.cos(f1+f1*b_c)
	ly_f1 = fl_1*math.sin(f1)+fl_2*math.sin(f1+f1*b_c)
	x1 = lx_f1*math.cos(k1) + offset_f1[0]
	y1 = lx_f1*math.sin(k1) + offset_f1[1]
	z1 = ly_f1 

	# Finger f2:
	lx_f2 = fl_1*math.cos(f2)+fl_2*math.cos(f2+f2*b_c)
	ly_f2 = fl_1*math.sin(f2)+fl_2*math.sin(f2+f2*b_c)
	x2 = lx_f2*math.cos(k2) + offset_f2[0]
	y2 = lx_f2*math.sin(k2) + offset_f2[1]
	z2 = ly_f2

	# Finger f3:
	lx_f3 = fl_1*math.cos(f3)+fl_2*math.cos(f3+f3*b_c)
	ly_f3 = fl_1*math.sin(f3)+fl_2*math.sin(f3+f3*b_c)
	x3 = lx_f3*math.cos(k3) + offset_f3[0]
	y3 = lx_f3*math.sin(k3) + offset_f3[1]
	z3 = ly_f3


	x = [x1,x2,x3]
	y = [y1,y2,y3]
	z = [z1,z2,z3]
	p1 = [x1,y1,z1]
	p2 = [x2,y2,z2]
	p3 = [x3,y3,z3]
	
	# return [x,y,z]
	return [p1,p2,p3]

def fw_kin_single(f,k,num):
	b_c  = 0.25  # bend constant of flexible finger 
	fl_1 = 77.82 # finger length 1: unit mm
	fl_2 = 45.55 # finger length 2
	
	offset_f1 = [40,30.75] # f1 joint position xy
	offset_f2 = [40,-30.75] # f1 joint position xy
	offset_f3 = [-40,0] # f1 joint position xy
	offset = [[40,30.75],[40,-30.75],[-40,0]]
	
	# All 3 finger has some kinematic except different offset
	lx_f = fl_1*math.cos(f)+fl_2*math.cos(f+f*b_c)
	ly_f = fl_1*math.sin(f)+fl_2*math.sin(f+f*b_c)
	x1 = lx_f*math.cos(k) + offset[num-1][0]
	y1 = lx_f*math.sin(k) + offset[num-1][1]
	z1 = ly_f
	
	#return [x1,y1,z1] 
	return [x1,y1]

def explore_singleCS(num):
	num_nodes = 100
	configSpace = {}

	f_space = 0.0 # f goes from 0.0 to 3*pi/4
	step_f = (3*math.pi/4)/num_nodes
	
	if (num == 3):
		k_space = -math.pi/2 # k3 goes from -pi/2 to pi/2
		step_k = math.pi/num_nodes
	elif (num==1):
		k_space = -math.pi/8
		step_k = (math.pi/2+math.pi/8)/num_nodes
	elif (num==2):
		k_space = -math.pi/2
		step_k = (math.pi/2+math.pi/8)/num_nodes

	for i in range(0,num_nodes):
		
		for j in range(0,num_nodes):
			configSpace[(f_space,k_space)] = fw_kin_single(f_space,k_space,num) 
			k_space = k_space + step_k
		if (num == 3):
			k_space = -math.pi/2
		elif (num==1):
			k_space = -math.pi/8
		elif (num==2):
			k_space = -math.pi/2
		f_space = f_space + step_f
	return configSpace
def exploreCS():
	global CS1,CS2,CS3
	CS1 = explore_singleCS(1)
	CS2 = explore_singleCS(2)
	CS3 = explore_singleCS(3)

# find the closest config space to a pose or 3 point (x,y)
def closestConfigspace(p1,p2,p3):
	min_value_1 = 99999
	min_value_2 = 99999
	min_value_3 = 99999

	for config in CS1:
		value = dist(p1,CS1[config])
		if (min_value_1>value):
			min_value_1 = value
			current_config_1 = config

	for config in CS2:
		value = dist(p2,CS2[config])
		if (min_value_2>value):
			min_value_2 = value
			current_config_2 = config

	for config in CS3:
		value = dist(p3,CS3[config])
		if (min_value_3>value):
			min_value_3 = value
			current_config_3 = config


	# print current_key
	# print CS1[current_key]

	# return 3 current config each is just (f,k)
	return [current_config_1,current_config_2,current_config_3]

# Testing figure stuff
global fig,ax,ax2,index,all_pose, all_pose_rect,ax3
global all_pose_after_kin

fig = plt.figure(figsize=(20,6))
ax = fig.add_subplot(131, projection='3d')
ax2 = fig.add_subplot(132, projection='3d')
ax3 = fig.add_subplot(133, projection='3d')
index = 0
all_pose = []
all_pose_after_kin = []
all_pose_rect = []

def updateData(num):
	global fig,ax,ax2,index,all_pose,all_pose_after_kin,ax3
	ax2.clear()
	ax2.set_xlim(-40, 40)
	ax2.set_ylim(-40, 40)
	ax2.set_zlim(0, 200)
	ax2.set_xlabel('X axis')
	ax2.set_ylabel('Y axis')
	ax2.set_zlabel('Z axis')
	ax3.clear()
	ax3.set_xlim(-40, 40)
	ax3.set_ylim(-40, 40)
	ax3.set_zlim(50, 150)
	ax3.set_xlabel('X axis')
	ax3.set_ylabel('Y axis')
	ax3.set_zlabel('Z axis')
	x = []
	y = []
	z = []
	if (index > 4):
		for point in all_pose[index]:
			x.append(point[0])
			y.append(point[1])
			z.append(10)
		ax2.scatter(x, y, z ,c='r',marker='o')

		for i in range(0,6):
			for point in all_pose[index-i]:
				x.append(point[0])
				y.append(point[1])
				z.append(10)
		ax2.scatter(x, y, z ,c='b',marker='.',alpha=0.5)

	else:
		for point in all_pose[index]:
			x.append(point[0])
			y.append(point[1])
			z.append(10)
		ax2.scatter(x, y, z ,c='r',marker='o')
	index = index + 1 
	# if (index > 4):

	# 	#draw path
	# 	for point in all_pose_after_kin[index]:
	# 		x.append(point[0])
	# 		y.append(point[1])
	# 		z.append(point[2])
	# 	ax2.scatter(x, y, z ,c='r',marker='o')
		
	# 	for i in range(0,6):
	# 		for point in all_pose_after_kin[index-i]:
	# 			x.append(point[0])
	# 			y.append(point[1])
	# 			z.append(point[2])
	# 	ax2.scatter(x, y, z ,c='b',marker='.',alpha=0.5)

	# 	#draw triangle
	# 	# for i in range(0,2):
	# 	# 	for point in all_pose_after_kin[index-i]:
	# 	# 		x.append(point[0])
	# 	# 		y.append(point[1])
	# 	# 		z.append(point[2])
	# 	# ax2.scatter(x, y, z ,c='b',marker='.',alpha=0.5)
		
	# 	# ax2.plot_wireframe(x, y, z)

	# else:
	# 	for point in all_pose_after_kin[index]:
	# 		x.append(point[0])
	# 		y.append(point[1])
	# 		z.append(point[2])
	# 	ax2.scatter(x, y, z ,c='r',marker='o')
	# 	# ax2.plot_wireframe(x, y, z)

	x = []
	y = []
	z = []
	for point in all_pose_rect[index]:
		x.append(point[0])
		y.append(point[1])
		z.append(100)
	ax3.plot_wireframe(x, y, z)

	index = index + 1 
	if ((index == len(all_pose)) or (index > len(all_pose_rect)-1)):
		index = 0;
	#ax2.scatter(x, y, z ,c='r',marker=',')
# main function
if __name__ == "__main__":
	global fig,ax,ax2,index,all_pose
	global totalCost,totalNode

	# Initial Angle setup stuff
	off_30dg = np.radians(30)
	profilingTime = []
	totalCostList = []
	totalNodeList = []
	inversePose = []
	startTime = time.time()
	for runningTime in range(0,1):
		all_pose = []
		all_pose_rect = []
		index = 0
		stepnum_side = 10
		stepnum_time = 36
		totalCost = 0
		totalNode = 0
		# Rotate Test and Grasp Planning Structure
		for i in range(0,stepnum_time):
			test_rotate = rotateObs(obs_1, -i*5)
			# draw extended geometry
			obs_points = extendGeometry(test_rotate,stepnum_side)
			#print "Length Obs: ", i
			# x = []
			# y = []
			# z = []
			# for point in obs_points:
			# 	x.append(point[0])
			# 	y.append(point[1])
			# 	z.append(i)
			# # ax.scatter(x, y, z ,c='r',marker='.',alpha=0.4)
			# ax.plot_wireframe(x, y, z,alpha=0.1)
			
			# draw original grasp

			# Control loop for regrasp 
			(ori_grasp, ind) = planStartGrasp(test_rotate)
			# x = []
			# y = []
			# z = []

			# for point in ori_grasp:
			# 	#print point
			# 	x.append(point[0])
			# 	y.append(point[1])
			# 	z.append(i)
			# ax.scatter(x, y, z ,c='r',marker='o',alpha=0.5)

			if (i>0):
				check = detectRegrasp(ind,last_ind,test_rotate,stepnum_side)
				if (check==0) :
					all_pose.append(ori_grasp)
					all_pose_rect.append(obs_points)
				else:
					extra_pose = regrasp(ori_grasp,last_pose,test_rotate,check)
					# x = []
					# y = []
					# z = []
					# for pose in extra_pose:
					# 	for point in pose:
					# 		x.append(point[0])
					# 		y.append(point[1])
					# 		z.append(i)
					# 	all_pose.append(pose)
					# 	all_pose_rect.append(obs_points)
					# 	ax.scatter(x, y, z ,c='b',marker='x')
						# ax.plot_wireframe(x, y, z,alpha=0.1)
					all_pose.append(ori_grasp)
					all_pose_rect.append(obs_points)
					#print "RREEEGRAAASSSPPPP"
			else:
				all_pose.append(ori_grasp)
				all_pose_rect.append(obs_points)

			last_pose = ori_grasp
			last_ind = ind
				
		# Kinematics Test ############################################
		# print forward_kinematics(0.0,0.0,0.0,0.0,0.0,0.0)
		exploreCS()
		# config_test = closestConfigspace([164.0,31.0],[164.0,-31.0],[-164.0,0.0])
		# finger1 = list(config_test[0])
		# finger2 = list(config_test[1])
		# finger3 = list(config_test[2])
		# print config_test
		# print forward_kinematics(finger1[0],finger1[1],finger2[0],finger2[1],finger3[0],finger3[1])
		
		global all_pose_after_kin
		all_pose_after_kin = []
		# x = []
		# y = []
		# z = []
		thefile = open('dataCube.txt', 'w')
		for pose in all_pose:
			config_test = closestConfigspace(pose[0],pose[1],pose[2])
			finger1 = list(config_test[0])
			finger2 = list(config_test[1])
			finger3 = list(config_test[2])
			fw_kin = forward_kinematics(finger1[0],finger1[1],finger2[0],finger2[1],finger3[0],finger3[1])
			inversePose.append([finger1[0],finger1[1],finger2[0],finger2[1],finger3[0],finger3[1]])
			item = "["+str(finger1[0]) + ","+ str(finger1[1])+ ","+str(finger2[0])+ ","+str(finger2[1])+ ","+str(finger3[0])+ ","+str(finger3[1])+"]"
			thefile.write("%s\n" % item)
			#print fw_kin
			all_pose_after_kin.append(fw_kin)
		thefile.close()
		executionTime = time.time() - startTime
		print "Execution Time: ", executionTime
		startTime = time.time()
		profilingTime.append(executionTime)
		totalCostList.append(totalCost)
		totalNodeList.append(totalNode)
		# plt.show()

	npProfilingTime = np.array(profilingTime)
	npTotalCost = np.array(totalCostList)
	npTotalNode = np.array(totalNodeList)
	print profilingTime
	print totalCostList
	print totalNodeList
	print "Mean of Execution Time: ", npProfilingTime.mean()
	print "Mean of Total Cost: ", npTotalCost.mean()
	print "Mean of Total Node: ", npTotalNode.mean()
	
	#ani = animation.FuncAnimation(fig, updateData,frames=np.arange(0, len(all_pose)),interval=200)
	# ani = animation.FuncAnimation(fig, updateData,frames=np.arange(0, 50),interval=200)
	#ani.save('hex.gif', dpi=80, writer='imagemagick')

	#print "Center of Obs", findCenterPoint(obs_3)
	# RRT Test Stuff ############################################
# 	[17.712376928240996, 15.390656589485101]
# [10.311221522147784, 21.069762874806809]
	# test_rrt_node = bidirectionalRRT([75, 75],[-75, -75],obs_1)

	# x = []
	# y = []
	# z = []
	# for node in test_rrt_node:
	# 	x.append(node.x)
	# 	y.append(node.y)
	# 	z.append(-1)
	# ax.scatter(x, y, z ,c='b',marker='.')
	# ax.plot_wireframe(x, y, z)
	# s

	
	