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
import numpy as np


# Some predefine Object
obs_1 = [[2.0,2.0],[2.0,-2.0],[-2.0,-2.0],[-2.0,2.0]]


# Plan the original Grasp
# obsGeo is list of (x,y) position, corner of the object
def planStartGrasp(obsGeo):
	return 0

# Extend the point of contact from 4 to whatever to limit size of search
def extendGeometry(obsGeo,numstep):
	allpoints=[]
	dummyGeo = copy.deepCopy(obsGeo)
	dummyGeo.append(dummyGeo[0])
	for i in range(0,len(obsGeo)-1):
		#print "i: ", i
		stepsize_f1 = (obsGeo[i+1][0]-obsGeo[i][0])/numstep
		stepsize_f2 = (obsGeo[i+1][1]-obsGeo[i][1])/numstep
		for j in range(0,numstep):
			point = [obsGeo[i][0]+j*stepsize_f1,obsGeo[i][1]+j*stepsize_f2]
			allpoints.append(point)
	print allpoints
	return allpoints

# main function

if __name__ == "__main__":
	extendGeometry(obs_1,10)