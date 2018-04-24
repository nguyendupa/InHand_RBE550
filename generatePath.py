rotateXPoints = [[0.7853899999999996, 0.7853899999999996, -0.0, 0.0, -0.0], \
[0.978018, 0.7853899999999996, 1.68473, 0.04, -0.00485107],\
[0.978018, 0.7853899999999996, 1.68473, 0.5131370000000001, -0.00485107],\
[0.978018, 0.7853899999999996, 1.37846, 0.513137, -0.00485107]]
numstep = 10
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
if __name__ == "__main__":
	print(len(rotateXPoints))
	generatePath(rotateXPoints,numstep)