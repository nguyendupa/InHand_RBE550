#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.vis import GLSimulationPlugin

test = "../Klampt/data/robots/baxter_with_parallel_gripper_col.rob"
global rotateXPoints2
global numstep
global allpose
rotateXPoints2 = [[2.16769893098,0.373064127614,2.04988920647,-1.23700210735,0.824668071567,-0.565486677646],\
[2.16769893098,0.333794219444,2.02632726157,-1.27627201552,0.918915851175,-0.659734457254],\
[2.19126087588,0.373064127614,2.00276531666,-1.31554192369,0.871791961371,-0.659734457254],\
[1.43727863902,-0.294524311274,1.72002197784,0.333794219444,1.29590696961,1.38230076758],\
[2.21482282078,0.373064127614,2.00276531666,-1.29590696961,0.801106126665,-0.471238898038],\
[2.19126087588,0.333794219444,1.97920337176,-1.33517687778,0.824668071567,-0.53407075111],\
[2.19126087588,0.294524311274,1.93207948196,-1.35481183186,0.848230016469,-0.565486677646],\
[2.23838476568,0.314159265359,1.97920337176,-1.31554192369,0.848230016469,-0.628318530718],\
[2.21482282078,0.274889357189,1.95564142686,-1.33517687778,0.871791961371,-0.659734457254],\
[1.50796447372,0.922842841992,1.67289808804,0.373064127614,1.31946891451,1.53938040026],\
[2.28550865549,0.412334035784,2.09701309627,-1.25663706144,0.848230016469,-0.439822971503],\
[2.26194671058,0.373064127614,2.07345115137,-1.29590696961,0.871791961371,-0.471238898038],\
[2.23838476568,0.412334035784,2.04988920647,-1.33517687778,0.848230016469,-0.439822971503],\
[2.21482282078,0.373064127614,2.02632726157,-1.37444678595,0.871791961371,-0.471238898038],\
[2.19126087588,0.333794219444,1.97920337176,-1.41371669412,0.871791961371,-0.502654824574],\
[1.93207948196,1.55116137271,2.09701309627,-0.157079632679,0.942477796077,0.753982236862],\
[2.16769893098,0.353429173529,2.09701309627,-1.27627201552,0.848230016469,-0.53407075111],\
[2.19126087588,0.412334035784,2.07345115137,-1.19773219918,0.871791961371,-0.565486677646],\
[2.16769893098,0.373064127614,2.04988920647,-1.23700210735,0.824668071567,-0.565486677646],\
[2.16769893098,0.333794219444,2.02632726157,-1.27627201552,0.918915851175,-0.659734457254],\
[2.19126087588,0.373064127614,2.00276531666,-1.31554192369,0.871791961371,-0.659734457254],\
[1.43727863902,-0.294524311274,1.72002197784,0.333794219444,1.29590696961,1.38230076758],\
[2.21482282078,0.373064127614,2.00276531666,-1.29590696961,0.801106126665,-0.471238898038],\
[2.19126087588,0.333794219444,1.97920337176,-1.33517687778,0.824668071567,-0.53407075111],\
[2.19126087588,0.294524311274,1.93207948196,-1.35481183186,0.848230016469,-0.565486677646],\
[2.23838476568,0.314159265359,1.97920337176,-1.31554192369,0.848230016469,-0.628318530718],\
[2.21482282078,0.274889357189,1.95564142686,-1.33517687778,0.871791961371,-0.659734457254],\
[1.50796447372,0.922842841992,1.67289808804,0.373064127614,1.31946891451,1.53938040026],\
[2.28550865549,0.412334035784,2.09701309627,-1.25663706144,0.848230016469,-0.439822971503],\
[2.26194671058,0.373064127614,2.07345115137,-1.29590696961,0.871791961371,-0.471238898038],\
[2.23838476568,0.412334035784,2.04988920647,-1.33517687778,0.848230016469,-0.439822971503],\
[2.21482282078,0.373064127614,2.02632726157,-1.37444678595,0.871791961371,-0.471238898038],\
[2.19126087588,0.333794219444,1.97920337176,-1.41371669412,0.871791961371,-0.502654824574],\
[1.93207948196,1.55116137271,2.09701309627,-0.157079632679,0.942477796077,0.753982236862],\
[2.16769893098,0.353429173529,2.09701309627,-1.27627201552,0.848230016469,-0.53407075111],\
[2.19126087588,0.412334035784,2.07345115137,-1.19773219918,0.871791961371,-0.565486677646]]

rotateXPoints2_ori = [[0.7853899999999996, 0.7853899999999996, 1.73257, 0.02, 0.0],\
 [0.848992, 0.7853899999999996, 1.73257, 0.060000000000000005, 0.0],\
  [0.848992, 0.7853899999999996, 1.73257, 0.509271, 0.0],\
   [0.848992, 0.955418, 1.73257, 0.509271, 0.0],\
    [0.848992, 0.955418, 1.89257, 0.349271, 0.0],\
    [1.26002, 1.18539, 1.14033, 0.315973, 0.0]]


global rotateXPoints
rotateXPoints = [[0.7853899999999996, 0.7853899999999996, 1.73257, 0.04, 0.0], \
[1.09749, 0.7853899999999996, 1.73257, 0.04, 0.0], \
[1.09749, 0.7853899999999996, 1.73257, 0.833137, 0.0], \
[1.09749, 0.7853899999999996, 1.5063, 0.833137, 0.0], \
[1.09749, 0.7853899999999996, 1.5063, 0.353137, 0.0], \
[0.939273, 1.05276, 1.5063, 0.353137, 0.0],\
[0.7853899999999996, 0.7853899999999996, 1.73257, 0.04, 0.0]]

numstep = 10

global r3
r3 = [[0.7853899999999996, 0.7853899999999996, 1.73257, 0.01, 0.0], \
[0.7853899999999996, 0.86539, 1.73257, 0.0228547, 0.0], \
[0.7853899999999996, 0.86539, 1.73257, 0.702855, 0.0], \
[0.7853899999999996, 0.86539, 1.73257, 0.182855, 0.0], \
[0.795149, 0.78539, 1.73257, 0.182855, 0.0]]

global r2test
r2test = [[0.795149, 0.78539, 1.73257, 0.179641, 0.0], [0.795149, 0.78539, 1.73257, 0.45964099999999997, 0.0], [0.955149, 0.943231, 1.57257, 0.4596410000000001, 0.0], [1.11515, 1.10323, 1.49257, 0.4596410000000001, 0.0], [1.11515, 1.10323, 1.49257, 0.087461, 0.0]]

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

allpose = generatePath(r3,numstep)



class MyGLViewer(GLSimulationPlugin):
    def __init__(self,files):
        #create a world from the given files
        world = WorldModel()
        for fn in files:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
        #initialize the simulation

        GLSimulationPlugin.__init__(self,world)

        #put custom action hooks here
        self.goal = 0
        self.step = 0
        self.waypoint = []
        self.add_action(self.function_f,'Some random function','f')

        # preshape 1
        self.add_action(self.function_q,'Some random function','q')
        self.add_action(self.function_w,'Some random function','w')

        # preshape 2
        self.add_action(self.function_t,'Some random function','t')
        self.add_action(self.function_y,'Some random function','y')

        # finger 1
        self.add_action(self.function_e,'Some random function','e')
        self.add_action(self.function_d,'Some random function','d')
        # finger 2
        self.add_action(self.function_r,'Some random function','r')
        self.add_action(self.function_f,'Some random function','f')
        # finger 3
        self.add_action(self.function_u,'Some random function','u')
        self.add_action(self.function_j,'Some random function','j')

        #Control waypoint function
        self.add_action(self.function_z,'Some random function','z')
        self.add_action(self.function_n,'Linear Plan','n')
        self.add_action(self.function_p,'Some random function','p')
        self.add_action(self.function_m,'Linear Plan Step','m')

        # robot = self.world.robot(0)
        # test = robot.getConfig()
        # print "Robot Config: " , test
        # test[3] = 0.78539
        # test[8] = 0.78539
        # print "Robot Config After: " , test
        # robot.setConfig(test)
    def function_z(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        currentPoint = [q[3],q[8],q[12],q[2],q[16]]
        self.waypoint.append(currentPoint)
        print "Save current pose", currentPoint

    def function_n(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        # pose = finger1 finger2 finger3 preshapeCouple preshapeThumb
        for pose in allpose:
            q[3] = pose[0]
            q[4] = q[3]/4
            q[8] = pose[1]
            q[9] = q[8]/4
            q[12]= pose[2]
            q[13] = q[12]/4
            q[2] = pose[3]
            q[16]= pose[4]
            sim.controller(0).addMilestone(q)
        print len(allpose)
        print "Done"

    def function_m(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        # pose = finger1 finger2 finger3 preshapeCouple preshapeThumb
        # Original rotate
        # q[3] = rotateXPoints2[self.step][0]
        # q[8] = rotateXPoints2[self.step][1]
        # q[12]= rotateXPoints2[self.step][2]
        # q[2] = rotateXPoints2[self.step][3]
        # q[16]= rotateXPoints2[self.step][4]

        #6DOF rotate
        q[3] = rotateXPoints2[self.step][0]
        q[4] = q[3]/4
        q[8] = rotateXPoints2[self.step][2]
        q[9] = q[8]/4
        q[12]= rotateXPoints2[self.step][4]
        q[13] = q[12]/4
        q[2] = rotateXPoints2[self.step][1]
        q[7] = -rotateXPoints2[self.step][3]
        q[16]= rotateXPoints2[self.step][5]

        sim.controller(0).setMilestone(q)
        if (self.step < len(rotateXPoints2)-1):
            self.step = self.step + 1
        else: 
            self.step = 0 
        print "Step:", self.step
        print "Done:", len(rotateXPoints2)

    def function_p(self):
        print self.waypoint

    def function_f(self):

        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[12] +=0.08
        sim.controller(0).setMilestone(q)
    
    # preshape 1
    def function_q(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[2]+=0.08
        sim.controller(0).setMilestone(q)

    def function_w(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[2]-=0.08
        sim.controller(0).setMilestone(q)
    
    # preshape 2
    def function_t(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[16]+=0.08
        sim.controller(0).setMilestone(q)

    def function_y(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[16]-=0.08
        sim.controller(0).setMilestone(q)
    

    # finger 1
    def function_e(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[3] +=0.08
        q[4] = q[3]/4 
        sim.controller(0).setMilestone(q)

    def function_d(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[3] -=0.08
        q[4] = q[3]/4
        sim.controller(0).setMilestone(q)

    # finger 2
    def function_r(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[8] +=0.08
        q[9] = q[8]/4 
        sim.controller(0).setMilestone(q)

    def function_f(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[8] -=0.08
        q[9] = q[8]/4
        sim.controller(0).setMilestone(q)

    # finger 3
    def function_u(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[12] +=0.08
        q[13] = q[12]/4 
        sim.controller(0).setMilestone(q)

    def function_j(self):
        sim = self.sim
        q=sim.controller(0).getCommandedConfig()
        q[12] -=0.08
        q[13] = q[12]/4
        sim.controller(0).setMilestone(q)

    def control_loop(self):
        #Put your control handler here
        # sim = self.sim
        # if sim.getTime() >= 2.0 and sim.getTime()-self.dt < 2.0:
        #   q=sim.controller(0).getCommandedConfig()
        #   q[3]-=0.40
        #   sim.controller(0).setMilestone(q)
        #   q[3]+=0.45
        #   sim.controller(0).addMilestone(q)
        #   print("Testing in here")
        sim = self.sim
        
        if sim.getTime() >= 2.0 and sim.getTime()-self.dt < 2.0:
            q=sim.controller(0).getCommandedConfig()
            q[2]+=0.04
            sim.controller(0).setMilestone(q)
            q[2]+=0.04
            sim.controller(0).addMilestone(q)
            print(q)
    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2:
            if state==0:
                print [o.getName() for o in self.click_world(x,y)]
            return
        return GLSimulationPlugin.mousefunc(self,button,state,x,y)

    def motionfunc(self,x,y,dx,dy):
        return GLSimulationPlugin.motionfunc(self,x,y,dx,dy)

if __name__ == "__main__":
    print "gltemplate.py: This example demonstrates how to simulate a world and read user input"
    if len(sys.argv)<=1:
        print "USAGE: gltemplate.py [world_file]"
        exit()

    viewer = MyGLViewer(sys.argv[1:])
    vis.run(viewer)
