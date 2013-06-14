"""Get a trajectory to a grasp before executing it.
"""
from openravepy import *
import numpy, time
import numpy as np

global pr2, target


def home(basemanip):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        pr2.SetActiveDOFs([pr2.GetJoint(name).GetDOFIndex() for name in jointnames])
        basemanip.MoveActiveJoints(goal=[1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])

def env1_setup():
    #global pr2, target
    env=Environment()
    env.Load('data/pr2test1.env.xml')
    env.SetViewer('qtcoin')
    pr2 = env.GetRobots()[0]
  
    target = env.GetKinBody('mug1')
    pr2.SetActiveManipulator('leftarm')
    getGraspTraj(pr2, target)


def getGraspTraj(pr2, target, move=False):
    with pr2:
        gmodel = databases.grasping.GraspingModel(pr2, target)
        if not gmodel.load():
            gmodel.autogenerate()

        validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
        basemanip = interfaces.BaseManipulation(pr2)
        grasp = validgrasps[0]
        gmodel.setPreshape(grasp)
        T = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
        traj = basemanip.MoveToHandPosition(matrices=[T],execute=False,outputtrajobj=True)
 
        for i in xrange(traj.GetNumWaypoints()):
            raveLogInfo('traj has %d waypoints, last waypoint is: %s'%(traj.GetNumWaypoints(),repr(traj.GetWaypoint(i))))
            
            
            

        if move:
            pr2.GetController().SetPath(traj)
            time.sleep(1)
        return traj
            #pr2.WaitForController(0)

def waitrobot(robot):
    robot.WaitForController(0)

def env2():
    global pr2
    env=Environment()
    env.Load('data/pr2test2.env.xml')
    env.SetViewer('qtcoin')
    pr2 = env.GetRobots()[0]
    
    robot=pr2
    manip = robot.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm + torso
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()

    # create the interface for basic manipulation programs
    basemanip = interfaces.BaseManipulation(robot,plannername='birrt')
    taskprob = interfaces.TaskManipulation(robot,plannername='birrt')


    target=env.GetKinBody('TibitsBox1')
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        basemanip.MoveActiveJoints(goal=[1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
    waitrobot(robot)


    print 'move robot base to target'
    with env:
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        basemanip.MoveActiveJoints(goal=[2.8,-1.3,0],maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot)

    taskprob.ReleaseFingers()
    waitrobot(robot)

    print 'move the arm to the target'
    Tgoal = np.array([[0,-1,0,3.5],[-1,0,0,-1.3],[0,0,-1,0.842],[0,0,0,1]])
    res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
    waitrobot(robot)

    print 'close fingers until collision'
    taskprob.CloseFingers()
    waitrobot(robot)

    print 'move the arm with the target back to the initial position'
    with env:
        robot.Grab(target)
        basemanip.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
    waitrobot(robot)

    print 'move the robot to another location'
    with env:
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        localgoal = [0,2.4,0]
        T = robot.GetTransform()
        goal = np.dot(T[0:3,0:3],localgoal) + T[0:3,3]
        with robot:
            robot.SetActiveDOFValues(goal)
            incollision = env.CheckCollision(robot)
            if incollision:
                print 'goal in collision!!'

    basemanip.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2)
    waitrobot(robot)

    print 'move the arm to the designated position on another table to place the target down'
    Tgoal = np.array([[0,-1,0,3.5],[-1,0,0,1.5],[0,0,-1,0.855],[0,0,0,1]])
    res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
    waitrobot(robot)

    taskprob.ReleaseFingers(target=target)
    waitrobot(robot)

    print 'move manipulator to initial position'
    basemanip.MoveManipulator(goal=[0, 0, 1.29023451, 0, -2.32099996, 0, -0.69800004, 0])
    waitrobot(robot)

    print 'close fingers until collision'
    taskprob.CloseFingers()
    waitrobot(robot)
    