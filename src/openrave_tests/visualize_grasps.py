from PlannerPR2 import PlannerPR2
import rospy
import time
import openravepy as rave
import numpy as np
import numpy.linalg as la

global PR2
global env

PR2 = None
env = None

global handles
handles = []


def plot_transform(T, s=0.1):
    h = []
    x = T[0:3,0]
    y = T[0:3,1]
    z = T[0:3,2]
    o = T[0:3,3]
    h.append(env.drawlinestrip(points=np.array([o, o+s*x]), linewidth=3.0, colors=np.array([(1,0,0),(1,0,0)])))
    h.append(env.drawlinestrip(points=np.array([o, o+s*y]), linewidth=3.0, colors=np.array(((0,1,0),(0,1,0)))))
    h.append(env.drawlinestrip(points=np.array([o, o+s*z]), linewidth=3.0, colors=np.array(((0,0,1),(0,0,1)))))
    return h


def visualize(rave_pr2, target, rl='l', exclude_angle=53):
    """
    Visualizes the grasps found by openrave
    and filters vertical grasps :
 
        EXCLUDE_ANGLE  : is the angle in degrees along the vertical
        of the region in which grasps are to be excluded.
        It should be in [0,91]. (Be careful about numerical precision :P)
    
        exclude_angle = this angle
         v
        \  |  /
         \ | / 
          \|/
   accept /|\ accepted
         / | \
        /  |  \
    """

    # get a goal grasp transform
    arm_name = '%sarm'% ('left' if rl=='l' else 'right')
    rave_pr2.SetActiveManipulator(arm_name)

    gmodel = rave.databases.grasping.GraspingModel(rave_pr2, target)
    if not gmodel.load():
        gmodel.autogenerate()

    validgrasps, validindicees = gmodel.computeValidGrasps()  
    manip = rave_pr2.GetActiveManipulator()
    
    d_thresh = np.cos(np.pi*exclude_angle/180.0)

    print "found %d grasps" % len(validgrasps)
    for i,grasp in enumerate(validgrasps):
        print i
        gmodel.setPreshape(grasp)

        # filter vertical grasps
        goal_tfm    = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
        zz = goal_tfm[2,2]
        if abs(zz) >= d_thresh: 
            print "vertical grasp ... ignoring"
            continue
        
        goal_joints = gmodel.manip.FindIKSolution(goal_tfm, filteroptions=rave.IkFilterOptions.CheckEnvCollisions)
        rave_pr2.SetJointValues(goal_joints,manip.GetArmIndices())
        h = plot_transform(goal_tfm)
        raw_input()
        

if __name__=="__main__":
    global env
    env = rave.Environment()
    env.Load('data/pr2test1.env.xml')
    env.SetViewer('qtcoin')
    rave_pr2 = env.GetRobots()[0]
    target = env.GetKinBody('mug1')
    visualize(rave_pr2, target)
    
