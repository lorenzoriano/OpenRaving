#! /usr/bin/python

import openravepy
import time
import numpy as np


def generate_grasping_pose(robot, obj_to_grasp):
    """Generator function with all the valid grasps for object obj_to_grasp.
    """        
    gmodel = openravepy.databases.grasping.GraspingModel(robot, obj_to_grasp)
    if not gmodel.load():
        openravepy.raveLogInfo("Generating grasping model...")
        gmodel.autogenerate()
    
    validgrasps, _ = gmodel.computeValidGrasps(checkik = False)
    openravepy.raveLogInfo("Number of valid grasps: %d" % len(validgrasps))
    return [gmodel.getGlobalGraspTransform(grasp) for grasp in validgrasps]

def generate_pose_to_reach(env, robot, obj_to_grasp, tries_per_grasp = 100,
                           grasp_marker = None):
    """Generator function with all the poses from which obj_to_grasp can be
    grasped by robot."""
            
    irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(robot)
    openravepy.raveLogInfo("Loading inverse reachability model")
    if not irmodel.load():
        raise ValueError("Inverse reachibility model not loaded, did you generate it?")    
    
    all_grasps = generate_grasping_pose(robot, obj_to_grasp)
    T_ing = robot.GetActiveManipulator().GetTransform()
    for i, Tgrasp in enumerate(all_grasps):
        openravepy.raveLogInfo("Testing grasp %d" % i)
        if grasp_marker is not None:
            grasp_marker.SetTransform(Tgrasp)
        
        #grasp_pose = np.dot(T_ing, Tgrasp)        
        grasp_pose = Tgrasp
        densityfn, samplerfn, bounds = irmodel.computeBaseDistribution(grasp_pose,
                                                                       logllthresh=-100)
        if samplerfn is None:
            openravepy.raveLogWarn("No grasp!")
            continue
        poses, jointstate = samplerfn(tries_per_grasp)
        for p in poses:
            yield p

def check_reachable(manip, pose):
    """Check if the robot can reach a pose p (4x4 matrix). 
    """
    
    sol = manip.FindIKSolution(pose, 
                               openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
    return sol
    

def main():
    """Loads an environment and generates random positions until the robot can
    reach an oject from a collision-free pose.
    """
    
    env = openravepy.Environment()    
    env.Load('data/pr2test1.env.xml')
    robot=env.GetRobots()[0]
    manip = robot.SetActiveManipulator('rightarm')

    v = robot.GetActiveDOFValues()
    v[robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()]= 3.14/2
    v[robot.GetJoint('r_shoulder_pan_joint').GetDOFIndex()] = -3.14/2
    v[robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = .54
    robot.SetActiveDOFValues(v)    
    
    mug =[b for b in env.GetBodies() if b.GetName() == 'mug1'][0]
    
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
        robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    
    num_contacts = 1 #just cheating
    isreachable = False
    env.SetViewer('qtcoin')
    num_trials = 1
    sol = None
    
    grasp_marker = openravepy.RaveCreateKinBody(env,'')
    grasp_marker.SetName("ciccio")
    grasp_marker.InitFromBoxes(np.array([[0,0,0,0.05,0.05,0.05]]),True)
    env.Add(grasp_marker, True)
    
    for pose in generate_pose_to_reach(env, robot, mug, 
                                       grasp_marker=grasp_marker):        
        num_trials +=1
        robot.SetTransform(pose)    
        env.GetCollisionChecker().SetCollisionOptions(openravepy.CollisionOptions.Contacts)
            
        # get first collision
        #report = openravepy.CollisionReport()
        #collision = env.CheckCollision(robot,report=report)
        #num_contacts = len(report.contacts)

        num_contacts = 0
        #openravepy.raveLogInfo("Number of contacts: " + str(num_contacts))
        sol = check_reachable(manip, pose)
        isreachable = sol is not None
        
        if num_contacts == 0 and isreachable:
            break
        openravepy.raveLogInfo("Testing pose number %d. Reachable? %s" % 
                               (num_trials, isreachable))
        time.sleep(0.001)
    
    if sol is not None:
        robot.SetDOFValues(sol, manip.GetArmIndices())
        openravepy.raveLogInfo("Done!!")
    

if __name__ == "__main__":
    main()
    raw_input("Press a button to continue")

    