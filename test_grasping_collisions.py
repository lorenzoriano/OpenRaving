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

def generate_random_pos(robot, obj_to_grasp = None):
    """Generate a random position for the robot within the boundaries of the 
    world.
    """
    
    T = robot.GetTransform()
    if obj_to_grasp is None:
        max_x = 2.5
        max_y = 2.5
        max_th = np.pi
    else:
        obj_pos = obj_to_grasp.GetTransform()[:3,-1]
        max_x = obj_pos[0] - 1.0
        max_y = obj_pos[1] - 1.0
    
    
    x = np.random.uniform(-max_x, max_x)
    y = np.random.uniform(-max_y, max_y)
    z = T[2,3]
    
    if obj_to_grasp is None:
        th = np.random.uniform(-max_th, max_th)
    else:
        robot_pos = T[:3,-1]
        th = np.arctan2(obj_pos[1] -y , obj_pos[0] - x)
    
    #rotation
    T = openravepy.matrixFromAxisAngle([0,0,th])
    #translation
    T[:, -1] = [x, y, z, 1]    
    return T


def check_reachable(manip, grasping_poses):
    """Check if the robot can reach a pose p (4x4 matrix). 
    """
    
    for pose in grasping_poses:
        sol = manip.FindIKSolution(pose, 
                               openravepy.IkFilterOptions.IgnoreEndEffectorCollisions |
                               openravepy.IkFilterOptions.CheckEnvCollisions)
        if sol is not None:
            return sol
    return None

class CustomIkFilter(object):
    def __init__(self, object_to_collide_with):
        self.report = None
        self.object_to_collide_with = object_to_collide_with.GetName()
        
    def __call__(self, values, manip, ikparam):
        if self.report.plink2 is None:
            return openravepy,IkReturnAction.Success
        
        collision = self.report.plink2.GetPartent().GetName()
        openravepy.raveLogInfo("Report says collision with %s" % collision)
        if collision == self.object_to_collide_with:
            return openravepy,IkReturnAction.Success
        else:
            return openravepy.IkReturnAction.Reject

def main():
    """Loads an environment and generates random positions until the robot can
    reach an oject from a collision-free pose.
    """
    
    env = openravepy.Environment()    
    env.Load('data/pr2test1.env.xml')
    robot=env.GetRobots()[0]
    manip = robot.SetActiveManipulator('rightarm')

    mug = env.GetKinBody('mug1')
    
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
        robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    
    num_contacts = 1 #just cheating
    isreachable = False
    env.SetViewer('qtcoin')
    num_trials = 1
    sol = None
    
    grasping_poses = generate_grasping_pose(robot, mug)
    openravepy.raveLogInfo("I've got %d grasping poses" % len(grasping_poses))
    env.GetCollisionChecker().SetCollisionOptions(openravepy.CollisionOptions.Contacts)

    num_contacts = 1 #just cheating
    isreachable = False
    
    while (num_contacts > 0) or (not isreachable):
        num_trials +=1        
        pose = generate_random_pos(robot, mug) 
        robot.SetTransform(pose)    
            
        # get first collision
        report = openravepy.CollisionReport()
        collision = env.CheckCollision(robot,report=report)
        num_contacts = len(report.contacts)
        
        sol = check_reachable(manip, grasping_poses)
        isreachable = sol is not None

        openravepy.raveLogInfo("Testing pose number %d. Reachable? %s. Contacts? %d" % 
                               (num_trials, isreachable, num_contacts))        
        
        if not collision and isreachable:
            break
        
        time.sleep(0.01)
    
    sol = check_reachable(manip, grasping_poses)
    if sol is not None:
        robot.SetDOFValues(sol, manip.GetArmIndices())
        openravepy.raveLogInfo("Done!!")
    else:
        openravepy.raveLogInfo("No solution????")
    

if __name__ == "__main__":
    main()
    raw_input("Press a button to continue")

    