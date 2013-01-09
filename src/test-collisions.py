#! /usr/bin/python

import openravepy
import time
import numpy as np

def make_orth_basis(x_ax):
    """
    John Schulman magic code.
    """
    x_ax = np.asarray(x_ax)

    x_ax = x_ax / np.linalg.norm(x_ax)
    if np.allclose(x_ax, [1,0,0]):
        return np.eye(3)
    elif np.allclose(x_ax, [-1, 0, 0]):
        return np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1]])
    else:
        y_ax = np.r_[0, x_ax[2], -x_ax[1]]
        y_ax /= np.linalg.norm(y_ax)
        z_ax = np.cross(x_ax, y_ax)
        return np.c_[x_ax, y_ax, z_ax]

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

def check_reachable(robot, manip, obj):
    """Check if the robot can reach an object. The object is reachable if the 
    manipulator can be placed over it (with no collision checking).
        
    Right now the target angles are the angles between the manipulator and the
    object, but this is not correct!
    """
    
    #all of this is to get the angles between the manipulator and the object
    obj_pos = obj.GetTransform()[:3,-1]
    robot_pos = robot.GetTransform()[:3,-1]    
    rot_mat = make_orth_basis(obj_pos - robot_pos)
    
    #Create the target position
    T = np.eye(4)
    T[:3, :-1] = rot_mat
    T[:3,-1] = obj_pos
    sol = manip.FindIKSolution(T, openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
    return sol
    

def main():
    """Loads an environment and generates random positions until the robot can
    reach an oject from a collision-free pose.
    """
    
    env = openravepy.Environment()    
    env.Load('data/pr2test1.env.xml')
    robot=env.GetRobots()[0]
    mug = env.GetKinBody('mug1')
    
    manip = robot.SetActiveManipulator('leftarm_torso')
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
        robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    
    num_contacts = 1 #just cheating
    isreachable = False
    while (num_contacts > 0) or (not isreachable):
        T = generate_random_pos(robot, mug)    
        robot.SetTransform(T)    
        env.GetCollisionChecker().SetCollisionOptions(openravepy.CollisionOptions.Contacts)
            
        # get first collision
        report = openravepy.CollisionReport()
        collision=env.CheckCollision(robot,report=report)
        num_contacts = len(report.contacts)
        openravepy.raveLogInfo("Number of contacts: " + str(num_contacts))
        sol = check_reachable(robot, manip, mug)
        isreachable = sol is not None
        #time.sleep(1)
    
    robot.SetDOFValues(sol, manip.GetArmIndices())
    openravepy.raveLogInfo("Done!!")
    env.SetViewer('qtcoin')

if __name__ == "__main__":
    main()
    while True:
        time.sleep(0.1)
    