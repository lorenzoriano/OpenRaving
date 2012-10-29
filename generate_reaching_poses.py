#! /usr/bin/python
import openravepy
import time
import numpy as np

import utils

def generate_manip_above_surface(obj, num_poses = 20):
    
    gripper_angle = (np.pi, 0., 0) #just got this from trial and test
    rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)
    
    ab = obj.ComputeAABB()
    max_x = ab.pos()[0] + ab.extents()[0]
    min_x = ab.pos()[0] - ab.extents()[0]
    
    max_y = ab.pos()[1] + ab.extents()[1]
    min_y = ab.pos()[1] - ab.extents()[1] 
    
    gripper_height = 0.18
    z = ab.pos()[2] + ab.extents()[2] + gripper_height + 0.03
    
    poses = []
    for _ in range(num_poses):
        x = np.random.uniform(min_x, max_x)
        y = np.random.uniform(min_y, max_y)
        
        T = np.eye(4)
        T[:3,:3] = rot_mat
        T[:3,3] = [x,y,z]
        yield T

def generate_grasping_pose(robot, obj_to_grasp, use_general_grasps=True):
    """Generator function with all the valid grasps for object obj_to_grasp.
    """        
    
    class __Options(object):
        def __init__(self):
            self.delta=0.1
            self.normalanglerange=0.5
            self.standoffs = [0]
            #self.rolls = np.arange(0,2*np.pi,0.5*np.pi)
            self.rolls = np.arange(0,np.pi,0.5*np.pi)
            self.directiondelta = 0.4
            pass
            
    gmodel = openravepy.databases.grasping.GraspingModel(robot, obj_to_grasp)
    
    if use_general_grasps:
        gmodel.grasps = utils.pre_grasps
    
    else:
        if not gmodel.load():
            openravepy.raveLogInfo("Generating grasping model...")
            gmodel.autogenerate(__Options())

    openravepy.raveLogInfo("Generating grasps")
    validgrasps, _ = gmodel.computeValidGrasps(checkcollision=False, 
                                               checkik = True,
                                               checkgrasper = False)
    openravepy.raveLogInfo("Number of valid grasps: %d" % len(validgrasps))
    return [gmodel.getGlobalGraspTransform(grasp) for grasp in validgrasps]

def generate_random_pos(robot, obj_to_grasp = None):
    """Generate a random position for the robot within the boundaries of the 
    world.
    """
    
    T = robot.GetTransform()
    if obj_to_grasp is None:
        envmin, envmax = utils.get_environment_limits(robot.GetEnv(), robot)
        max_x = envmax[0]
        max_y = envmax[1]
        max_th = np.pi
        min_x = envmin[0]
        min_y = envmin[1]
        min_th = np.pi        
    else:
        obj_pos = obj_to_grasp.GetTransform()[:3,-1]
        max_x = obj_pos[0] + 1.2
        max_y = obj_pos[1] + 1.2
        min_x = obj_pos[0] - 1.2
        min_y = obj_pos[1] - 1.2        
    
    #print "X ", (min_x, max_x), " Y: ", (min_y, max_y)
    x = np.random.uniform(min_x, max_x)
    y = np.random.uniform(min_y, max_y)
    z = T[2,3]
    
    if obj_to_grasp is None:
        th = np.random.uniform(min_th, max_th)
    else:
        robot_pos = T[:3,-1]
        facing_angle = np.arctan2(obj_pos[1] -y , obj_pos[0] - x)
        th = np.random.uniform(facing_angle - np.pi/4., 
                               facing_angle + np.pi/4.)
    
    #rotation
    T = openravepy.matrixFromAxisAngle([0,0,th])
    #translation
    T[:, -1] = [x, y, z, 1]    
    return T


def check_reachable(manip, grasping_poses):
    """Check if the robot can reach at least one pose 

    Parameters:
    manip: a Manipulator instance
    grasping_poses: a list of (4x4 matrix). 
    """
    
    for pose in grasping_poses:
        sol = manip.FindIKSolution(pose, 
                               openravepy.IkFilterOptions.IgnoreEndEffectorCollisions |
                               openravepy.IkFilterOptions.CheckEnvCollisions)
        if sol is not None:
            return sol
    return None

def get_collision_free_grasping_pose(robot, object_to_grasp, 
                                         max_trials = 100,
                                         use_general_grasps = True):
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    manip = robot.GetActiveManipulator()
    
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()    

    grasping_poses = generate_grasping_pose(robot, object_to_grasp,
                                            use_general_grasps)
    openravepy.raveLogInfo("I've got %d grasping poses" % len(grasping_poses))
    env.GetCollisionChecker().SetCollisionOptions(openravepy.CollisionOptions.Contacts)    
    
    collision = env.CheckCollision(robot)
    sol = check_reachable(manip, grasping_poses)
    isreachable = sol is not None
    min_torso, max_torso = utils.get_pr2_torso_limit(robot)
    
    num_trial = 0
    with robot:
        while ((collision) or (not isreachable)) and (num_trial < max_trials):
            num_trial +=1
            torso_angle = move_random_torso(robot, min_torso, max_torso)
            robot_pose = generate_random_pos(robot, object_to_grasp)
            
            robot.SetTransform(robot_pose) 
            report = openravepy.CollisionReport()
            collision = env.CheckCollision(robot, report=report)
            
            if not collision:
                grasping_poses = generate_grasping_pose(robot, object_to_grasp,
                                                        use_general_grasps)                
                sol = check_reachable(manip, grasping_poses)
                isreachable = sol is not None
            else:
                continue

    if (sol is None) or collision:
        raise ValueError("No collision free grasping pose found within %d steps" % max_trials)    
    else:
        return (robot_pose, sol, torso_angle)

def move_random_torso(robot, min_angle, max_angle, joint_index=[]):
    if joint_index == []:
        joint_index.append(robot.GetJointIndex('torso_lift_joint'))
    
    angle = np.random.uniform(min_angle, max_angle)
    robot.SetDOFValues([angle],  joint_index)
    return angle

def get_collision_free_surface_pose(robot, obj, 
                                         max_trials = 100,
                                         use_general_grasps = True):
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    manip = robot.GetActiveManipulator()
    
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()    

    grasping_poses = generate_manip_above_surface(obj)
    env.GetCollisionChecker().SetCollisionOptions(openravepy.CollisionOptions.Contacts)    
    
    collision = env.CheckCollision(robot)
    sol = check_reachable(manip, grasping_poses)
    isreachable = sol is not None
    
    min_torso, max_torso = utils.get_pr2_torso_limit(robot)
    
    num_trial = 0
    with robot:
        while ((collision) or (not isreachable)) and (num_trial < max_trials):
            num_trial +=1
            torso_angle = move_random_torso(robot, min_torso, max_torso)
            robot_pose = generate_random_pos(robot, obj)
            
            robot.SetTransform(robot_pose)
            report = openravepy.CollisionReport()
            collision = env.CheckCollision(robot, report=report)
            
            if not collision:
                grasping_poses = generate_manip_above_surface(obj)
                sol = check_reachable(manip, grasping_poses)
                isreachable = sol is not None
            else:
                continue

    if (sol is None) or collision:
        raise ValueError("No collision free grasping pose found within %d steps" % max_trials)    
    else:
        return (robot_pose, sol, torso_angle)

def main():
    """Loads an environment and generates random positions until the robot can
    reach an oject from a collision-free pose.
    """
    
    env = openravepy.Environment()    
    env.Load('data/pr2test1.env.xml')
    env.SetViewer('qtcoin')
    robot=env.GetRobots()[0]
    manip = robot.SetActiveManipulator('rightarm')
    
    pose = generate_random_pos(robot) 
    robot.SetTransform(pose)        

    obj = env.GetKinBody('mug1')
    #env.GetKinBody('obstacle').SetVisible(False)
    
    try:
        pose, sol = get_collision_free_grasping_pose(robot,
                                                     obj,
                                                     1000)
        robot.SetTransform(pose)
        robot.SetDOFValues(sol, manip.GetArmIndices())        
        openravepy.raveLogInfo("Done!!")        
    except ValueError, e:
        openravepy.raveLogError("Error while trying to find a pose: %s" % e)
        return
        
if __name__ == "__main__":
    main()
    raw_input("Press a button to continue")

    
    