#! /usr/bin/python
import openravepy
import time
import numpy as np

import reachability
import utils
import sys

class GraspingPoseError(Exception):
    pass

def generate_manip_above_surface(obj, num_poses = 20):
    
    gripper_angle = (np.pi, 0., 0) #just got this from trial and test
    rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)
    
    min_x, max_x, min_y, max_y, z = utils.get_object_limits(obj)
    
    gripper_height = 0.18
    z += gripper_height + 0.03
    
    poses = []
    for _ in range(num_poses):
        x = np.random.uniform(min_x, max_x)
        y = np.random.uniform(min_y, max_y)
        
        T = np.eye(4)
        T[:3,:3] = rot_mat
        T[:3,3] = [x,y,z]
        poses.append(T)
    
    return poses

def generate_grasping_pose(robot, 
                           obj_to_grasp, 
                           use_general_grasps=True,
                           checkik = True):
    """Returns a list with all the valid grasps for object obj_to_grasp.
    
    Parameters:
    robot: an OpenRave robot
    obj_to_grasp: a KinBody that the robot should grasp
    use_general_grasps: if True, don't calculate actual grasp points, but use
     a pre-generated list. It is much faster if a grasping model has not been
     generated.
    checkik: passed to GraspingModel.computeValidGrasps
    
    Returns:
    a list of Transformation matrices, one for each valid grasping pose
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
                                               checkik = checkik,
                                               checkgrasper = False)
    openravepy.raveLogInfo("Number of valid grasps: %d" % len(validgrasps))
    return [gmodel.getGlobalGraspTransform(grasp) for grasp in validgrasps]

def generate_random_pos(robot, obj_to_grasp = None):
    """Generate a random position for the robot within the boundaries of the 
    world.
    """
    
    T = robot.GetTransform()
    envmin, envmax = utils.get_environment_limits(robot.GetEnv(), robot)
    if obj_to_grasp is None:        
        max_x = envmax[0]
        max_y = envmax[1]
        max_th = np.pi
        min_x = envmin[0]
        min_y = envmin[1]
        min_th = np.pi        
    else:
        if type(obj_to_grasp) is openravepy.KinBody:
            obj_pos = obj_to_grasp.GetTransform()[:3,-1]
        else:
            obj_pos = obj_to_grasp
        max_x = min(obj_pos[0] + 1.2, envmax[0])
        max_y = min(obj_pos[1] + 1.2, envmax[1])
        min_x = max(obj_pos[0] - 1.2, envmin[0])
        min_y = max(obj_pos[1] - 1.2, envmin[1])
        
    
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


def check_reachable(manip, grasping_poses, only_reachable = False):
    """Check if the robot can reach at least one pose 

    Parameters:
    manip: a Manipulator instance
    grasping_poses: a list of (4x4 matrix). 
    only_reachable: do not check for collisions
    """
    
    if len(grasping_poses) == 0:
        return None
    if only_reachable:
        options = (openravepy.IkFilterOptions.IgnoreEndEffectorCollisions 
                   )
    else:
        options = (openravepy.IkFilterOptions.IgnoreEndEffectorCollisions |
                   openravepy.IkFilterOptions.CheckEnvCollisions)
    for pose in grasping_poses:
        sol = manip.FindIKSolution(pose, 
                                   options
                                   )
        if sol is not None:
            return sol
    return None

def get_collision_free_grasping_pose(robot,
                                     object_to_grasp, 
                                     max_trials = 100,
                                     use_general_grasps = True,
                                     ):
    """Returns the position from where the robot can grasp an object.
    
    Parameters:
    robot: an OpenRave robot instance
    object_to_grasp: a KinBody that the robot should grasp
    max_trials: how many attempts before giving up
    use_general_grasps: f True, don't calculate actual grasp points, but use
     a pre-generated list. It is much faster if a grasping model has not been
     generated.
     
    Returns:
    (robot_pose, sol, torso_angle): the robot position (as a transformation matrix),
    the active manipulator angles and the torso joint angle from where the robot
    can grasp an object.
    
    Raises GraspingPoseError if no valid solution is found.
    """
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    torso_angle = robot.GetJoint("torso_lift_joint").GetValues()[0]
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
                grasping_poses = generate_grasping_pose(robot, 
                                                        object_to_grasp,
                                                        use_general_grasps)                
                sol = check_reachable(manip, grasping_poses)
                isreachable = sol is not None                
            else:
                continue

    if (sol is None) or collision:
        e = GraspingPoseError("No collision free grasping pose found within %d steps" % max_trials)    
        raise e
    else:
        return (robot_pose, sol, torso_angle)

def get_collision_free_ik_pose(robot,
                                     ik_pose, 
                                     max_trials = 100,
                                     only_reachable=False
                                     ):
    """Returns the position from where the robot can reach a position (in
    cartesian coordinates). The active manipulator is used.
    
    Parameters:
    robot: an OpenRave robot instance
    ik_pose: a 4x4 matrix with the desired 6D pose
    max_trials: how many attempts before giving up
    use_general_grasps: f True, don't calculate actual grasp points, but use
     a pre-generated list. It is much faster if a grasping model has not been
     generated.
     
    Returns:
    (robot_pose, sol, torso_angle): the robot position (as a transformation matrix),
    the active manipulator angles and the torso joint angle from where the robot
    can grasp an object.
    
    Raises GraspingPoseError if no valid solution is found.
    """
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    torso_angle = robot.GetJoint("torso_lift_joint").GetValues()[0]
    manip = robot.GetActiveManipulator()
    
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    
    env.GetCollisionChecker().SetCollisionOptions(openravepy.CollisionOptions.Contacts)    
    
    collision = env.CheckCollision(robot)
    sol = check_reachable(manip, [ik_pose], only_reachable)
    isreachable = sol is not None
    min_torso, max_torso = utils.get_pr2_torso_limit(robot)
    
    num_trial = 0
    xyz = ik_pose[:3, 3]
    with robot:
        while ((collision) or (not isreachable)) and (num_trial < max_trials):
            num_trial +=1
            torso_angle = move_random_torso(robot, min_torso, max_torso)
            robot_pose = generate_random_pos(robot, xyz)
            
            robot.SetTransform(robot_pose) 
            report = openravepy.CollisionReport()
            collision = env.CheckCollision(robot, report=report)
            
            if not collision:
                sol = check_reachable(manip, [ik_pose], only_reachable)
                isreachable = sol is not None                
            else:
                continue

    if (sol is None) or collision:
        e = GraspingPoseError("No collision free IK pose found within %d steps" % max_trials)    
        raise e
    else:
        return (robot_pose, sol, torso_angle)

def get_torso_grasping_pose(robot,
                                     object_to_grasp, 
                                     max_trials = 100,
                                     use_general_grasps = True,
                                     ):
    """Returns the torso height from where the robot can grasp an object.
    
    Parameters:
    robot: an OpenRave robot instance
    object_to_grasp: a KinBody that the robot should grasp
    max_trials: how many attempts before giving up
    use_general_grasps: f True, don't calculate actual grasp points, but use
     a pre-generated list. It is much faster if a grasping model has not been
     generated.
     
    Returns:
    (sol, torso_angle): ,
    the active manipulator angles and the torso joint angle from where the robot
    can grasp an object.
    
    Raises GraspingPoseError if no valid solution is found.
    """
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    torso_angle = robot.GetJoint("torso_lift_joint").GetValues()[0]
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
            
            report = openravepy.CollisionReport()
            collision = env.CheckCollision(robot, report=report)
            
            if not collision:
                grasping_poses = generate_grasping_pose(robot, 
                                                        object_to_grasp,
                                                        use_general_grasps)                
                sol = check_reachable(manip, grasping_poses)
                isreachable = sol is not None                
            else:
                continue

    if (sol is None) or collision:
        e = GraspingPoseError("No collision free grasping pose found within %d steps" % max_trials)    
        raise e
    else:
        return (sol, torso_angle)

def move_random_torso(robot, min_angle, max_angle, joint_index=[]):
    if joint_index == []:
        joint_index.append(robot.GetJointIndex('torso_lift_joint'))
    
    angle = np.random.uniform(min_angle, max_angle)
    robot.SetDOFValues([angle],  joint_index)
    return angle

def get_torso_surface_pose(robot, obj, 
                                         max_trials = 100,
                                         use_general_grasps = True):
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    torso_angle = robot.GetJoint("torso_lift_joint").GetValues()[0]
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
            
            report = openravepy.CollisionReport()
            collision = env.CheckCollision(robot, report=report)
            
            if not collision:
                grasping_poses = generate_manip_above_surface(obj)
                sol = check_reachable(manip, grasping_poses)
                isreachable = sol is not None
            else:
                continue

    if (sol is None) or collision:
        raise GraspingPoseError("No collision free putdown pose found within %d steps" % max_trials)    
    else:
        return (sol, torso_angle)

def get_collision_free_surface_pose(robot, obj, 
                                         max_trials = 100,
                                         use_general_grasps = True):
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    torso_angle = robot.GetJoint("torso_lift_joint").GetValues()[0]
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
        raise GraspingPoseError("No collision free grasping pose found within %d steps" % max_trials)    
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
        pose, sol, torso= get_collision_free_grasping_pose(robot,
                                                     obj,
                                                     100)
        robot.SetTransform(pose)
        robot.SetDOFValues(sol, manip.GetArmIndices())        
        openravepy.raveLogInfo("Done!!")        
    except ValueError, e:
        openravepy.raveLogError("Error while trying to find a pose: %s" % e)
        return

def generate_all_obstructions(env = None):
    """Loads an environment and generates, for each object, the list of obstructions
    to reach it (if they exist).
    """
    import settings
    
    if env is None:
        env = openravepy.Environment()    
        env.Load('boxes.dae')
    elif type(env) is str:
        filename = env
        env = openravepy.Environment()    
        env.Load(filename)
        
    #env.SetViewer('qtcoin')
    robot=env.GetRobots()[0]
    utils.pr2_tuck_arm(robot)
    manip = robot.SetActiveManipulator('rightarm')
    objects = [b
               for b in env.GetBodies()
               if b.GetName().startswith("random_")]
    
    obstructions_text = []
    position_index = 0
    for obj in objects:        
        #trying to grasp
        print "Testing object ", obj
        try:
            get_collision_free_grasping_pose(
                robot, 
                obj,
                max_trials=settings.collision_free_grasping_samples
                )
            print "Object ", obj, "is graspable"
        except GraspingPoseError:
            print "Object ", obj, "is NOT graspable, getting occlusions"
            collision_list = reachability.get_occluding_objects_names(robot,
                                        obj,
                                        lambda b:b.GetName().startswith("random"),
                                        settings.occluding_objects_grasping_samples,
                                        just_one_attempt=False)
            for coll in collision_list:
                for obstr in coll:
                    s =  "(Obstructs p%d %s %s)" %(position_index,
                                                           obstr, obj.GetName())
                    obstructions_text.append(s)
                position_index += 1

        print "\n\n\n"
        print "\n".join(obstructions_text)
        
    
        
if __name__ == "__main__":
    env_filename = None
    if len(sys.argv) == 2:
        env_filename = sys.argv[1]
    
    generate_all_obstructions(env_filename)
    #raw_input("Press a button to continue")

    
    
