import generate_reaching_poses
import utils
import openravepy
import time

def get_occluding_objects_names(robot, 
                                obj,
                                body_filter,
                                num_trials = 300,
                                just_one_attempt = False,
                                return_pose = False):
    """
    Returns the names of all the objects as calculated by get_occluding_objects
    with additional filtering.
    
    Paramters:
    body_filter: a function that takes a KinBody and returns True or False.
    just_one_attempt: If True then it will return only the result of one successfull grasping attempt.
    
    Example usage:
    get_occluding_objects_names(robot, obj, lambda b:b.GetName().startswith("random"), 500)
    """
    
    if return_pose:
        (pose,
         sol, torso_angle,         
         obstacles_bodies) = get_occluding_objects(robot, obj, num_trials, just_one_attempt,
                                             return_pose)
    else:
        obstacles_bodies =  get_occluding_objects(robot, obj, num_trials, just_one_attempt,
                                             return_pose)
    openravepy.raveLogInfo("Bodies: %s" % obstacles_bodies)  
    nonempty = lambda l:len(l)>0
    obstacles = set( filter(nonempty,
                            (tuple(str(b.GetName()) 
                                  for b in l 
                                  if body_filter(b) and
                                  b.GetName() != obj.GetName()
                                  ) 
                            for l in obstacles_bodies
                            )
                            )
                     )
    if return_pose:
        return pose, sol, torso_angle, obstacles
    else:
        return obstacles
    
def get_occluding_objects(robot, 
                             object_to_grasp, 
                             max_trials = 100,
                              just_one_attempt = False,
                              return_pose = False
                             ):
    """Generates a list of all the objects that prevent the robot from reaching
    a target object. Several (up to max_trials) attempts are performed to grasp

    Parameters:
    robot: a openravepy.Robot instance
    object_to_grasp: a openravepy.KinBody instance representing the object to grasp    
    
    Returns:
    a list of sets of objects
    """
    if just_one_attempt != return_pose:
        raise ValueError("If asking for a return poes then set just_one_attempt to True")
    
    env = robot.GetEnv()
    robot_pose = robot.GetTransform()
    manip = robot.GetActiveManipulator()
    
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot,iktype=openravepy.IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()    
    min_torso, max_torso = utils.get_pr2_torso_limit(robot)

    num_trial = 0
    collisions_list = []
    with robot:
        while num_trial < max_trials:
            num_trial +=1

            # sample a base pose
            torso_angle = generate_reaching_poses.move_random_torso(robot, 
                                                                    min_torso, 
                                                                    max_torso)
            robot_pose = generate_reaching_poses.generate_random_pos(robot, 
                                                                     object_to_grasp)
            
            robot.SetTransform(robot_pose) 
            report = openravepy.CollisionReport()
            
            collision = env.CheckCollision(robot, report=report)
            
            if not collision:                
                openravepy.raveLogInfo("Got a position not in collision")
                
                #sample a gripper pose
                grasping_poses = generate_reaching_poses.generate_grasping_pose(robot,
                                                        object_to_grasp,
                                                        use_general_grasps = True,
                                                        checkik=False) 
                openravepy.raveLogInfo("Got %d grasping poses" % len(grasping_poses))
                #check if gripper pose is reachable from base pose
                #use robot's base pose to transform precomputed
                #gripper poses into the robot's frame of reference
                sol = generate_reaching_poses.check_reachable(manip, 
                                                           grasping_poses, 
                                                              only_reachable = True)
                if sol is None:
                    print "Trial {0} No sol from base pose to gripper pose".\
                        format(num_trial)
                
                if sol is not None:                  
                    print "Sol from base pose to gripper pose found in trial {0}".\
                        format(num_trial)
                    openravepy.raveLogInfo("Getting the list of collisions")
                    with robot:
                        robot.SetDOFValues(sol, robot.GetActiveManipulator().GetArmIndices());                    
                        collisions_list.append(utils.get_all_collisions(robot, env))
                        if just_one_attempt:
                            if return_pose:
                                return (robot_pose, sol, torso_angle, collisions_list)
                            else:
                                return collisions_list
        if num_trial == max_trials:
            print "No gripper pose reachable from collision free base pose found",
            print "after {0} trials".format(num_trial)
            

    return collisions_list

def predicates(target, occlusions_sets, initial_number=0):
    strs = []
    i = initial_number
    for occ_tuple in occlusions_sets:
        pos = "p%d" %i
        for occ in occ_tuple:
            strs.append("obs(%s,%s,%s)" % (pos, occ, target))
        i +=1
            
    return i, "\n".join(strs)
        
    
