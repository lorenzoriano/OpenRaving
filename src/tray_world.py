import numpy as np
import openravepy
import generate_reaching_poses
import utils

def recognize_random_object(obj):
    if type(obj) is not str:
        obj = obj.GetName()
    
    if obj.startswith("random_object1"):
        return "bowl"
    elif obj.startswith("random_object2"):
        return "mug"    
    elif obj.startswith("random_object3"):
        return "plate"
    else:
        raise ValueError("Unknown object type: "+obj)

def can_stack(stacktop, newobj):
    stacktop = recognize_random_object(stacktop)
    newobj = recognize_random_object(newobj)
    
    if stacktop == "mug":
        return False
    elif stacktop == "plate" :
        return True
    elif stacktop == "bowl" and newobj == "plate":
        return False
    elif newobj == "plate":
        return False
    else:
        return True
        

tray_destination =  np.array([[  1.00000000e+00,  -2.32949609e-15,  -2.56425998e-17,
         -2.61714161e+00],
       [ -2.56425998e-17,  -1.42108547e-14,  -1.00000000e+00,
          1.08424306e-01],
       [  2.32949609e-15,   1.00000000e+00,  -1.42108547e-14,
          7.44705533e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

tray_initial_loc = np.array([[  1.00000000e+00,  -3.15748247e-15,   3.94430453e-31,
          3.47457995e+00],
       [  1.38050658e-30,   2.22044605e-16,  -1.00000000e+00,
         -9.65913272e-01],
       [  3.15748247e-15,   1.00000000e+00,   2.22044605e-16,
          7.44705319e-01],
       [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
          1.00000000e+00]])

def put_right_arm_over_tray(robot, tray):
    """Move the robot's right arm so that it's about to grasp the right edge
    of the tray. Raises a generate_reaching_poses.GraspingPoseError if no
    IK solution can be found.
    
    Parameters:
    robot: a Robot instance
    tray: a KinBody instance
    """
    manip = robot.GetManipulator('rightarm')
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
                robot,iktype=openravepy.IkParameterization.Type.Transform6D)    
    if not ikmodel.load():
        ikmodel.autogenerate()    
    
    min_x, max_x, min_y, max_y, z = utils.get_object_limits(tray)
    z -= 0.06
    x = min_x + (max_x - min_x)/2
    y = min_y + 0.02
    gripper_angle = (np.pi, 0., 0) #pointing downward
    rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)
    
    T = np.eye(4)    
    T[:3,:3] = rot_mat
    T[:3,3] = [x,y,z]
    
    sol = generate_reaching_poses.check_reachable(env, tray, manip, [T], False)
    if sol is not None:
        robot.SetDOFValues(sol, manip.GetArmIndices())
        #opening gripper
        robot.SetDOFValues([0.2], manip.GetGripperJoints())        
    else:
        raise generate_reaching_poses.GraspingPoseError("No IK solution for tray right side")
        
def put_left_arm_over_tray(robot, tray):
    """Move the robot's left arm so that it's about to grasp the right edge
    of the tray. Raises a generate_reaching_poses.GraspingPoseError if no
    IK solution can be found.
    
    Parameters:
    robot: a Robot instance
    tray: a KinBody instance
    """
    manip = robot.GetManipulator('leftarm')
    ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
                robot,iktype=openravepy.IkParameterization.Type.Transform6D)    
    if not ikmodel.load():
        ikmodel.autogenerate()    
    
    min_x, max_x, min_y, max_y, z = utils.get_object_limits(tray)
    z -= 0.06
    x = min_x + (max_x - min_x)/2
    y = max_y - 0.02
    gripper_angle = (np.pi, 0., 0) #pointing downward
    rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)
    
    T = np.eye(4)    
    T[:3,:3] = rot_mat
    T[:3,3] = [x,y,z]
    
    sol = generate_reaching_poses.check_reachable(env, tray, manip, [T], False)
    if sol is not None:
        robot.SetDOFValues(sol, manip.GetArmIndices())
        #opening gripper
        robot.SetDOFValues([0.2], manip.GetGripperJoints())
    else:
        raise generate_reaching_poses.GraspingPoseError("No IK solution for tray right side")
    
def move_robot_base_infront_tray(robot, tray, execute=True):
    """Move the robot so that it is facing the tray. This means that the tray x and
    y position will be positive in respect to the robot's. Orientation will be fixed.
    """
    
    robotT = robot.GetTransform()
    if type(tray) is openravepy.KinBody:
        trayT = tray.GetTransform()
    else:
        trayT = tray
    
    robotT[:3, :3] = np.eye(3)
    #changing x
    robotT[0, 3] = trayT[0, 3] - 0.6197117799999998
    #changing y
    robotT[1, 3] = trayT[1, 3]
    #z is unchanged
    if execute:
        robot.SetTransform(robotT)
    return robotT
    
def tray_putdown_pose(tray, stack_of_items = None):
    """Returns a position right above the tray with the gripper pointing down.
    """
    T = tray.GetTransform()
    
    gripper_angle = (np.pi, 0., 0) #pointing downward
    rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)
    T[:3,:3] = rot_mat
    
    #fixing the height
    correction = 0.1
    if stack_of_items is not None:
        correction += get_stack_height(stack_of_items)
    T[2,3] += correction
    
    return T

def get_stack_height(list_of_objects):
    """Return the height of a stack of objects
    """
    height = 0
    for obj in list_of_objects:
        ab = obj.ComputeAABB()
        height += ab.extents()[2]
    
    return height

from numpy import array
dest_random_object21 = array([[  1.000e+00,  -3.157e-15,   0.000e+00,  -1.361e+00],
       [  0.000e+00,  -2.492e-30,  -1.000e+00,   2.321e-01],
       [  3.157e-15,   1.000e+00,  -2.492e-30,   7.447e-01],
       [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
dest_random_object31 = array([[  1.000e+00,  -3.157e-15,   0.000e+00,  -1.454e+00],
       [  0.000e+00,  -2.492e-30,  -1.000e+00,  -4.364e-01],
       [  3.157e-15,   1.000e+00,  -2.492e-30,   7.447e-01],
       [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
dest_random_object32 = array([[  1.000e+00,  -3.157e-15,   0.000e+00,  -1.419e+00],
       [  0.000e+00,  -2.492e-30,  -1.000e+00,   4.608e-01],
       [  3.157e-15,   1.000e+00,  -2.492e-30,   7.447e-01],
       [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
dest_random_object22 = array([[  1.000e+00,  -3.157e-15,   0.000e+00,  -1.597e+00],
       [  0.000e+00,  -2.492e-30,  -1.000e+00,  -2.633e-01],
       [  3.157e-15,   1.000e+00,  -2.492e-30,   7.447e-01],
       [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
dest_random_object11 = array([[  1.000e+00,  -3.157e-15,   0.000e+00,  -1.230e+00],
       [  0.000e+00,  -2.492e-30,  -1.000e+00,  -4.188e-01],
       [  3.157e-15,   1.000e+00,  -2.492e-30,   7.447e-01],
       [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
dest_random_object12 = array([[  1.000e+00,  -3.157e-15,   0.000e+00,  -1.647e+00],
       [  0.000e+00,  -2.492e-30,  -1.000e+00,   4.507e-01],
       [  3.157e-15,   1.000e+00,  -2.492e-30,   7.447e-01],
       [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])

