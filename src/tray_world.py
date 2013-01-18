import numpy as np
import openravepy
import generate_reaching_poses
import utils

tray_destination =  np.array([[  1.00000000e+00,  -2.32949609e-15,  -2.56425998e-17,
         -2.61714161e+00],
       [ -2.56425998e-17,  -1.42108547e-14,  -1.00000000e+00,
          1.08424306e-01],
       [  2.32949609e-15,   1.00000000e+00,  -1.42108547e-14,
          7.44705533e-01],
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
    
    sol = generate_reaching_poses.check_reachable(manip, [T], True)
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
    
    sol = generate_reaching_poses.check_reachable(manip, [T], True)
    if sol is not None:
        robot.SetDOFValues(sol, manip.GetArmIndices())
        #opening gripper
        robot.SetDOFValues([0.2], manip.GetGripperJoints())
    else:
        raise generate_reaching_poses.GraspingPoseError("No IK solution for tray right side")
    
def move_robot_base_infront_tray(robot, tray):
    """Move the robot so that it is facing the tray. This means that the tray x and
    y position will be positive in respect to the robot's. Orientation will be fixed.
    """
    
    robotT = robot.GetTransform()
    trayT = tray.GetTransform()
    
    robotT[:3, :3] = np.eye(3)
    #changing x
    robotT[0, 3] = trayT[0, 3] - 0.6197117799999998
    #changing y
    robotT[1, 3] = trayT[1, 3]
    #z is unchanged
    robot.SetTransform(robotT)
    
def tray_putdown_pose(tray):
    """Returns a position right above the tray with the gripper pointing down.
    """
    T = tray.GetTransform()
    
    gripper_angle = (np.pi, 0., 0) #pointing downward
    rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)
    T[:3,:3] = rot_mat
    
    #fixing the height
    T[2,3] += 0.1
    
    return T
    