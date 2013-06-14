import numpy as np
import openravepy
import utils
import json
import trajoptpy
from PlannerPR2 import PlannerPR2
from collision_checker import CollisionChecker


class ObjectMover(object):
  def __init__(self, env, use_ros, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.grasping_pose_cache = {}
    self.use_ros = use_ros
    self.unmovable_objects = unmovable_objects
    self.collision_checker = CollisionChecker(self.env)
    if self.use_ros:
      self.pr2 = PlannerPR2(self.robot)

  def pickup(self, obj):
    gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)

    # begin pickup actions
    traj = self._get_grasping_trajectory(obj, gmodel)
    traj_obj = utils.array_to_traj(self.robot, traj)
    print("Executing trajectory...")
    self.robot.GetController().SetPath(traj_obj)
    self.robot.WaitForController(0)
    self.robot.GetController().Reset()
    if self.use_ros:
      self.pr2.rarm.execute_openrave_trajectory(traj)
      # self.pr2.join_all() # Doesn't work in sim for some reason..
      raw_input("Press enter when real PR2 is done moving...")  # temporary fix for above

    print("Trajectory execution complete!")

    print("Grasping object...")
    self.robot.Grab(obj)

  def _get_grasping_trajectory(self, obj_to_grasp, gmodel):
    """
    Finds a valid grasping trajectory or raises an ObjectMoveError
    if a valid trajectory cannot be found

    Parameters:
    obj_to_grasp: Object for which to compute a grasping trajectory
    gmodel: An OpenRave GraspingModel object
    
    Returns:
    An OpenRave trajectory object
    """
    obj_name = obj_to_grasp.GetName()

    grasps = self._generate_grasps(obj_to_grasp, gmodel)

    print "Trying to find a collision-free trajectory..."
    traj, _ = self._grasping_trajectory_generator(obj_to_grasp, grasps, gmodel)

    if traj is not None:
      print "Found a collision-free trajectory!!"
      return traj
    print "No collision-free trajectory found!"

    print "Trying to find any trajectory..."
    traj, collisions = self._grasping_trajectory_generator(obj_to_grasp, grasps,
                                                           gmodel,
                                                           collisionfree=False)
    if traj is not None:
      print "Trajectory found with collisions: {}".format(collisions)
      # TODO: cache and raise
      raise

    print "Object cannot be moved!"
    raise

  def _grasping_trajectory_generator(self, obj_to_grasp, grasps, gmodel,
                                     approachdist=0.1,
                                     collisionfree=True):
    for grasp in grasps:
      gmodel.setPreshape(grasp)

      # find trajectory to pregrasp
      Tgrasp1 = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)

      approach = gmodel.getGlobalApproachDir(grasp) * approachdist
      Tgrasp1[0][3] -= approach[0]
      Tgrasp1[1][3] -= approach[1]
      Tgrasp1[2][3] -= approach[2]

      if collisionfree:
        init_joints1 = self.manip.FindIKSolution(Tgrasp1,
          openravepy.IkFilterOptions.CheckEnvCollisions)
      else:
        init_joints1 = self.manip.FindIKSolution(Tgrasp1,
          openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)

      if init_joints1 is None:
        continue

      gripper_pose1 = openravepy.poseFromMatrix(Tgrasp1).tolist()
      xyz_target1 = gripper_pose1[4:7]
      # quaternions are rotated by pi/2 around y for some reason...
      quat_target1 = openravepy.quatMultiply(gripper_pose1[:4],
                                            (0.707, 0, -0.707, 0)).tolist()

      traj1, collisions1 = self._trajectory_generator(xyz_target1, quat_target1,
                                                      init_joints1,
                                                      collisionfree)
      if traj1 is None:
        continue

      # find trajectory to grasp
      orig_values = self.robot.GetDOFValues(
        self.robot.GetManipulator('rightarm').GetArmIndices())

      self.robot.SetDOFValues(traj1[-1],
        self.robot.GetManipulator('rightarm').GetArmIndices())

      Tgrasp2 = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)

      if collisionfree:
        init_joints2 = self.manip.FindIKSolution(Tgrasp2,
          openravepy.IkFilterOptions.CheckEnvCollisions)
      else:
        init_joints2 = self.manip.FindIKSolution(Tgrasp2,
          openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)

      if init_joints2 is None:
        continue

      gripper_pose2 = openravepy.poseFromMatrix(Tgrasp2).tolist()
      xyz_target2 = gripper_pose2[4:7]
      # quaternions are rotated by pi/2 around y for some reason...
      quat_target2 = openravepy.quatMultiply(gripper_pose2[:4],
                                            (0.707, 0, -0.707, 0)).tolist()

      traj2, collisions2 = self._trajectory_generator(xyz_target2, quat_target2,
                                                      init_joints2,
                                                      collisionfree)

      # reset 
      self.robot.SetDOFValues(orig_values,
        self.robot.GetManipulator('rightarm').GetArmIndices())

      if traj2 is None:
        continue

      return traj1.tolist() + traj2.tolist(), collisions1.union(collisions2)

    return None, set()

  def _trajectory_generator(self, pos, rot, init_joints,
                            collisionfree=True):
    # trajoptpy.SetInteractive(True)
    request = {
      "basic_info" : {
        "n_steps" : 10,
        "manip" : "rightarm", # see below for valid values
        "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
      },
      "costs" : [
      {
        "type" : "joint_vel", # joint-space velocity cost
        "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
        # Also valid: "coeffs" : [7,6,5,4,3,2,1]
      },
      {
        "type" : "collision",
        "name" : "col", # Shorten name so printed table will be prettier
        "params" : {
          "coeffs" : [40], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
          "dist_pen" : [0.0] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
        }
      },
      {
        "type" : "continuous_collision",
        "name" : "cont_col", # Shorten name so printed table will be prettier
        "params" : {
          "coeffs" : [40], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
          "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
        }
      }
      ],
      "constraints" : [
      {
        "type" : "pose",
        "params" : {"xyz" : pos,
                    "wxyz" : rot,
                    "link": "r_gripper_tool_frame"}
      }],
      "init_info" : {
          "type" : "straight_line", # straight line in joint space.
          "endpoint" : init_joints.tolist()
      }
    }
    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, self.env)
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    traj = result.GetTraj()
    # print "Got a trajectory!"

    collisions = self.collision_checker.get_collisions(traj)
    # print "Collisions: {}".format(collisions)

    if collisionfree and collisions:
      return None, set()

    return traj, collisions

  def _get_min_col_grasping_pose(self, obj_to_grasp, gmodel):
    min_col = float('inf')
    best_pose = None
    best_grasp = None
    best_collisions = []
    for pose, grasp, collisions in self._get_grasping_poses(obj_to_grasp,
                                                            gmodel,
                                                            col_free=False):
      if len(collisions) < min_col:
        best_pose = pose
        best_grasp = grasp
        best_collisions = collisions
    return best_pose, best_grasp, best_collisions

  def _get_grasping_poses(self, obj_to_grasp, gmodel, col_free=True):
    # generating grasps
    grasps = self._generate_grasps(obj_to_grasp, gmodel)
    openravepy.raveLogInfo("I've got %d grasps" % len(grasps))
    if len(grasps) == 0:
      e = ObjectMoveError("No good grasp for %s!" % obj_name)
      raise e

    # finding collision free grasping pose
    if col_free:
      ik_options = openravepy.IkFilterOptions.CheckEnvCollisions
    else:
      ik_options = openravepy.IkFilterOptions.IgnoreEndEffectorCollisions

    for grasp in grasps:
      # open gripper when checking for collisions
      dof_orig = self.robot.GetActiveDOFValues()
      dof_copy = list(dof_orig)
      gripper_joint_index = self.robot.GetJoint('r_gripper_l_finger_joint').GetDOFIndex()
      dof_copy[gripper_joint_index] = 0.54
      self.robot.SetDOFValues(dof_copy)
      # remove obj_to_grasp from environment when checking for collisions
      self.env.Remove(obj_to_grasp)

      grasp_transform = gmodel.getGlobalGraspTransform(grasp)
      pose = self.manip.FindIKSolution(grasp_transform, ik_options)

      if pose is None:
        # restore removed obj_to_grasp and robot DOFs
        self.env.AddKinBody(obj_to_grasp)
        self.robot.SetDOFValues(dof_orig)
        continue

      self.robot.SetDOFValues(pose, self.robot.GetActiveManipulator().GetArmIndices());

      collisions = utils.get_all_collisions(self.robot, self.env)

      # restore removed obj_to_grasp and robot DOFs
      self.env.AddKinBody(obj_to_grasp)
      self.robot.SetDOFValues(dof_orig)

      # make sure collisions don't contain any unmovable objects
      has_unmovable_obj = False
      for body in collisions:
        if body.GetName() in self.unmovable_objects:
          has_unmovable_obj = True
          break
      if has_unmovable_obj:
        continue

      yield pose, grasp, collisions
      
  def _generate_grasps(self, obj, gmodel, use_general_grasps=True):
    """
    Returns a list with all the valid grasps for object obj.

    Parameters:
    obj: blah
    
    Returns:
    List of Transformation matrices, one for each valid grasping pose.
    """    
    
    class _GraspOptions(object):
      def __init__(self):
        #self.delta = 0.0
        self.normalanglerange = 0.0
        self.standoffs = [0]
        self.rolls = np.arange(0.49*np.pi, 0.51*np.pi, 0.25*np.pi)
        self.directiondelta = 0.1
        pass

    if use_general_grasps:
      gmodel.grasps = utils.side_cylinder_pre_grasps
    else:
      if not gmodel.load():
        openravepy.raveLogInfo("Generating grasping model...")
        gmodel.autogenerate(_GraspOptions())

    openravepy.raveLogInfo("Generating grasps")
    validgrasps, _ = gmodel.computeValidGrasps(checkcollision=False, 
                                               checkik=False,
                                               checkgrasper=False)
    np.random.shuffle(validgrasps)
    
    openravepy.raveLogInfo("Number of valid grasps: %d" % len(validgrasps))
    return validgrasps
    # return [gmodel.getGlobalGraspTransform(grasp) for grasp in validgrasps]

class ObjectMoveError(Exception):
  pass
