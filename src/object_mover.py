import numpy as np
import openravepy
import utils
from PlannerPR2 import PlannerPR2


class ObjectMover(object):
  def __init__(self, env, use_ros, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.grasping_pose_cache = {}
    self.use_ros = use_ros
    self.unmovable_objects = unmovable_objects
    if self.use_ros:
      self.pr2 = PlannerPR2(self.robot)

  def pickup(self, obj):
    gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)

    # begin pickup actions
    traj = self._get_grasping_trajectory(obj, gmodel)
    print("Executing trajectory...")
    self.robot.GetController().SetPath(traj)
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

    pose_and_grasp = self.grasping_pose_cache.get(obj_name, None)
    if pose_and_grasp is not None:
      print("Pose and grasp for %s in cache!" % obj_name)
      pose, grasp = pose_and_grasp
      gmodel.setPreshape(grasp)
      basemanip = openravepy.interfaces.BaseManipulation(self.robot)
      print("Calculating trajectory...")
      try:
        traj = basemanip.MoveManipulator(goal=pose, execute=False,
                                         outputtrajobj=True)
        print("Got a trajectory!")
        return traj
      except:
        print("No collision-free trajetory for this pose.\
               Clearing cache for this object")
        del self.grasping_pose_cache[obj_name]

    for pose, grasp, _ in self._get_grasping_poses(obj_to_grasp, gmodel):
      gmodel.setPreshape(grasp)
      basemanip = openravepy.interfaces.BaseManipulation(self.robot)
      print("Calculating trajectory...")
      try:
        traj = basemanip.MoveManipulator(goal=pose, execute=False,
                                         outputtrajobj=True)
        print("Got a trajectory!")
        return traj
      except:
        print("No collision-free trajetory for this pose. Trying again...")
      
    # No collision free trajectory found.
    pose, grasp, collisions = self._get_min_col_grasping_pose(obj_to_grasp, gmodel)
    if pose is None:
      e = ObjectMoveError("Object %s out of reach!" % obj_name)
      raise e
    else:
      self.grasping_pose_cache[obj_name] = (pose, grasp)
      e = ObjectMoveError("No collision free trajectory found!")
      e.collision_list = [obj.GetName() for obj in collisions]
      raise e

  # def _get_grasping_pose(self, obj_to_grasp, gmodel, col_free=True, bad_bodies=[]):
  #   obj_name = obj_to_grasp.GetName()
    
  #   cached_value = self.col_free_grasping_pose_cache.get(obj_name, None)
  #   if cached_value is None:
  #     # print "Collision free pose for object %s is not cached, \
  #     #        looking for a pose" % obj_name
  #     pose, grasp, collisions = self._get_min_col_grasping_pose(obj_to_grasp, gmodel,
  #                                                        col_free,
  #                                                        bad_bodies)
  #     if pose is None:
  #       if col_free:
  #         pose, grasp, collisions = self._get_grasping_pose(obj_to_grasp,
  #                                                           gmodel,
  #                                                           col_free=False,
  #                                                           bad_bodies=bad_bodies)
  #         e = ObjectMoveError("No collision free grasping pose found")
  #         e.collision_list = collisions
  #       else:
  #         e = ObjectMoveError("No grasping pose found")
  #       raise e
  #     # TODO: Enable caching later
  #     # self.col_free_grasping_pose_cache[obj_name] = pose
  #   else:
  #     # print "Collision free pose for object %s already cached" %  obj_name
  #     # pose = cached_value
  #     pass

  #   return pose, grasp, collisions

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
        continue

      self.robot.SetDOFValues(pose, self.robot.GetActiveManipulator().GetArmIndices());

      collisions = utils.get_all_collisions(self.robot, self.env)

      # restore removed obj_to_grasp and robot DOFs after collision check
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

  # def _get_min_col_grasping_pose(self, obj_to_grasp, gmodel, col_free=True,
  #                                bad_bodies=None):
  #   # generating grasps
  #   grasps = self._generate_grasps(obj_to_grasp, gmodel)
  #   openravepy.raveLogInfo("I've got %d grasps" % len(grasps))
  #   if len(grasps) == 0:
  #     return None, []

  #   # open gripper when checking for collisions
  #   dof_orig = self.robot.GetActiveDOFValues()
  #   dof_copy = list(dof_orig)
  #   gripper_joint_index = self.robot.GetJoint('r_gripper_l_finger_joint').GetDOFIndex()
  #   dof_copy[gripper_joint_index] = 0.54
  #   self.robot.SetDOFValues(dof_copy)

  #   # remove obj_to_grasp from environment when checking for collisions
  #   self.env.Remove(obj_to_grasp)

  #   # finding collision free grasping pose
  #   if col_free:
  #     ik_options = openravepy.IkFilterOptions.CheckEnvCollisions
  #   else:
  #     ik_options = openravepy.IkFilterOptions.IgnoreEndEffectorCollisions

  #   best_pose = None
  #   best_grasp = None
  #   min_collisions = []
  #   num_min_collisions = float('inf')
  #   for grasp in grasps:
  #     grasp_transform = gmodel.getGlobalGraspTransform(grasp)
  #     pose = self.manip.FindIKSolution(grasp_transform, ik_options)

  #     if pose is None:
  #       continue

  #     self.robot.SetDOFValues(pose, self.robot.GetActiveManipulator().GetArmIndices());

  #     collisions = utils.get_all_collisions(self.robot, self.env)
  #     # make sure collisions don't include any bad bodies
  #     bad_body_exists = False
  #     for body in collisions:
  #       if body in bad_bodies:
  #         bad_body_exists = True
  #         break;
  #     if bad_body_exists:
  #       continue

  #     if col_free:
  #       best_pose = pose
  #       min_collisions = collisions
  #       best_grasp = grasp
  #       break

  #     if len(collisions) < num_min_collisions:
  #       num_min_collisions = len(collisions)
  #       min_collisions = collisions
  #       best_pose = pose
  #       best_grasp = grasp

  #   # restore removed obj_to_grasp and robot DOFs before returning
  #   self.env.AddKinBody(obj_to_grasp)
  #   self.robot.SetDOFValues(dof_orig)

  #   return best_pose, best_grasp, [obj.GetName() for obj in min_collisions]

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
        self.delta = 0.0
        self.normalanglerange = 0.0
        self.standoffs = [0]
        self.rolls = np.arange(0.49*np.pi, 0.51*np.pi, 0.25*np.pi)
        self.directiondelta = 0.0
        pass

    if use_general_grasps:
      gmodel.grasps = utils.side_cylinder_pre_grasps
    else:
      if not gmodel.load():
        openravepy.raveLogInfo("Generating grasping model...")
        gmodel.autogenerate(_GraspOptions())

    openravepy.raveLogInfo("Generating grasps")
    validgrasps, _ = gmodel.computeValidGrasps(checkcollision=False, 
                           checkik=True,
                           checkgrasper=False)
    np.random.shuffle(validgrasps)
    
    openravepy.raveLogInfo("Number of valid grasps: %d" % len(validgrasps))
    return validgrasps
    # return [gmodel.getGlobalGraspTransform(grasp) for grasp in validgrasps]

class ObjectMoveError(Exception):
  pass
