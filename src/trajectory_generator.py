import openravepy
from motion_planner import TrajoptPlanner
from collision_checker import CollisionChecker
import utils


class TrajectoryGenerator(object):
  """
  Makes calls to the motion planner and checks the resulting
  trajectory for collisions.
  """
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.motion_planner = TrajoptPlanner(self.env)
    self.collision_checker = CollisionChecker(self.env)

  def traj_from_pose(self, pos, rot,
                     collisionfree=True,
                     joint_targets=None,
                     n_steps=None,
                     manip='rightarm'):
    # find IK for initialization if possible
    if joint_targets is None:
      manip_to_use = self.robot.GetManipulator(manip)
      tmat = openravepy.matrixFromPose(rot + pos)
      joint_targets = manip_to_use.FindIKSolution(tmat,
          openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
      if joint_targets is not None:
        joint_targets = joint_targets.tolist()

    if joint_targets is not None:
      joint_targets = utils.extend_joints_dofs(self.robot, joint_targets, manip)

    # quaternions are rotated by pi/2 around y for some reason...
    rot = openravepy.quatMultiply(rot, (0.7071, 0, -0.7071, 0)).tolist()

    traj = self.motion_planner.plan_with_pose(pos, rot, collisionfree,
      joint_targets, n_steps, manip=manip)
    collisions = self.collision_checker.get_collisions(traj)
    if collisionfree and collisions:
      return None, collisions
    else:
      return traj, collisions

  def traj_from_joints(self, joint_targets,
                       collisionfree=True,
                       n_steps=None):
    traj = self.motion_planner.plan_with_joints(joint_targets, collisionfree,
      n_steps)
    collisions = self.collision_checker.get_collisions(traj)
    if collisionfree and collisions:
      return None, collisions
    else:
      return traj, collisions

  def optimize_traj(self, init_traj, collisionfree=True, manip='rightarm'):
    """
    Takes a given trajectory and optimizes it, keeping the end gripper pose
    the same for the chosen manipulator.

    If the motion planner does not support optimizing trajectories, just
    returns the original trajectory.
    """
    try:
      traj = self.motion_planner.optimize_traj(init_traj, manip,
        collisionfree=collisionfree)
    except NotImplementedError:
      print "Motion planner does not support optimizng trajectories!"
    collisions = self.collision_checker.get_collisions(traj)
    if collisionfree and collisions:
      return None, collisions
    else:
      return traj, collisions


class GraspTrajectoryGenerator(object):
  def __init__(self, env, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.traj_generator = TrajectoryGenerator(self.env)

  def optimize_grasping_trajs(self, init_trajs, manip, obj_to_grasp, collisionfree=True):
      with self.env:
        orig_values = self.robot.GetDOFValues(self.robot.GetActiveDOFIndices())

        opt_traj1, col1 = self.traj_generator.optimize_traj(init_trajs[0],
          collisionfree=collisionfree, manip=manip)
        self.robot.SetDOFValues(opt_traj1[-1], self.robot.GetActiveDOFIndices())
        self.env.Remove(obj_to_grasp)
        opt_traj2, col2 = self.traj_generator.optimize_traj(init_trajs[1],
          collisionfree=collisionfree, manip=manip)
        self.env.AddKinBody(obj_to_grasp)

        # reset
        self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())

      return [opt_traj1.tolist(), opt_traj2.tolist()], col1.union(col2), manip

  def min_arm_col_grasping_trajs(self, obj, grasp_pose_list, collisionfree=True):
    """
    Calls generate_grasping_trajs() with each arm and returns the trajectory
    with the fewest collisions.

    Returns the tuple: (trajectories, collisions, manip)
    trajectories: list of trajectories returned by _generate_grasping_trajs()
    collisions: set of collisions returned by _generate_grasping_trajs()
    manip: the manipulator ('rightarm' or 'leftarm') chosen
    """
    trajs_r, col_r, manip_r = self.right_arm_grasping_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree)
    trajs_l, col_l, manip_l = self.left_arm_grasping_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree)

    if (trajs_r is not None) and (trajs_l is None):
      return trajs_r, col_r, manip_r
    elif (trajs_r is None) and (trajs_l is not None):
      return trajs_l, col_l, manip_l
    elif (trajs_r is not None) and (trajs_l is not None):
      if len(col_r) <= len(col_l):
        return trajs_r, col_r, manip_r
      else:
        return trajs_l, col_l, manip_l
    else:
      return None, set(), None

  def right_arm_grasping_trajs(self, obj, grasp_pose_list, collisionfree=True):
    manip = 'rightarm'
    trajs, col = self._generate_grasping_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree, manip=manip)
    return trajs, col, manip

  def left_arm_grasping_trajs(self, obj, grasp_pose_list, collisionfree=True):
    manip = 'leftarm'
    trajs, col = self._generate_grasping_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree, manip=manip)
    return trajs, col, manip

  def _generate_grasping_trajs(self, obj, grasp_pose_list, collisionfree=True,
    manip='rightarm'):
    """
    Returns a list of trajectories, one for each step of the grasp.
    Currently, there are two trajectories:
    1: trajectory from initial position to pregrasp
    2: trajectory from pregrasp to grasp
    """
    manip_to_use = self.robot.GetManipulator(manip)

    for grasp_pose, pre_grasp_pose in grasp_pose_list:
      # find IK for pregrasp
      if collisionfree:
        init_joints1 = manip_to_use.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.CheckEnvCollisions)
      else:
        init_joints1 = manip_to_use.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)

      with self.env:
        # find IK for grasp
        self.env.Remove(obj)
        if collisionfree:
          init_joints2 = manip_to_use.FindIKSolution(grasp_pose,
            openravepy.IkFilterOptions.CheckEnvCollisions)
        else:
          init_joints2 = manip_to_use.FindIKSolution(grasp_pose,
            openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
        self.env.AddKinBody(obj)

      if (init_joints1 is None) or (init_joints2 is None):
        continue

      # find traj for pregrasp
      gripper_pose1 = openravepy.poseFromMatrix(pre_grasp_pose).tolist()
      xyz_target1 = gripper_pose1[4:]
      quat_target1 = gripper_pose1[:4]

      traj1, collisions1 = self.traj_generator.traj_from_pose(
        xyz_target1, quat_target1,
        collisionfree=collisionfree, joint_targets=init_joints1.tolist(),
        manip=manip)
      if traj1 is None:
        continue

      with self.env:
        # find trajectory to grasp
        orig_values = self.robot.GetDOFValues(self.robot.GetActiveDOFIndices())
        self.robot.SetDOFValues(traj1[-1], self.robot.GetActiveDOFIndices())

        gripper_pose2 = openravepy.poseFromMatrix(grasp_pose).tolist()
        xyz_target2 = gripper_pose2[4:]
        quat_target2 = gripper_pose2[:4]

        self.env.Remove(obj)
        traj2, collisions2 = self.traj_generator.traj_from_pose(
          xyz_target2, quat_target2, n_steps=2,
          collisionfree=collisionfree, joint_targets=init_joints2.tolist(),
          manip=manip)
        self.env.AddKinBody(obj)

        # reset
        self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())

      if traj2 is None:
        continue

      collisions = collisions1.union(collisions2)
      if obj in collisions:
        collisions.remove(obj)
      if self.unmovable_objects.intersection(collisions):
        continue

      return [traj1.tolist(), traj2.tolist()], collisions

    return None, set()
