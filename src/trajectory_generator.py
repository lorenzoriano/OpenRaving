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

    traj, cost = self.motion_planner.plan_with_pose(pos, rot, collisionfree,
      joint_targets, n_steps, manip=manip)
    collisions = self.collision_checker.get_collisions(traj)
    if collisionfree and collisions:
      return None, float('inf'), collisions
    else:
      return traj, cost, collisions

  def traj_from_joints(self, joint_targets,
                       collisionfree=True,
                       n_steps=None):
    traj, cost = self.motion_planner.plan_with_joints(joint_targets, collisionfree,
      n_steps)
    collisions = self.collision_checker.get_collisions(traj)
    if collisionfree and collisions:
      return None, float('inf'), collisions
    else:
      return traj, cost, collisions

  def optimize_traj(self, init_traj, collisionfree=True, manip='rightarm'):
    """
    Takes a given trajectory and optimizes it, keeping the end gripper pose
    the same for the chosen manipulator.

    If the motion planner does not support optimizing trajectories, just
    returns the original trajectory.
    """
    try:
      traj, cost = self.motion_planner.optimize_traj(init_traj, manip,
        collisionfree=collisionfree)
    except NotImplementedError:
      print "Motion planner does not support optimizng trajectories!"
    collisions = self.collision_checker.get_collisions(traj)
    if collisionfree and collisions:
      return None, float('inf'), collisions
    else:
      return traj, cost, collisions


class PickTrajGenerator(object):
  def __init__(self, env, unmovable_objects=set(), lift_amount=0.2):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.unmovable_objects = unmovable_objects
    self.traj_generator = TrajectoryGenerator(self.env)
    self.lift_amount = lift_amount

  def optimize_picking_trajs(self, init_trajs, manip, obj_to_grasp, collisionfree=True):
      with self.env:
        orig_values = self.robot.GetDOFValues(self.robot.GetActiveDOFIndices())

        opt_traj1, _, col1 = self.traj_generator.optimize_traj(init_trajs[0],
          collisionfree=collisionfree, manip=manip)

        self.env.Remove(obj_to_grasp)

        self.robot.SetDOFValues(opt_traj1[-1], self.robot.GetActiveDOFIndices())
        opt_traj2, _, col2 = self.traj_generator.optimize_traj(init_trajs[1],
          collisionfree=collisionfree, manip=manip)

        self.robot.SetDOFValues(opt_traj2[-1], self.robot.GetActiveDOFIndices())
        opt_traj3, _, col3 = self.traj_generator.optimize_traj(init_trajs[2],
          collisionfree=collisionfree, manip=manip)

        # reset
        self.env.AddKinBody(obj_to_grasp)
        self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())

      return ([opt_traj1.tolist(), opt_traj2.tolist(), opt_traj3.tolist()],
        col1.union(col2).union(col3), manip)

  def min_arm_col_picking_trajs(self, obj, grasp_pose_list, collisionfree=True):
    """
    Calls generate_picking_trajs() with each arm and returns the trajectory
    with the fewest collisions. If two arms have the same number of collisions,
    uses motion planner cost.

    Returns the tuple: (trajectories, collisions, manip)
    trajectories: list of trajectories returned by _generate_picking_trajs()
    collisions: set of collisions returned by _generate_picking_trajs()
    manip: the manipulator ('rightarm' or 'leftarm') chosen
    """
    trajs_r, cost_r, col_r, manip_r = self.right_arm_picking_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree)
    trajs_l, cost_l, col_l, manip_l = self.left_arm_picking_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree)

    if (trajs_r is not None) and (trajs_l is None):
      return trajs_r, col_r, manip_r
    elif (trajs_r is None) and (trajs_l is not None):
      return trajs_l, col_l, manip_l
    elif (trajs_r is not None) and (trajs_l is not None):
      if len(col_r) < len(col_l):
        return trajs_r, col_r, manip_r
      elif len(col_r) > len(col_l):
        return trajs_l, col_l, manip_l
      else:
        if cost_r <= cost_l:
          return trajs_r, col_r, manip_r
        else:
          return trajs_l, col_l, manip_l
    else:
      return None, set(), None

  def right_arm_picking_trajs(self, obj, grasp_pose_list, collisionfree=True):
    manip = 'rightarm'
    trajs, cost, col = self._generate_picking_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree, manip=manip)
    return trajs, cost, col, manip

  def left_arm_picking_trajs(self, obj, grasp_pose_list, collisionfree=True):
    manip = 'leftarm'
    trajs, cost, col = self._generate_picking_trajs(obj, grasp_pose_list,
      collisionfree=collisionfree, manip=manip)
    return trajs, cost, col, manip

  def _generate_picking_trajs(self, obj, grasp_pose_list, collisionfree=True,
    manip='rightarm'):
    """
    Returns a list of trajectories, one for each step of the pick.
    Currently, there are three trajectories:
    1: trajectory from initial position to pregrasp
    2: trajectory from pregrasp to grasp
    3: trajectory for lifting up
    """
    manip_to_use = self.robot.GetManipulator(manip)

    for grasp_pose, pre_grasp_pose in grasp_pose_list:
      lift_pose = grasp_pose.copy()
      lift_pose[2][3] += self.lift_amount

      # find IK for pregrasp
      if collisionfree:
        init_joints1 = manip_to_use.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.CheckEnvCollisions)
      else:
        init_joints1 = manip_to_use.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
      if init_joints1 is None:
        continue

      with self.env:
        self.env.Remove(obj)

        # find IK for grasp
        if collisionfree:
          init_joints2 = manip_to_use.FindIKSolution(grasp_pose,
            openravepy.IkFilterOptions.CheckEnvCollisions)
        else:
          init_joints2 = manip_to_use.FindIKSolution(grasp_pose,
            openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
        if init_joints2 is None:
          # reset
          self.env.AddKinBody(obj)
          continue

        # find IK for lift
        if collisionfree:
          init_joints3 = manip_to_use.FindIKSolution(lift_pose,
            openravepy.IkFilterOptions.CheckEnvCollisions)
        else:
          init_joints3 = manip_to_use.FindIKSolution(lift_pose,
            openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
        if init_joints3 is None:
          # reset
          self.env.AddKinBody(obj)
          continue          

        # reset
        self.env.AddKinBody(obj)

      # find trajectory for pregrasp
      gripper_pose1 = openravepy.poseFromMatrix(pre_grasp_pose).tolist()
      xyz_target1 = gripper_pose1[4:]
      quat_target1 = gripper_pose1[:4]

      traj1, cost1, collisions1 = self.traj_generator.traj_from_pose(
        xyz_target1, quat_target1,
        collisionfree=collisionfree, joint_targets=init_joints1.tolist(),
        manip=manip)
      if traj1 is None:
        continue

      with self.env:
        orig_values = self.robot.GetDOFValues(self.robot.GetActiveDOFIndices())
        self.env.Remove(obj)

        # find trajectory to grasp
        self.robot.SetDOFValues(traj1[-1], self.robot.GetActiveDOFIndices())

        gripper_pose2 = openravepy.poseFromMatrix(grasp_pose).tolist()
        xyz_target2 = gripper_pose2[4:]
        quat_target2 = gripper_pose2[:4]

        traj2, cost2, collisions2 = self.traj_generator.traj_from_pose(
          xyz_target2, quat_target2, n_steps=2,
          collisionfree=collisionfree, joint_targets=init_joints2.tolist(),
          manip=manip)
        if traj2 is None:
          # reset
          self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())
          self.env.AddKinBody(obj)
          continue

        # find trajectory to lift
        self.robot.SetDOFValues(traj2[-1], self.robot.GetActiveDOFIndices())

        gripper_pose3 = openravepy.poseFromMatrix(lift_pose).tolist()
        xyz_target3 = gripper_pose3[4:]
        quat_target3 = gripper_pose3[:4]

        traj3, cost3, collisions3 = self.traj_generator.traj_from_pose(
          xyz_target3, quat_target3, n_steps=2,
          collisionfree=collisionfree, joint_targets=init_joints3.tolist(),
          manip=manip)
        if traj3 is None:
          # reset
          self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())
          self.env.AddKinBody(obj)
          continue

        # reset
        self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())
        self.env.AddKinBody(obj)

      collisions = collisions1.union(collisions2).union(collisions3)
      if obj in collisions:
        collisions.remove(obj)
      if self.unmovable_objects.intersection(collisions):
        continue

      return ([traj1.tolist(), traj2.tolist(), traj3.tolist()],
        cost1 + cost2 + cost3, collisions)

    return None, float('inf'), set()
