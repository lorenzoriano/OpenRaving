import openravepy
from motion_planner import TrajoptPlanner
from collision_checker import CollisionChecker


class TrajectoryGenerator(object):
  """
  Makes calls to the motion planner and checks the resulting
  trajectory for collisions.
  """
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.motion_planner = TrajoptPlanner(self.env)
    self.collision_checker = CollisionChecker(self.env)

  def traj_from_pose(self, pos, rot,
                     collisionfree=True,
                     joint_targets=None,
                     n_steps=None):
    traj = self.motion_planner.plan_with_pose(pos, rot, collisionfree,
      joint_targets, n_steps)
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


class GraspTrajectoryGenerator(object):
  def __init__(self, env, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.unmovable_objects = unmovable_objects
    self.traj_generator = TrajectoryGenerator(self.env)

  def generate_grasping_traj(self, obj, grasp_pose_list, collisionfree=True):
    for grasp_pose, pre_grasp_pose in grasp_pose_list:
      # find IK for pregrasp
      if collisionfree:
        init_joints1 = self.manip.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.CheckEnvCollisions)
      else:
        init_joints1 = self.manip.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)

      with self.env:
        # find IK for grasp
        self.env.Remove(obj)
        if collisionfree:
          init_joints2 = self.manip.FindIKSolution(grasp_pose,
            openravepy.IkFilterOptions.CheckEnvCollisions)
        else:
          init_joints2 = self.manip.FindIKSolution(grasp_pose,
            openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
        self.env.AddKinBody(obj)

      if (init_joints1 is None) or (init_joints2 is None):
        continue

      # find traj for pregrasp
      gripper_pose1 = openravepy.poseFromMatrix(pre_grasp_pose).tolist()
      xyz_target1 = gripper_pose1[4:7]
      # quaternions are rotated by pi/2 around y for some reason...
      quat_target1 = openravepy.quatMultiply(gripper_pose1[:4],
                                            (0.7071, 0, -0.7071, 0)).tolist()

      traj1, collisions1 = self.traj_generator.traj_from_pose(
        xyz_target1, quat_target1,
        collisionfree=collisionfree, joint_targets=init_joints1.tolist())
      if traj1 is None:
        continue

      with self.env:
        # find trajectory to grasp
        orig_values = self.robot.GetDOFValues(
          self.robot.GetManipulator('rightarm').GetArmIndices())
        self.robot.SetDOFValues(traj1[-1],
          self.robot.GetManipulator('rightarm').GetArmIndices())

        gripper_pose2 = openravepy.poseFromMatrix(grasp_pose).tolist()
        xyz_target2 = gripper_pose2[4:7]
        # quaternions are rotated by pi/2 around y for some reason...
        quat_target2 = openravepy.quatMultiply(gripper_pose2[:4],
                                              (0.7071, 0, -0.7071, 0)).tolist()

        self.env.Remove(obj)
        traj2, collisions2 = self.traj_generator.traj_from_pose(
          xyz_target2, quat_target2, n_steps=2,
          collisionfree=collisionfree, joint_targets=init_joints2.tolist())
        self.env.AddKinBody(obj)

        # reset
        self.robot.SetDOFValues(orig_values,
          self.robot.GetManipulator('rightarm').GetArmIndices())

      if traj2 is None:
        continue

      collisions = collisions1.union(collisions2)
      if obj in collisions:
        collisions.remove(obj)
      if self.unmovable_objects.intersection(collisions):
        continue

      return traj1.tolist() + traj2.tolist(), collisions

    return None, set()
