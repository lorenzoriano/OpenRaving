import json
import trajoptpy
import openravepy
from collision_checker import CollisionChecker


class TrajectoryGenerator(object):
  def __init__(self, env, n_steps=30):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.n_steps = n_steps
    self.collision_checker = CollisionChecker(self.env)
    # self.viewer = trajoptpy.GetViewer(self.env)
    # trajoptpy.SetInteractive(True)
    self.lower,self.upper = self.robot.GetDOFLimits()
    self.lower -= .3
    self.upper += .3

  def _generate_traj(self, goal_constraint, n_steps, collisionfree,
                     joint_targets):
    with self.env:
      self.robot.SetDOFLimits(self.lower, self.upper)

      if joint_targets is None:
        init_info = {
          "type" : "stationary"
        }
      else:
        init_info = {
          "type" : "straight_line", # straight line in joint space.
          "endpoint" : joint_targets
        }

      request = {
        "basic_info" : {
          "n_steps" : self.n_steps if (n_steps is None) else n_steps,
          "manip" : "rightarm", # see below for valid values
          "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
        },
        "costs" : [
        {
          "type" : "joint_vel", # joint-space velocity cost
          "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
        },
        # {
        #   "type" : "collision",
        #   "name" : "col", # Shorten name so printed table will be prettier
        #   "params" : {
        #     "continuous":False,
        #     "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
        #     "dist_pen" : [0.02] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
        #   }
        # },
        {
          "type" : "collision",
          "name" : "cont_col", # Shorten name so printed table will be prettier
          "params" : {
            "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
            "dist_pen" : [0.04] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
          }
        }],
        "constraints" : [
          goal_constraint
        ],
        "init_info" : init_info
      }

      # add high penalty for collision if collisionfree
      if collisionfree:
        request['costs'].append({
          "type" : "collision",
          "name" : "cont_col_free",
          "params" : {
            "coeffs" : [200],
            "dist_pen" : [0.01]
          }
        })

      prob = trajoptpy.ConstructProblem(json.dumps(request), self.env)
      result = trajoptpy.OptimizeProblem(prob)
      traj = result.GetTraj()

      collisions = self.collision_checker.get_collisions(traj)
      if collisionfree and collisions:
        return None, collisions

      return traj, collisions

  def generate_traj_with_pose(self, pos, rot,
                              collisionfree=True,
                              joint_targets=None,
                              n_steps=None):

    goal_constraint = {
      "type" : "pose",
      "params" : {"xyz" : pos,
                  "wxyz" : rot,
                  "link": "r_gripper_tool_frame",
                  "pos_coeffs" : [20, 20, 20],
                  "rot_coeffs" : [20, 20, 20]}
    }
    return self._generate_traj(goal_constraint, n_steps, collisionfree,
                               joint_targets)

  def generate_traj_with_joints(self, joint_targets,
                                collisionfree=True,
                                n_steps=None):
    goal_constraint = {
      "type" : "joint",
      "params" : {"vals" : joint_targets}
    }
    return self._generate_traj(goal_constraint, n_steps, collisionfree,
                               joint_targets)

class GraspTrajectoryGenerator(object):
  def __init__(self, env, unmovable_objects=set()):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.unmovable_objects = unmovable_objects
    self.pregrasp_trajectory_generator = TrajectoryGenerator(self.env)
    self.grasp_trajectory_generator = TrajectoryGenerator(self.env, 5)

  def generate_grasping_traj(self, obj, grasp_pose_list, collisionfree=True):
    for grasp_pose, pre_grasp_pose in grasp_pose_list:
      # find IK for pregrasp
      if collisionfree:
        init_joints1 = self.manip.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.CheckEnvCollisions)
      else:
        init_joints1 = self.manip.FindIKSolution(pre_grasp_pose,
          openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)

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

      traj1, collisions1 = self.pregrasp_trajectory_generator.generate_traj_with_pose(
        xyz_target1, quat_target1,
        collisionfree=collisionfree, joint_targets=init_joints1.tolist())
      if traj1 is None:
        continue

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
      traj2, collisions2 = self.grasp_trajectory_generator.generate_traj_with_pose(
        xyz_target2, quat_target2,
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
