import json
import trajoptpy


class MotionPlanner(object):
  def __init__(self, env, n_steps=30):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.n_steps = n_steps
    self.lower,self.upper = self.robot.GetDOFLimits()
    self.lower -= .1
    self.upper += .1

  def plan_with_pose(self, pos, rot,
                     collisionfree=True,
                     joint_targets=None,
                     n_steps=None):
    raise NotImplementedError("generate_traj_with_pose() not implemented!")

  def plan_with_joints(self, joint_targets,
                       collisionfree=True,
                       n_steps=None):
    raise NotImplementedError("generate_traj_with_joints() not implemented!")


class TrajoptPlanner(MotionPlanner):
  def __init__(self, env, n_steps=30):
    super(TrajoptPlanner, self).__init__(env, n_steps)
    # self.viewer = trajoptpy.GetViewer(self.env)
    # trajoptpy.SetInteractive(True)

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
        # {
        #   "type" : "collision",
        #   "name" : "cont_col", # Shorten name so printed table will be prettier
        #   "params" : {
        #     "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
        #     "dist_pen" : [0.00] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
        #   }
        # }
        ],
        "constraints" : [
          goal_constraint
        ],
        "init_info" : init_info
      }

      if collisionfree:
        request['costs'].append({
          "type" : "collision",
          "name" : "cont_col_free",
          "params" : {
            "coeffs" : [20],
            "dist_pen" : [0.02]
          }
        })
      else:
        # TODO: replace with better cost function
        request['costs'].append({
          "type" : "collision",
          "name" : "cont_col",
          "params" : {
            "coeffs" : [5],
            "dist_pen" : [0.01]
          }
        })

      prob = trajoptpy.ConstructProblem(json.dumps(request), self.env)
      result = trajoptpy.OptimizeProblem(prob)
      traj = result.GetTraj()
      return traj

  def plan_with_pose(self, pos, rot,
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

  def plan_with_joints(self, joint_targets,
                       collisionfree=True,
                       n_steps=None):
    goal_constraint = {
      "type" : "joint",
      "params" : {"vals" : joint_targets}
    }
    return self._generate_traj(goal_constraint, n_steps, collisionfree,
                               joint_targets)
