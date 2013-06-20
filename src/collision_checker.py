import trajoptpy.math_utils as mu
import numpy as np
import utils


class CollisionChecker(object):
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]

  def get_collisions(self, traj, n=100):
    orig_values = self.robot.GetDOFValues(self.robot.GetActiveDOFIndices());

    collisions = set()
    traj_up = mu.interp2d(np.linspace(0,1,n), np.linspace(0,1,len(traj)), traj)
    with self.env:
      for joint_values in traj_up:
        self.robot.SetDOFValues(joint_values, self.robot.GetActiveDOFIndices())
        for obj in utils.get_all_collisions(self.robot, self.env):
          collisions.add(obj)

      self.robot.SetDOFValues(orig_values, self.robot.GetActiveDOFIndices())
    return collisions
