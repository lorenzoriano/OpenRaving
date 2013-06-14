import openravepy
import numpy as np
import utils


class GraspGenerator(object):
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]

  def generate_poses(self, obj,
                     use_general_grasps=True,
                     approach_dist=0.1):
    """
    Parameters:
    
    Returns:
    Returns a list of tuples (grasp_pose, pre_grasp_pose)
    Where grasp_pose and pre_grasp_pose are both 4x4 transformation matrices
    """    

    class _GraspOptions(object):
      def __init__(self):
        self.normalanglerange = 0.0
        self.standoffs = [0]
        self.rolls = np.arange(0.49*np.pi, 0.51*np.pi, 0.25*np.pi)
        self.directiondelta = approach_dist

    gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)

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

    grasp_pose_list = []
    for grasp in validgrasps:
      gmodel.setPreshape(grasp)

      grasp_pose = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)

      pre_grasp_pose = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)
      approach = gmodel.getGlobalApproachDir(grasp) * approach_dist
      pre_grasp_pose[0][3] -= approach[0]
      pre_grasp_pose[1][3] -= approach[1]
      pre_grasp_pose[2][3] -= approach[2]

      grasp_pose_list.append((grasp_pose, pre_grasp_pose))

    return grasp_pose_list
