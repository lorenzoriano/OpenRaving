import openravepy
import numpy as np
import json

GRASPS_FILE_NAME = 'generated_grasps.json'


class GraspPoseGenerator(object):
  def __init__(self, env):
    self.env = env
    self.robot = self.env.GetRobots()[0]
    with open(GRASPS_FILE_NAME, 'r') as grasps_file:
      self.pregenerated_grasps = np.array(json.loads(grasps_file.read()))

  def generate_poses(self, obj,
                     use_general_grasps=True,
                     approach_dist=0.15):
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
        self.rolls = np.arange(0.5*np.pi, 2*np.pi, np.pi)
        self.boxdelta = 0.01
        self.directiondelta = approach_dist

    gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)

    if use_general_grasps:
      gmodel.grasps = self.pregenerated_grasps
    else:
      if not gmodel.load():
        openravepy.raveLogInfo("Generating grasping model...")
        gmodel.autogenerate(_GraspOptions())

        # only use horizontal grasps
        horiz_grasp_filter = lambda g: \
          abs(gmodel.getGlobalApproachDir(g)[2]) < 0.01
        gmodel.grasps = filter(horiz_grasp_filter, gmodel.grasps)

        # only use grasps in the upper half (+y is up)
        upper_grasp_filter = lambda g: \
          gmodel.GetLocalGraspTransform(g, collisionfree=True)[1][3] > 0
        gmodel.grasps = filter(upper_grasp_filter, gmodel.grasps)

        data = json.dumps([g.tolist() for g in gmodel.grasps])
        with open(GRASPS_FILE_NAME, 'w') as outfile:
          outfile.write(data)

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
