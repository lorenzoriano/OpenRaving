import rospy
import openravepy as rave
from PR2 import PR2, Arm
import numpy as np
import time

class PlannerArm(Arm):
    """
    Planner class for the Arm.
    """
    def __init__ (self, pr2, rl):
        Arm.__init__(self, pr2, rl)
        self.openrave_dofs = self.manip.GetArmIndices()

       
    def move_arm(self, goal_tfm):
        """
        Generate a collision free trajectory given a goal transform
        """
        print goal_tfm
        self.pr2.robot.SetActiveManipulator(self.manip)
        basemanip = rave.interfaces.BaseManipulation(self.pr2.robot)

        traj = basemanip.MoveToHandPosition(matrices=[goal_tfm],execute=False,outputtrajobj=True)
        if traj:
            self.execute_openrave_trajectory(traj)
        else:
            rospy.loginfo('OpenRAVE plan failed!')
        

    def execute_openrave_trajectory(self, traj, sample_freq=10):
        """
        executes the trajectory on the real robot.
        traj is a trajectory generated through openrave planners.
        """
        sample_times    = np.arange(0, traj.GetDuration(), 1.0/sample_freq)
        print sample_times
        print self.openrave_dofs

        cspec           = traj.GetConfigurationSpecification()
        time_derivative = 0
        
        traj_joints = []
        for t in sample_times:
            joints = cspec.ExtractJointValues(traj.Sample(t), self.pr2.robot, self.openrave_dofs, time_derivative)
            print joints
            traj_joints.append(joints)

        self.follow_joint_trajectory(traj_joints)


class PlannerPR2 (PR2):
    """
    Planner class for PR2 with planning arms.
    """
    def __init__ (self, rave_robot=None):      
        PR2.__init__(self, rave_robot)
        time.sleep(1)
        self.rarm = PlannerArm(self, 'r')
        self.larm = PlannerArm(self, 'l')


    def gotoArmPosture (self, pos):
        """
        Makes both arms go to the specified posture.
        """
        self.larm.goto_posture(pos)
        self.rarm.goto_posture(pos)
        self.join_all()
