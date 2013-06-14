from PlannerPR2 import PlannerPR2
import rospy
import time
import openravepy as rave


global PR2
global env

PR2 = None
env = None


def move_pr2(rave_pr2, traj, rl):
    """
    Basically, the PlannerPR2 object is the one which moves the real PR2.
    This function basically caches the object, hence you are not required to use this
    if you keep arounf the PR2Planner object.
    """
    
    global PR2
    if not PR2:
        # initialize the PlannerPR2 object if it has not been.
        PR2 = PlannerPR2(rave_pr2)

    # ask the PlannerPR2 to execute the trajectory.
    # note PlannerPR2 is a misnomer as all the planning is done in the function below (move_arm).
    arm = PR2.rarm if rl=='r' else PR2.larm
    arm.execute_openrave_trajectory(traj)



def move_arm(rave_pr2, goal_tfm, rl):
    """
    Call this function to move the pr2's arm to GOAL_TFM (the goal transform).
    RAVE_PR2 is the openrave robot object.
    rl : says which arm it should move: use 'r' for right and 'l' for left
    """
    
    manip_name = '%sarm'% ('left' if rl=='l' else 'right')
    rave_pr2.SetActiveManipulator(manip_name)
    
    print goal_tfm

    basemanip = rave.interfaces.BaseManipulation(rave_pr2)
    traj = basemanip.MoveToHandPosition(matrices=[goal_tfm],execute=False,outputtrajobj=True)
    if traj:
        move_pr2(rave_pr2, traj, rl)
    else:
        rospy.loginfo('OpenRAVE plan failed!')



def test(rave_pr2, target, rl='l'):
    """
    This is a test function which generates a goal transform based on a grasp found by openrave.
    """
    # get a goal grasp transform
    arm_name = '%sarm'% ('left' if rl=='l' else 'right')
    rave_pr2.SetActiveManipulator(arm_name)

    gmodel = rave.databases.grasping.GraspingModel(rave_pr2, target)
    if not gmodel.load():
        gmodel.autogenerate()

    validgrasps, validindicees = gmodel.computeValidGrasps(returnnum=1)
    grasp  = validgrasps[0]
    gmodel.setPreshape(grasp)
    goal_tfm    = gmodel.getGlobalGraspTransform(grasp,collisionfree=True)
    
    # calculate goal joints. not required for our purposes.
    # goal_joints = gmodel.manip.FindIKSolution(goal_tfm, filteroptions=rave.IkFilterOptions.CheckEnvCollisions)
    
    # move the pr2 arm.
    move_arm(rave_pr2, goal_tfm, rl)
    

if __name__=="__main__":
    global env
    rospy.init_node("test_grasp_move")
    time.sleep(1)
    
    env = rave.Environment()
    env.Load('data/pr2test1.env.xml')
    #env.SetViewer('qtcoin')
    rave_pr2 = env.GetRobots()[0]
    target = env.GetKinBody('mug1')
