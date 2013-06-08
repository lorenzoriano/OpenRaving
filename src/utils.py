import numpy as np
import openravepy
import re
import math

pre_grasps = np.array([[ -3.14269681e-01,   1.11111111e-01,   9.42809042e-01,
                         -8.88888889e-01,   3.14269681e-01,  -3.33333333e-01,
                         -3.33333333e-01,  -9.42809042e-01,   0.00000000e+00,
                          1.53599998e-02,   1.68760976e-02,  -4.00000000e-02,
                         -4.24847351e-03,  -3.33333333e-01,  -9.42809042e-01,
                         -1.51615257e-16,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.91763442e-06,
                          2.00000000e-02,   3.00000000e-02,  -4.00000000e-02,
                          5.48000224e-01,  -3.14269681e-01,   1.11111111e-01,
                          9.42809042e-01,  -8.88888889e-01,   3.14269681e-01,
                         -3.33333333e-01,  -3.33333333e-01,  -9.42809042e-01,
                          0.00000000e+00,   1.52999998e-02,   1.67063920e-02,
                         -4.00000000e-02,   1.57079633e+00],
                       [ -3.33333333e-01,  -8.16431199e-17,   9.42809042e-01,
                         -8.16431199e-17,   1.00000000e+00,   5.77304037e-17,
                         -9.42809042e-01,  -5.77304037e-17,  -3.33333333e-01,
                         -1.81869716e-02,  -1.03862769e-17,   2.29633328e-02,
                         -1.25620088e-03,  -9.42809042e-01,  -5.77304037e-17,
                         -3.33333333e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.52767676e-03,
                          3.00000000e-02,   0.00000000e+00,   4.00000000e-02,
                          5.48000224e-01,  -3.33333333e-01,  -8.16431199e-17,
                          9.42809042e-01,  -8.16431199e-17,   1.00000000e+00,
                          5.77304037e-17,  -9.42809042e-01,  -5.77304037e-17,
                         -3.33333333e-01,  -1.82718244e-02,  -1.03914727e-17,
                          2.29333328e-02,   0.00000000e+00],
                       [ -3.33333333e-01,   8.16431199e-17,  -9.42809042e-01,
                          8.16431199e-17,   1.00000000e+00,   5.77304037e-17,
                          9.42809042e-01,  -5.77304037e-17,  -3.33333333e-01,
                          1.81021188e-02,  -1.03810812e-17,   2.29933328e-02,
                         -1.25620088e-03,   9.42809042e-01,  -5.77304037e-17,
                         -3.33333333e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.55474007e-03,
                         -3.00000000e-02,   0.00000000e+00,   4.00000000e-02,
                          5.48000224e-01,  -3.33333333e-01,   8.16431199e-17,
                         -9.42809042e-01,   8.16431199e-17,   1.00000000e+00,
                          5.77304037e-17,   9.42809042e-01,  -5.77304037e-17,
                         -3.33333333e-01,   1.82718244e-02,  -1.03914727e-17,
                          2.29333328e-02,   0.00000000e+00],
                       [  3.33333333e-01,  -4.08215600e-17,   9.42809042e-01,
                         -4.08215600e-17,   1.00000000e+00,   5.77304037e-17,
                         -9.42809042e-01,  -5.77304037e-17,   3.33333333e-01,
                         -1.81021188e-02,  -3.81366568e-17,  -2.29933328e-02,
                         -1.25620088e-03,  -9.42809042e-01,  -5.77304037e-17,
                          3.33333333e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.64296933e-03,
                          3.00000000e-02,   0.00000000e+00,  -4.00000000e-02,
                          5.48000224e-01,   3.33333333e-01,  -4.08215600e-17,
                          9.42809042e-01,  -4.08215600e-17,   1.00000000e+00,
                          5.77304037e-17,  -9.42809042e-01,  -5.77304037e-17,
                          3.33333333e-01,  -1.82718244e-02,  -3.81470483e-17,
                         -2.29333328e-02,   0.00000000e+00],
                       [  3.33333333e-01,   4.08215600e-17,  -9.42809042e-01,
                          4.08215600e-17,   1.00000000e+00,   5.77304037e-17,
                          9.42809042e-01,  -5.77304037e-17,   3.33333333e-01,
                          1.81869716e-02,   1.73692987e-17,  -2.29633328e-02,
                         -1.25620088e-03,   9.42809042e-01,  -5.77304037e-17,
                          3.33333333e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.54499208e-03,
                         -3.00000000e-02,   0.00000000e+00,  -4.00000000e-02,
                          5.48000224e-01,   3.33333333e-01,   4.08215600e-17,
                         -9.42809042e-01,   4.08215600e-17,   1.00000000e+00,
                          5.77304037e-17,   9.42809042e-01,  -5.77304037e-17,
                          3.33333333e-01,   1.82718244e-02,   1.73641029e-17,
                         -2.29333328e-02,   0.00000000e+00],
                       [ -6.10622664e-16,  -3.82683432e-01,   9.23879533e-01,
                         -1.00000000e+00,   4.99600361e-16,  -5.55111512e-16,
                         -1.11022302e-16,  -9.23879533e-01,  -3.82683432e-01,
                         -1.11022302e-16,  -1.67575446e-02,   2.06323909e-02,
                         -1.18760866e-03,  -6.12323400e-17,  -9.23879533e-01,
                         -3.82683432e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   7.56882316e-04,
                          0.00000000e+00,   3.00000000e-02,   4.00000000e-02,
                          5.48000224e-01,  -6.10622664e-16,  -3.82683432e-01,
                          9.23879533e-01,  -1.00000000e+00,   4.99600361e-16,
                         -5.55111512e-16,  -1.11022302e-16,  -9.23879533e-01,
                         -3.82683432e-01,  -1.11022302e-16,  -1.68406937e-02,
                          2.05979494e-02,   1.57079633e+00],
                       [ -5.55111512e-16,   3.82683432e-01,  -9.23879533e-01,
                         -1.00000000e+00,  -3.33066907e-16,   4.44089210e-16,
                         -5.55111512e-17,   9.23879533e-01,   3.82683432e-01,
                         -1.11022302e-16,   1.67575446e-02,  -2.06323909e-02,
                         -1.18760866e-03,  -6.12323400e-17,   9.23879533e-01,
                          3.82683432e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   8.58379104e-04,
                          0.00000000e+00,  -3.00000000e-02,  -4.00000000e-02,
                          5.48000224e-01,  -5.55111512e-16,   3.82683432e-01,
                         -9.23879533e-01,  -1.00000000e+00,  -3.33066907e-16,
                          4.44089210e-16,  -5.55111512e-17,   9.23879533e-01,
                          3.82683432e-01,  -1.11022302e-16,   1.68406937e-02,
                         -2.05979494e-02,   1.57079633e+00],
                       [ -5.55111512e-16,   9.23879533e-01,   3.82683432e-01,
                         -1.00000000e+00,  -4.44089210e-16,  -3.05311332e-16,
                         -1.38777878e-16,  -3.82683432e-01,   9.23879533e-01,
                          0.00000000e+00,   6.32390901e-04,  -3.24245544e-03,
                         -9.50971244e-04,  -6.12323400e-17,  -3.82683432e-01,
                          9.23879533e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   8.00254671e-04,
                          0.00000000e+00,   2.00000000e-02,  -5.00000000e-02,
                          5.48000224e-01,  -5.55111512e-16,   9.23879533e-01,
                          3.82683432e-01,  -1.00000000e+00,  -4.44089210e-16,
                         -3.05311332e-16,  -1.38777878e-16,  -3.82683432e-01,
                          9.23879533e-01,   0.00000000e+00,   5.97949392e-04,
                         -3.15930628e-03,   1.57079633e+00],
                       [ -1.87350135e-16,  -9.23879533e-01,  -3.82683432e-01,
                         -1.00000000e+00,   1.45716772e-16,   5.55111512e-17,
                          5.55111512e-17,   3.82683432e-01,  -9.23879533e-01,
                         -1.11022302e-16,  -6.32390901e-04,   3.24245544e-03,
                         -9.50971244e-04,  -6.12323400e-17,   3.82683432e-01,
                         -9.23879533e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   8.36500208e-04,
                          0.00000000e+00,  -2.00000000e-02,   5.00000000e-02,
                          5.48000224e-01,  -1.87350135e-16,  -9.23879533e-01,
                         -3.82683432e-01,  -1.00000000e+00,   1.45716772e-16,
                          5.55111512e-17,   5.55111512e-17,   3.82683432e-01,
                         -9.23879533e-01,  -1.11022302e-16,  -5.97949392e-04,
                          3.15930628e-03,   1.57079633e+00],
                       [  9.42809042e-01,  -2.97149141e-17,  -3.33333333e-01,
                         -2.97149141e-17,   1.00000000e+00,  -1.73191211e-16,
                          3.33333333e-01,   1.73191211e-16,   9.42809042e-01,
                         -2.99333281e-03,   3.38766799e-18,  -1.89788122e-03,
                         -9.45463674e-04,   3.33333333e-01,   1.73191211e-16,
                          9.42809042e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.61552927e-03,
                         -2.00000000e-02,   0.00000000e+00,  -5.00000000e-02,
                          5.48000224e-01,   9.42809042e-01,  -2.97149141e-17,
                         -3.33333333e-01,  -2.97149141e-17,   1.00000000e+00,
                         -1.73191211e-16,   3.33333333e-01,   1.73191211e-16,
                          9.42809042e-01,  -2.93333281e-03,   3.41884241e-18,
                         -1.72817560e-03,   0.00000000e+00],
                       [ -9.42809042e-01,  -3.36477451e-16,   3.33333333e-01,
                         -3.36477451e-16,   1.00000000e+00,   5.77304037e-17,
                         -3.33333333e-01,  -5.77304037e-17,  -9.42809042e-01,
                          2.99333281e-03,   1.73744944e-17,   1.89788122e-03,
                         -9.45463674e-04,  -3.33333333e-01,  -5.77304037e-17,
                         -9.42809042e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.58216750e-03,
                          2.00000000e-02,   0.00000000e+00,   5.00000000e-02,
                          5.48000224e-01,  -9.42809042e-01,  -3.36477451e-16,
                          3.33333333e-01,  -3.36477451e-16,   1.00000000e+00,
                          5.77304037e-17,  -3.33333333e-01,  -5.77304037e-17,
                         -9.42809042e-01,   2.93333281e-03,   1.73641029e-17,
                          1.72817560e-03,   0.00000000e+00],
                       [ -9.42809042e-01,   3.36477451e-16,  -3.33333333e-01,
                          3.36477451e-16,   1.00000000e+00,   5.77304037e-17,
                          3.33333333e-01,  -5.77304037e-17,  -9.42809042e-01,
                         -2.96333281e-03,  -3.81418526e-17,   1.81302841e-03,
                         -9.45463674e-04,   3.33333333e-01,  -5.77304037e-17,
                         -9.42809042e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.51542409e-03,
                         -2.00000000e-02,   0.00000000e+00,   5.00000000e-02,
                          5.48000224e-01,  -9.42809042e-01,   3.36477451e-16,
                         -3.33333333e-01,   3.36477451e-16,   1.00000000e+00,
                          5.77304037e-17,   3.33333333e-01,  -5.77304037e-17,
                         -9.42809042e-01,  -2.93333281e-03,  -3.81470483e-17,
                          1.72817560e-03,   0.00000000e+00],
                       [  9.42809042e-01,   2.97149141e-17,   3.33333333e-01,
                          2.97149141e-17,   1.00000000e+00,  -1.73191211e-16,
                         -3.33333333e-01,   1.73191211e-16,   9.42809042e-01,
                          2.96333281e-03,   3.11588308e-17,  -1.81302841e-03,
                         -9.45463674e-04,  -3.33333333e-01,   1.73191211e-16,
                          9.42809042e-01,   0.00000000e+00,   0.00000000e+00,
                          1.00000000e+00,   0.00000000e+00,   3.53295021e-03,
                          2.00000000e-02,   0.00000000e+00,  -5.00000000e-02,
                          5.48000224e-01,   9.42809042e-01,   2.97149141e-17,
                          3.33333333e-01,   2.97149141e-17,   1.00000000e+00,
                         -1.73191211e-16,  -3.33333333e-01,   1.73191211e-16,
                          9.42809042e-01,   2.93333281e-03,   3.11744180e-17,
                         -1.72817560e-03,   0.00000000e+00]])

side_pre_grasps = pre_grasps[1:7]

pr2_l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, 
                    -0.0962141, -0.0864407]

pr2_r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, 
                    -1.4175, -1.8417, 0.21436]

def pr2_tuck_arm(robot):
    robot.SetDOFValues(pr2_r_arm_tucked, 
                       robot.GetManipulator("rightarm").GetArmIndices())
    robot.SetDOFValues(pr2_l_arm_tucked, 
                       robot.GetManipulator("leftarm").GetArmIndices())

def get_pr2_torso_limit(robot):
    torso = robot.GetJoint('torso_lift_joint')
    return torso.GetLimits()
    
def get_environment_limits(env, robot= None):
    """Calculates the limits of an environment, If robot is not None then then
    limits are shrunk by the size of the robot.
    
    Return:
    envmin, envmax: two 1x3 arrays with the environment minimun and maximum 
    extension.
    """
    
    if robot is not None:
        abrobot = robot.ComputeAABB()
        min_r = abrobot.extents()
        max_r = -abrobot.extents()    
    else:
        min_r = max_r = 0
    
    with env:
        envmin = []
        envmax = []
        for b in env.GetBodies():
            ab = b.ComputeAABB()
            envmin.append(ab.pos()-ab.extents())
            envmax.append(ab.pos()+ab.extents())
        envmin = np.min(np.array(envmin),0) + min_r
        envmax = np.max(np.array(envmax),0) + max_r
        
    return envmin, envmax


def test_position(manip, env, gripper_angle = None):
    if gripper_angle is None:
        gripper_angle = (np.deg2rad(180), np.deg2rad(0), np.deg2rad(0))
        
    rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)
    T = manip.GetEndEffectorTransform()
    T[:3,:3] = rot_mat
    p2 = np.array([0.1, 0, 0, 1]);
    arrow = env.drawarrow(np.dot(T,p1), np.dot(T, p2), 0.01, np.array([1.0,0,0]))
    sol = manip.FindIKSolution(T, 
                               openravepy.IkFilterOptions.IgnoreEndEffectorCollisions)
    if sol is not None:
        robot.SetDOFValues(sol, robot.GetActiveManipulator().GetArmIndices())
    else:
        print "NO SOLUTION"
    
    return arrow

def get_object_limits(obj):
    """Returns the bounding box of an object.
    
    Returns: min_x, max_x, min_y, max_y, z
    """
    
    ab = obj.ComputeAABB()
    max_x = ab.pos()[0] + ab.extents()[0]
    min_x = ab.pos()[0] - ab.extents()[0]
    
    max_y = ab.pos()[1] + ab.extents()[1]
    min_y = ab.pos()[1] - ab.extents()[1]     
    z = ab.pos()[2] + ab.extents()[2]
    
    return min_x, max_x, min_y, max_y, z
    
def create_random_object(surface, env, dimensions, name=None):
    extent_x, extent_y, extent_z = dimensions
    min_x, max_x, min_y, max_y, z = get_object_limits(surface)
    z = z + extent_z + 0.00
    
    x = np.random.uniform(min_x, max_x)
    y = np.random.uniform(min_y, max_y)
    values = [0,0,0, extent_x, extent_y, extent_z]
    body = openravepy.RaveCreateKinBody(env,'')
    if name is not None:
        body.SetName(name)    
    else:
        body.SetName("random_object")
        
    body.InitFromBoxes(np.array([values]), True)
    T = body.GetTransform()
    T[:,-1] = [x,y,z,1]
    body.SetTransform(T)
    if name is not None:
        env.Add(body, False)
    else:
        env.Add(body, True)
    return body

def create_collision_free_random_object(surface, env, dimensions, 
                                        name = None,
                                        num_trials = 20
                                        ):
    
    env.GetCollisionChecker().SetCollisionOptions(openravepy.CollisionOptions.Contacts)
    for iteration in xrange(num_trials):        
        body = create_random_object(surface, env, dimensions, name)
        report = openravepy.CollisionReport()
        collision = env.CheckCollision(body, report)
        if collision:
            if report.plink2.GetParent() == surface:
                #only collision is with the surface, everything is ok
                break
            else:
                #colliding with something else
                env.Remove(body)
                body = None
        else:
            break
        
    return body
            
def get_all_collisions(obj, env):
    """Returns a set with all the objects in collision with obj, empty if no
    collision """
    collisions = set()
    report = openravepy.CollisionReport()
    for link in obj.GetLinks():
        if env.CheckCollision(link, report=report):
            collisions.add(report.plink2.GetParent())
    
    return collisions


def setGoalObject(objName, pddlFile):
    pddlStr = open(pddlFile).read()
    goalStr = "At " + objName + " table6"
    outStr = re.sub(r'At \w* table6', goalStr, pddlStr)
    outf =  pddlFile.replace(".pddl", "_edited.pddl")
    f = open(outf, 'w')
    f.write(outStr)
    return outf

def find_nearest_box(obj, box_msgs):
    obj_x = obj.GetTransform()[0][3]
    obj_y = obj.GetTransform()[1][3]
    obj_z = obj.GetTransform()[2][3]

    closest_dist = float("inf")
    best_box_msg = None
    index = 0
    for i, box_msg in enumerate(box_msgs):
        box_x = box_msg.pose.pose.position.x
        box_y = box_msg.pose.pose.position.y
        box_z = box_msg.pose.pose.position.z

        dist = np.sqrt(np.power(box_x - obj_x, 2) + \
                       np.power(box_y - obj_y, 2) + \
                       np.power(box_z - obj_z, 2))
        if dist < closest_dist:
            best_box_msg = box_msg
            closest_dist = dist
            index = i

    return best_box_msg, index
