import openravepy
import utils
import inspect
import generate_reaching_poses
import reachability
import time
import tray_world
import numpy as np
from settings import *
import sys
import pdb
import numpy

if OpenRavePlanning:
    from openrave_tests import test_grasp_move



try:
    from tf import transformations
    from geometry_msgs.msg import PoseStamped

    from pr2model import PR2Robot
    from pr2_control_utilities.pr2_planning import PR2MoveArm
    from pr2_control_utilities.pr2_joint_mover import PR2JointMover

    import grasp_generator
    import object_pickup

    detector_and_cluster_map = None
except:
    print "Warning: ROS imports failed. Okay if not using ROS."

class ExecutingException(Exception):
    """This is a general exception that returns information on failure for an execution step.
    """
    def __init__(self, problem, line_number = None):
        self.line_number = line_number
        self.problem = problem
        self.robot = None
        self.object_to_grasp = None
        self.collision_list = None
        self.pddl_error_info = ""
        
    def __repr__(self):
        return "ExecutingException, line %d, problem %s" % (self.line_number, self.problem)

class Executor(object):
    def __init__(self, robot, viewer = False):
        self.robot = robot
        self.env = robot.GetEnv ()
        self.grasping_locations_cache = {}
        self.viewMode = viewer
        self.tray_stack = []
        self.unMovableObjects = ['table']
        self.objSequenceInPlan = []
        self.handled_objs = set()
        
        v = self.robot.GetActiveDOFValues()
        v[self.robot.GetJoint('torso_lift_joint').GetDOFIndex()]=1.0
        self.robot.SetDOFValues(v)      

        if use_ros:
            self.pr2robot = PR2Robot(self.env)
            self.pr2robot.robot = self.robot
            self.arm_mover = PR2MoveArm()
        
        #loading the IK models
        utils.pr2_tuck_arm(robot)
        robot.SetActiveManipulator('leftarm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
                        robot,iktype=openravepy.IkParameterization.Type.Transform6D)    
        if not ikmodel.load():
            ikmodel.autogenerate()            
        robot.SetActiveManipulator('rightarm')
        ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
            robot,iktype=openravepy.IkParameterization.Type.Transform6D)    
        if not ikmodel.load():
            ikmodel.autogenerate()

    def getGoodBodies(self):
        if not doJointInterpretation:
            body_filter = lambda b: b.GetName().startswith("random") or\
                                    b.GetName().startswith('object')
        else:
            futureObjects = set(self.objSequenceInPlan) - self.handled_objs
            body_filter = lambda b: (b.GetName() not in futureObjects) \
                                   and (b.GetName() not in self.unMovableObjects)
        return filter(body_filter, self.env.GetBodies())

    def setObjSequenceInPlan(self, objList):
        self.objSequenceInPlan = objList
        print 
        print "Will try to pick objs in the order " + repr(objList)


    def clear_gp_cache(self):
        self.grasping_locations_cache = {}
        self.objSequenceInPlan = []

    def pause(self, msg = None):
        if self.viewMode:
            if msg is None:
                raw_input("Press return to continue")
            else:
                print msg
                #time.sleep(0.5)
                raw_input(msg + "... [press return]")
    
    def moveto(self, _unused1, pose):
        if type(pose) is str:
            print "Pose ", pose, " ignored"
            return
        
        else:
            print "Moving to pose "
            self.robot.SetTransform(pose)
    
    def movetowithinr1(self, _unused1, unused2):
        print "Ignore me!"
    def movetowithinr2(self, _unused1, unused2):
            print "ignore me !"    
#    def movetoacrossrooms(self, _unused1, unused2, unused):
#        print "ignore me !"    
    def movetoacrossrooms(self, _unused1, unused2):
        print "ignore me !"    
    
    def grasp(self, obj_name, _unused1, _unused2):
        # update openrave
        if use_ros:
            self.pr2robot.update_rave()

        print "Grasping object ", obj_name
        obj = self.env.GetKinBody(obj_name)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_name)
        
        cached_value = self.grasping_locations_cache.get(obj_name, None)
        if cached_value is None:
            print "Object %s is not cached, looking for a value" % obj_name
            try:
                pose, sol, torso_angle = generate_reaching_poses.get_collision_free_grasping_pose(
                                                                                  self.getGoodBodies(),
                                                                                  self.robot, 
                                                                                  obj,
                                                                                  max_trials=collision_free_grasping_samples
                                                                                  )
                self.grasping_locations_cache[obj_name] =  pose, sol, torso_angle
            except generate_reaching_poses.GraspingPoseError:
                e = ExecutingException("Object in collision")
                e.robot = self.robot
                e.object_to_grasp = obj
                raise e
        else:
            print "Object %s already cached" %  obj_name
            pose, sol, torso_angle = cached_value
        
        if OpenRavePlanning:
            test_grasp_move.move_arm(robot, pose, 'r')
        
        if use_ros:
            # # object matching
            detector, _ = detector_and_cluster_map
            # index = cluster_map[obj_name]

            # # generating grasps
            # box_msg = detector.detect_bounding_box(detector.last_detection_msg.detection.clusters[index])
            # desired_grasps = grasp_generator.generate_grasps(box_msg, 8)
            # grasp_generator.draw_grasps(desired_grasps)

            # object matching
            detector.detect()
            box_msgs = []
            for cluster in detector.last_detection_msg.detection.clusters:
                box_msgs.append(detector.detect_bounding_box(cluster))
            box_msg, index = utils.find_nearest_box(obj, box_msgs)

            # generating grasps
            desired_grasps = grasp_generator.generate_grasps(box_msg, 16)
            grasp_generator.draw_grasps(desired_grasps)

            # pickup
            grabber = object_pickup.Grabber()
            processing_reply = detector.call_collision_map_processing(detector.last_detection_msg)
            
            # # reset planning scene
            self.arm_mover.reset_collision_map()
            self.arm_mover.update_planning_scene()

            res = grabber.pickup_object(processing_reply.graspable_objects[index],
                                        processing_reply.collision_object_names[index],
                                        processing_reply.collision_support_surface_name,
                                        'right_arm',
                                        desired_grasps=desired_grasps,
                                        lift_desired_distance = 0.3)
            if res is None:
                e = ExecutingException("ROS pickup failed")
                e.robot = self.robot
                e.object_to_grasp = obj
                raise e

            # update openrave
            if use_ros:
                self.pr2robot.update_rave()
        else:
            self.pause("Moving to location")
            self.robot.SetTransform(pose)
            
            self.pause("Moving arm")
            self.robot.SetDOFValues([torso_angle],
                                    [self.robot.GetJointIndex('torso_lift_joint')])        
            self.robot.SetDOFValues(sol,
                                    self.robot.GetActiveManipulator().GetArmIndices())

        self.robot.Grab(obj)
    
    def putdown(self, obj_name, table_name, _unused1):
        
        print "Putting down object %s on %s" %(obj_name, table_name)
        obj = self.env.GetKinBody(obj_name)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_name)
        
        if table_name.startswith("dest_"):
            #this is a fixed location!!!
            T = getattr(tray_world, table_name, None)            
            if T is None:
                raise ValueError("The location %s is unknown! check spelling?" % table_name)
            T = T.copy()
            #put the gripper facing down
            gripper_angle = (np.pi, 0., 0) #just got this from trial and test
            rot_mat = openravepy.rotationMatrixFromAxisAngle(gripper_angle)            
            T[:3,:3] = rot_mat            
            T[2, 3] += 0.03
            
            try:
                (pose,
                 sol,
                 torso_angle) = generate_reaching_poses.get_collision_free_ik_pose(
                     self.getGoodBodies(),
                     self.env,
                     obj,
                     self.robot,
                     T,
                     only_reachable=False,
                     max_trials =1000
                 )
            except generate_reaching_poses.GraspingPoseError:
                raise ExecutingException("Putting down on location has problems!")            
        
        else:           
            table = self.env.GetKinBody(table_name)        
            if table is None:
                raise ValueError("Object %s does not exist" % table_name)

            try:
                pose, sol, torso_angle = generate_reaching_poses.get_collision_free_surface_pose(self.getGoodBodies(),
                                                                                          self.robot, 
                                                                                          table,
                                                                                          )
            except generate_reaching_poses.GraspingPoseError, e:
                raise e
        
        if use_ros:
            # Move arm to drop location
            p = (0.3, -0.5, 0.3)
            q = transformations.quaternion_from_euler(0, 0, -numpy.pi/2)
            res = self.arm_mover.move_right_arm(p, q, '/torso_lift_link', 30)
            if not res:
                e = ExecutingException("ROS putdown step 1 failed")
                e.robot = self.robot
                e.object_to_grasp = obj
                raise e

            p = (0.1, -0.8, -0.2)
            q = transformations.quaternion_from_euler(0, 0, -numpy.pi/2)
            res = self.arm_mover.move_right_arm(p, q, '/torso_lift_link', 30)
            if not res:
                e = ExecutingException("ROS putdown step 2 failed")
                e.robot = self.robot
                e.object_to_grasp = obj
                raise e

            # Drop object
            joint_mover = PR2JointMover()
            joint_mover.open_right_gripper(True)

            # update openrave
            if use_ros:
                self.pr2robot.update_rave()
        else:
            self.pause("Moving to location")
            self.robot.SetTransform(pose)
            
            self.pause("Moving arm")
            self.robot.SetDOFValues([torso_angle],
                                    [self.robot.GetJointIndex('torso_lift_joint')])
            self.robot.SetDOFValues(sol,
                                    self.robot.GetActiveManipulator().GetArmIndices())

        self.robot.Release(obj)
        
        #putting the object straight
        if table_name.startswith("dest_"):
            print "Putting object in the right location"
            T = getattr(tray_world, table_name, None)
        else:
            T = obj.GetTransform()
            rot_angle = (np.pi / 2., 0., 0) #got this from the model
            rot_mat = openravepy.rotationMatrixFromAxisAngle(rot_angle)
            T[:3, :3] = rot_mat
            _, _, _, _, z = utils.get_object_limits(table)
            T[2, 3] = z
        
        obj.SetTransform(T)
        try:
            del self.grasping_locations_cache[obj_name]
        except KeyError:
            print "funny, object ", obj_name, " was not in cache.. something wrong here!"
            raw_input("Press return to continue")    
        #self.pause()
        
    def find_gp(self, object_to_grasp) :
        """"Find a grasping pose
        """"" 
        
        print "Getting a collision free grasping pose"
        robot_pose, _, _= generate_reaching_poses.get_collision_free_grasping_pose(
                                                                                               self.robot, 
                                                                                               object_to_grasp, 
                                                                                               )
        return robot_pose
    
    def find_bp(self, surface):
        """""Find a base pose for putting down
        """""
        
        print "Getting base pose for putting down"
        robot_pose, _, _= generate_reaching_poses.get_collision_free_surface_pose(self.getGoodBodies(),
                                                                                                       self.robot, 
                                                                                                       surface, 
                                                                                                       )
        return robot_pose        

    def placeontray(self, unused1, obj_name, unused2, tray_name, unused4):
        """Put an item on the tray.
        """
        tray = self.env.GetKinBody(tray_name)
        if tray is None:
            raise ValueError("Object %s does not exist" % tray_name)        
        obj =  self.env.GetKinBody(obj_name)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_name)

        if (not len(self.tray_stack) == 0 and
            not tray_world.can_stack(self.tray_stack[-1], obj)):
            e = ExecutingException("Incompatible objects")
            e.robot = self.robot
            e.object_to_grasp = obj
            e.stacktop = self.tray_stack[-1]
            raise e
        
        T = tray_world.tray_putdown_pose(tray, self.tray_stack)
        try:
            (pose,
             sol,
             torso_angle) = generate_reaching_poses.get_collision_free_ik_pose(
                 self.getGoodBodies(),
                 self.env,
                 obj,
                 self.robot,
                 T,
             )
        except generate_reaching_poses.GraspingPoseError:
            raise ExecutingException("Putting down on tray has problems!")
        
        self.pause("Going to the tray")
        self.robot.SetTransform(pose)
        
        self.pause("Arm/Torso in position")
        self.robot.SetDOFValues([torso_angle],
                                [self.robot.GetJointIndex('torso_lift_joint')])
        self.robot.SetDOFValues(sol,
                                self.robot.GetActiveManipulator().GetArmIndices())
        print "Releasing object"
        self.robot.Release(obj)
        self.tray_stack.append(obj)
        
        #putting the object straight
        rot_angle = (np.pi / 2., 0., 0) #got this from the model
        rot_mat = openravepy.rotationMatrixFromAxisAngle(rot_angle)
        T[:3, :3] = rot_mat
        T[2,3] -= 0.05        
        obj.SetTransform(T)

        try:
            del self.grasping_locations_cache[obj_name]
        except KeyError:
            print "funny, object ", obj_name, " was not in cache.. something wrong here!"
            print "cache is: ", self.grasping_locations_cache
            raw_input("Press return to continue")            
  
        print "Back to rest"
        utils.pr2_tuck_arm(self.robot)
        print "The tray now has: ", self.tray_stack
        #self.pause()
    
    def picktray(self, unused1, tray_name, unused2):
        """Grabs the tray and all the items on it
        """
        tray = self.env.GetKinBody(tray_name)
        if tray is None:
            raise ValueError("Object %s does not exist" % tray_name)
        
        if len(self.tray_stack) > 0 and (np.random.uniform() <= 0.0):
            e = ExecutingException("Tray is heavy!")
            e.robot = self.robot
            e.object_to_grasp = tray
            raise e        
        
        self.pause("Moving in front of the tray")
        tray_world.move_robot_base_infront_tray(self.robot, tray)

        self.pause("Grasping the tray")
        tray_world.put_left_arm_over_tray(self.env, self.robot, tray)
        tray_world.put_right_arm_over_tray(self.env, self.robot, tray)
        self.robot.Grab(tray)        
        for obj in self.tray_stack:
            self.robot.Grab(obj)
        
        #self.pause()
    
    def putdowntray(self, unused1, tray_name, tray_loc):
        """Move the robot to the tray goal location and releases the tray        
        """
        tray = self.env.GetKinBody(tray_name)
        if tray is None:
            raise ValueError("Object %s does not exist" % tray_name)
        
        if tray_loc == "trayloc2":            
            T = tray_world.tray_destination
        else:
            T = tray_world.tray_initial_loc

        self.pause("Moving to target position")
        tray_world.move_robot_base_infront_tray(self.robot, T)
        
        print "Releasing the tray"
        self.robot.Release(tray)
        for obj in self.tray_stack:
            self.robot.Release(obj)
        self.tray_stack = []
        utils.pr2_tuck_arm(self.robot)
        #self.pause()
    
    def pickfromtray(self, unused1, tray_name, obj_name, unused2, unused3):
        tray = self.env.GetKinBody(tray_name)
        if tray is None:
            raise ValueError("Object %s does not exist" % tray_name)
        obj = self.env.GetKinBody(obj_name)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_name)        
        
        self.grasp(obj_name, unused1, unused2)
        try:
            self.tray_stack.remove(obj)
        except ValueError:
            print "Apperntly %s is not on the tray. Never mind!" % (obj,
                                                                              )
            

class PlanParser(object):
    def __init__(self, file_object_or_name, executor):
        if type(file_object_or_name) is str:
            file_obj = open(file_object_or_name)
        else:
            file_obj = file_object_or_name
        
        isinstance(executor, Executor)
        self.executor = executor   
        
        self.grasping_locations = set()
        self.bindings = {}
        self.base_locations = set()
        self.need_binding = set()
        self.parsing_result = None
        
        self.parse(file_obj)  
        
        self.handled_objs = set()
    
    def updateHandledObjs(self, args):
        self.executor.handled_objs = self.handled_objs
        return
    
    def parse(self, file_obj): 
        functions = []
        for l in file_obj:            
            function = {}
            stripped_line =  l.strip(" \t\n")
            
            if stripped_line.startswith('#'):
                continue
            if len(stripped_line) < 2:
                continue
            
            tokens_list = stripped_line.split(" ")[1:] #skip instruction number
            function_name = tokens_list[0].lower()
            
            method = getattr(self.executor, function_name, None)
            if method is None:
                raise TypeError("Object %s does not have a method called %s" %(
                    self.executor.__class__.__name__,
                    function_name))
            
            function['name'] = function_name
            function['method'] = method
            function['args'] = []
            num_args = len(inspect.getargspec(method).args[1:]) #remove self
            num_variables = len(tokens_list[1:]) #remove function name
            if num_args != num_variables:
                raise TypeError("Wrong arguments for %s, expected %d, got %d" %(
                    function_name, num_args, num_variables))
            
            args = (t.strip("\n")
                    for t in tokens_list[1:] if len(t) > 0)
            for arg in args:
                #grasping location ignored!
                if arg.startswith("gp_"):
                    pass
                    #self.grasping_locations.add(arg)
                    #self.need_binding.add(arg)
                #base location
                elif arg.startswith("blf_"):
                    self.base_locations.add(arg)
                    self.need_binding.add(arg)
                
                function['args'].append(arg.lower())
            
            functions.append(function)
        self.parsing_result = functions

    def bind_grasping_location(self, var):
        obj_label = "_".join(var.split('_')[1:])
        obj = self.executor.env.GetKinBody(obj_label)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_label)
        
        pose =  self.executor.find_gp(obj)
        self.bindings[var] = pose
        return pose

    def bind_base_location(self, var):
        table_label = "_".join(var.split('_')[1:])
        table = self.executor.env.GetKinBody(table_label)
        if table is None:
            raise ValueError("Object %s does not exist" % table_label)
        
        pose = self.executor.find_bp(table)
        self.bindings[var] = pose
        return pose
        
    def bind_variable(self, var):
        if var not in self.need_binding:
            return var
        
        elif var in self.bindings:
            return self.bindings[var]

        elif var in self.base_locations:
            return self.bind_base_location(var)

        elif var in self.grasping_locations:
            return self.bind_grasping_location(var)
    
    def execute(self, timestep = 0.5):
        print "Starting..."
        #time.sleep(timestep)
        
        for lineno, instruction in enumerate(self.parsing_result):
            action_name =  instruction["name"] + "(" + ", ".join( instruction['args']) + ")"
            print "Executing action ", action_name
            
            if action_name == 'grasp':
                self.updateHandledObjs(args)
            
            method = instruction['method']
            
            args = [self.bind_variable(a) for a in instruction['args']]
        
            try:
                method(*args)
            except ExecutingException, e:
                e.line_number = lineno
                self.handle_error(e)
                raise e
                
            time.sleep(timestep)
            #raw_input("Press a button to continue")
            
        print "Done"
        
    def handle_error(self, error):
        assert isinstance(error, ExecutingException)
        robot = error.robot
        obj = error.object_to_grasp
        
        print "Handling an error"
        if "collision" in error.problem:
            print "Got a collision error, finding occlusions"
            if not doJointInterpretation:
                body_filter = lambda b: b.GetName().startswith("random") or\
                                        b.GetName().startswith('object')
            else:
                futureObjects = set(self.executor.objSequenceInPlan) - self.handled_objs
                body_filter = lambda b: (b.GetName() not in futureObjects) \
                                       and (b.GetName() not in self.executor.unMovableObjects)
                
            (pose,
             sol, torso_angle,
             collision_list) = reachability.get_occluding_objects_names(self.executor.getGoodBodies(),
                                                         robot,
                                                         obj,
                                                         body_filter,
                                                         occluding_objects_grasping_samples,
                                                         just_one_attempt=True,
                                                         return_pose=True)
            
            if len(collision_list) == 0:
                raise ExecutingException("No way I can grasp that object!", error.line_number)
            
            #updating the executor cache
            self.executor.grasping_locations_cache[obj.GetName()] =  pose, sol, torso_angle
            
            first_collisions = collision_list.pop()
            object_to_grasp_name = error.object_to_grasp.GetName()
            obst_list = "\n".join("(Obstructs %s %s %s)" %("gp_"+ object_to_grasp_name,
                                                           obstr, object_to_grasp_name) for obstr in first_collisions)
            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number, obst_list)
            return
        
        elif "heavy" in error.problem:
            print "They tray is too heavy!"
            msg = "(Heavy %s)" % error.object_to_grasp.GetName()
            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number,
                                                          msg )
            return
        elif "Incompatible" in error.problem:
            print "Objects are incompatible!"
            #msg = "(Bigger %s %s)" % (error.object_to_grasp.GetName(),
             #                         error.stacktop.GetName())
            msg = "(not (smaller %s %s))" % (error.object_to_grasp.GetName(),
                                             error.stacktop.GetName())

            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number,
                                                          msg )
        elif 'ROS pickup' in error.problem:
            print 'ROS pickup failed!'
        elif 'ROS putdown' in error.problem:
            print 'ROS putdown failed!'
        else:
            print "Don't know how to handle this problem!"

            
def initOpenRave(viewer = False, envFile = None):
    env = openravepy.Environment()
    if viewer: 
        env.SetViewer('qtcoin')

    env.Load(envFile);
    robot = env.GetRobots()[0];
    manip = robot.SetActiveManipulator('rightarm')
    ex = Executor(robot, viewer)
    return ex
        

def test(planFName, ex, objList):
    parser = PlanParser(planFName, ex);
    ex.setObjSequenceInPlan(objList)
    
    try:
        parser.execute(1)
        print "All ok"
    except ExecutingException, e:
        raise 

def clearGPCache(ex):
    print "clearing gp cache"
    ex.clear_gp_cache()
    
    
if __name__ == "__main__":
    global envFile
    initOpenRave(envFile)

    try:
        test("wrongplan.txt")
    except ExecutingException, e:
        print "Got an execution problem: ", e
        print "PDDL message is: ", e.pddl_error_info
        
    raw_input("Press a button to continue")
