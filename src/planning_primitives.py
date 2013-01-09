import openravepy
import utils
import inspect
import generate_reaching_poses
import reachability
import time

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
    def __init__(self, robot):
        self.robot = robot
        utils.pr2_tuck_arm(robot)
        self.env = robot.GetEnv ()
        self.grasping_locations_cache = {}
    
    def moveto(self, _unused1, pose):
        if type(pose) is str:
            print "Pose ", pose, " ignored"
            return
        
        else:
            print "Moving to pose "
            self.robot.SetTransform(pose)
    
    def grasp(self, obj_name, _unused1, _unused2):
        #!TODO remember the grasping poses so to re-use them later!
        print "Grasping object ", obj_name
        obj = self.env.GetKinBody(obj_name)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_name)
        
        cached_value = self.grasping_locations_cache.get(obj_name, None)
        if cached_value is None:
            print "Object %s is not cached, looking for a value" % obj_name
            try:
                pose, sol, torso_angle = generate_reaching_poses.get_collision_free_grasping_pose(
                                                                                  self.robot, 
                                                                                  obj,
                                                                                  max_trials=500
                                                                                  )
                self.grasping_locations_cache[obj_name] =  pose, sol, torso_angle
            except generate_reaching_poses.GraspingPoseError:
                e = ExecutingException("Object in collision")
                e.robot = self.robot
                e.object_to_grasp = obj
                raise e
        else:
            print "Object %s already cached"
            pose, sol, torso_angle = cached_value
        
        self.robot.SetTransform(pose)
        self.robot.SetDOFValues([torso_angle],
                                [self.robot.GetJointIndex('torso_lift_joint')])
        self.robot.SetDOFValues(sol,
                                self.robot.GetActiveManipulator().GetArmIndices())
        self.robot.Grab(obj)
        utils.pr2_tuck_arm(self.robot)
    
    def putdown(self, obj_name, table_name, _unused1):
        print "Putting down object %s on %s" %(obj_name, table_name)
        table = self.env.GetKinBody(table_name)        
        if table is None:
            raise ValueError("Object %s does not exist" % table_name)
        obj = self.env.GetKinBody(obj_name)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_name)        
        
        try:
            pose, sol, torso_angle = generate_reaching_poses.get_collision_free_surface_pose(
                                                                                      self.robot, 
                                                                                      table,
                                                                                      )
        except generate_reaching_poses.GraspingPoseError:
            raise ExecutingException("Putdown has problems!")
                
        self.robot.SetTransform(pose)
        self.robot.SetDOFValues([torso_angle],
                                [self.robot.GetJointIndex('torso_lift_joint')])
        self.robot.SetDOFValues(sol,
                                self.robot.GetActiveManipulator().GetArmIndices())
        self.robot.Release(obj)
        utils.pr2_tuck_arm(self.robot)        
        
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
        robot_pose, _, _= generate_reaching_poses.get_collision_free_surface_pose(
                                                                                                       self.robot, 
                                                                                                       surface, 
                                                                                                       )
        return robot_pose        

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
            collision_list = reachability.get_occluding_objects_names(robot,
                                                         obj,
                                                         lambda b:b.GetName().startswith("random"),
                                                         100)
            if len(collision_list) == 0:
                raise ExecutingException("No way I can grasp that object!", error.line_number)
            first_collisions = collision_list.pop()
            object_to_grasp_name = error.object_to_grasp.GetName()
            obst_list = "\n".join("(Obstructs %s %s %s)" %("gp_"+ object_to_grasp_name,
                                                           obstr, object_to_grasp_name) for obstr in first_collisions)
            error.pddl_error_info = "LineNumber: %d\n%s" % (error.line_number, obst_list)
            return
        else:
            print "Don't know how to handle this problem!"
            
        
def test(planFName):
    env = openravepy.Environment()
    #env.SetViewer('qtcoin')
    env.Load('boxes.dae');
    robot = env.GetRobots()[0];
    manip = robot.SetActiveManipulator('rightarm')
    ex = Executor(robot); 
    parser = PlanParser(planFName, ex);
    
    try:
        parser.execute(1)
        print "All ok"
    except ExecutingException, e:
        raise 
    
    
    
if __name__ == "__main__":
    try:
        test("wrongplan.txt")
    except ExecutingException, e:
        print "Got an execution problem: ", e
        print "PDDL message is: ", e.pddl_error_info
        
    raw_input("Press a button to continue")
