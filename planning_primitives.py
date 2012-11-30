import openravepy
import utils
import inspect
import generate_reaching_poses
import time

class Executor(object):
    def __init__(self, robot):
        self.robot = robot
        utils.pr2_tuck_arm(robot)
        self.env = robot.GetEnv ()
    
    def moveto(self, _unused1, pose):
        if type(pose) is str:
            print "Pose ", pose, " ignored"
            return
        
        else:
            print "Moving to pose "
            self.robot.SetTransform(pose)
    
    def grasp(self, obj_name, _unused1, _unused2):
        print "Grasping object ", obj_name
        obj = self.env.GetKinBody(obj_name)
        if obj is None:
            raise ValueError("Object %s does not exist" % obj_name)
        
        try:
            sol, torso_angle = generate_reaching_poses.get_torso_grasping_pose(
                                                                              self.robot, 
                                                                              obj,
                                                                              )
        except generate_reaching_poses.GraspingPoseError():
            raise
        
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
            sol, torso_angle = generate_reaching_poses.get_torso_surface_pose(
                                                                                      self.robot, 
                                                                                      table,
                                                                                      )
        except generate_reaching_poses.GraspingPoseError():
            raise
                
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

class Uninterpreted(object):
    pass

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
            
            if l.startswith('#'):
                continue
            if len(l) < 2:
                continue
            
            tokens_list = l.split(" ")[1:] #skip instruction number
            function_name = tokens_list[0].lower()
            
            method = getattr(self.executor, function_name, None)
            if method is None:
                raise TypeError("Object %s does not have a method called %s" %(
                    self.executor.__class__.__name,
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
                #grasping location
                if arg.startswith("GP_"):
                    self.grasping_locations.add(arg)
                    self.need_binding.add(arg)
                #base location
                elif arg.startswith("BLF_"):
                    self.base_locations.add(arg)
                    self.need_binding.add(arg)
                
                function['args'].append(arg)
            
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
        time.sleep(timestep)
        
        for instruction in self.parsing_result:            
            action_name =  instruction["name"] + "(" + ", ".join( instruction['args']) + ")"
            print "Executing action ", action_name
            
            method = instruction['method']
            
            args = [self.bind_variable(a) for a in instruction['args']]
            
            method(*args)
            time.sleep(timestep)
            
        print "Done"
        
def test():
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.Load('/home/pezzotto/Projects/OpenRaving/boxes.dae');
    robot = env.GetRobots()[0]; manip = robot.SetActiveManipulator('rightarm')
    ex = Executor(robot); 
    parser = PlanParser("plan.txt", ex);
    parser.execute(2)
    
if __name__ == "__main__":
    test()
    raw_input("Press a button to continue")