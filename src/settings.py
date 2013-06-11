path = "../domains/"
errFileName = "robotics_autogen_err1.txt"

ff = "../planners/FF-v2.3/ff"
fd = "../planners/FD/src/plan-ipc seq-sat-fd-autotune-1 "
pddlDomainFile = path #+ raw_input("Enter PDDL domain file name: ")
if pddlDomainFile == path:
    #pddlDomainFile = path+ "robotics_obstrn_compiled_dom2.pddl"
    pddlDomainFile = path+ "robotics_obstrn_compiled_dom2_typed.pddl"

     # for ff, dinnertime:
     #pddlDomainFile = path+ "dinnerTime_dom.pddl"   
     #pddlDomainFile = path+ "dinnerTimeNoNegation_dom.pddl"
    #pddlDomainFile = path+ "dinnerTimeNoNegationCosts_dom.pddl"

initialProblemFile = path #+ raw_input("Enter PDDL problem file name: ")
if initialProblemFile == path:
    #initialProblemFile = path + "robotics_autogen_prob_65objs_typed.pddl"
    initialProblemFile = path + "canworld_gazebo_12cans.pddl"
    
    
    #for ff, dinnertime:
    #initialProblemFile = path + "dinnerTime_prob.pddl"
    #initialProblemFile = path + "dinnerTimeNoNegation_prob.pddl"
    #initialProblemFile = path + "dinnerTimeNoNegationCostsTargets_prob.pddl"


collision_free_grasping_samples = 1
occluding_objects_grasping_samples = 1

doJointInterpretation = False
use_ros = False
OpenRavePlanning = False

envFile = "created_info.dae"
#envFile = "tray_world.dae"
#envFile = "boxes65.dae"