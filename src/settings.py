path = "../domains/"
errFileName = "robotics_autogen_err1.txt"

ff = "../planners/FF-v2.3/ff"
fd = "../planners/FD/src/plan-ipc seq-sat-fd-autotune-1 "
pddlDomainFile = path #+ raw_input("Enter PDDL domain file name: ")
if pddlDomainFile == path:
#    pddlDomainFile = path+ "robotics_obstrn_compiled_dom2.pddl"
     # for ff, dinnertime:
     #pddlDomainFile = path+ "dinnerTime_dom.pddl"   
     pddlDomainFile = path+ "dinnerTimeNoNegation_dom.pddl"
    #pddlDomainFile = path+ "dinnerTimeNoNegationCosts_dom.pddl"

initialProblemFile = path #+ raw_input("Enter PDDL problem file name: ")
if initialProblemFile == path:
#    initialProblemFile = path + "robotics_autogen_prob.pddl"
    #for ff, dinnertime:
    #initialProblemFile = path + "dinnerTime_prob.pddl"
    initialProblemFile = path + "dinnerTimeNoNegation_prob.pddl"

    #initialProblemFile = path + "dinnerTimeNoNegationCosts_prob.pddl"


collision_free_grasping_samples = 500
occluding_objects_grasping_samples = 200
#envFile = "boxes50.dae"
envFile = "tray_world.dae"
