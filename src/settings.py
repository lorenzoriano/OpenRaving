path = "../domains/"
errFileName = "robotics_autogen_err1.txt"

ff = "../planners/FF-v2.3/ff"
pddlDomainFile = path #+ raw_input("Enter PDDL domain file name: ")
if pddlDomainFile == path:
    pddlDomainFile = path+ "robotics_obstrn_compiled_dom2.pddl"
initialProblemFile = path #+ raw_input("Enter PDDL problem file name: ")
if initialProblemFile == path:
    initialProblemFile = path + "robotics_autogen_prob.pddl"


collision_free_grasping_samples = 500
occluding_objects_grasping_samples = 200
envFile = "boxes.dae"
