#! /usr/bin/python

pr2_l_arm_tucked = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, 
                    -0.0962141, -0.0864407]

pr2_r_arm_tucked = [-0.023593, 1.1072800, -1.5566882, -2.124408, 
                    -1.4175, -1.8417, 0.21436]

def pr2_tuck_arm(robot):
    robot.SetDOFValues(pr2_r_arm_tucked, 
                       robot.GetManipulator("rightarm").GetArmIndices())
    robot.SetDOFValues(pr2_l_arm_tucked, 
                       robot.GetManipulator("leftarm").GetArmIndices())

def main():
    import openravepy; 
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.Load('data/pr2test2.env.xml');
    robot = env.GetRobots()[0];
    
    #disable physics
    for l in robot.GetLinks():
        l.SetStatic(True)
    
    raw_input("press a key to tuck arms")
    #move in a tuck position
    pr2_tuck_arm(robot)
    

if __name__ == "__main__":
    main()
    raw_input("press a key to exit")