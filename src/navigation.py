import time
import numpy as np
import openravepy

import utils

class SimpleNavigationPlanning:
    def __init__(self, robot):
        self.env = robot.GetEnv()
        self.robot = robot
        self.cdmodel = openravepy.databases.convexdecomposition.ConvexDecompositionModel(self.robot)
        if not self.cdmodel.load():
            self.cdmodel.autogenerate()
        self.basemanip = openravepy.interfaces.BaseManipulation(self.robot)
        
    def performNavigationPlanning(self, goal, execute = True, 
                                  draw_marker = False, steplength = 0.1):
        goal = np.array(goal)
        if goal.ndim == 2:
            #assume a 4x4 transformation matrix
            angle = openravepy.axisAngleFromRotationMatrix(goal)[-1]
            goal = np.array([goal[0,-1], goal[1,-1], angle])
            
        # find the boundaries of the environment
        envmin, envmax = utils.get_environment_limits(self.env, self.robot)        

        with self.env:
            self.robot.SetAffineTranslationLimits(envmin,envmax)
            self.robot.SetAffineTranslationMaxVels([0.3,0.3,0.3])
            self.robot.SetAffineRotationAxisMaxVels(np.ones(4))
            self.robot.SetActiveDOFs([],
                openravepy.DOFAffine.X|openravepy.DOFAffine.Y|openravepy.DOFAffine.RotationAxis,
                [0,0,1])

        # draw the marker
        if draw_marker:
            center = np.r_[goal[0:2],0.2]
            xaxis = 0.5* np.array((np.cos(goal[2]),np.sin(goal[2]),0))
            yaxis = 0.25*np.array((-np.sin(goal[2]),np.cos(goal[2]),0))
            h = self.env.drawlinelist(np.transpose(np.c_[center-xaxis,
                                                         center+xaxis,
                                                         center-yaxis,
                                                         center+yaxis,
                                                         center+xaxis,
                                                         center+0.5*xaxis+0.5*yaxis,
                                                         center+xaxis,
                                                         center+0.5*xaxis-0.5*yaxis]),
                                      linewidth=5.0,
                                      colors=np.array((0,1,0)))

        openravepy.RaveLogInfo("Planning to goal " + str(goal))
        res = self.basemanip.MoveActiveJoints(goal = goal,
                                           maxiter = 3000,
                                           steplength = steplength,
                                           execute = execute,
                                           outputtrajobj = True
                                           ) 
        if res is None:
            raise ValueError("Could not find a trajectory")
        
        if execute:
            openravepy.RaveLogInfo("Waiting for controller to finish")
            self.robot.WaitForController(0)
            self.robot.GetController().Reset()
        return res