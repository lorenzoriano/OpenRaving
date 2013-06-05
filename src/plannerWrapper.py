#! /usr/bin/python

PKG='perception_manipulation'

import roslib
roslib.load_manifest(PKG)

import rospy
import openravepy

import hybridPlanner
import openrave_input

ENV_FILE_NAME = "created_info.dae"


if __name__=="__main__":
    rospy.init_node(PKG, anonymous=False)

    # load new environment
    env = openravepy.Environment()
    detector_and_cluster_map = openrave_input.create_openrave_bodies(env, False)
    # add custom table
    openrave_input.add_block(env, 'table6', 0, -1.5, 0.5, 0.5, 0.25, 0.005)
    env.Save(ENV_FILE_NAME)

    # run planner
    hybridPlanner.plan(detector_and_cluster_map, ENV_FILE_NAME, True)
