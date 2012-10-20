#! /usr/bin/python
import rosbag
import openravepy
import numpy as np
import sys

def load_env_from_bag(bagname):
    bag = rosbag.Bag(bagname)
    env = openravepy.Environment()
    topic, msg, stamp = bag.read_messages('/collision_map_occ').next()
    
    num_boxes = len(msg.boxes)
    for i,box in enumerate(msg.boxes):
        body = openravepy.RaveCreateKinBody(env,'')
        body.SetName("obstacle_%d" %i)
        values = [0,
                  0,
                  0,
                  box.extents.x,
                  box.extents.y,
                  box.extents.z,
                  ]
        body.InitFromBoxes(np.array([values]), True)
        T = body.GetTransform()
        T[:3,-1] = [box.center.x, box.center.y, box.center.z]
        body.SetTransform(T)        
        env.Add(body, False)    
    return env

def load_body_from_bag(bagname, env):
    bag = rosbag.Bag(bagname)    
    topic, msg, stamp = bag.read_messages('/collision_map_occ').next()

    all_boxes = np.empty( (len(msg.boxes), 6) )
    body = openravepy.RaveCreateKinBody(env,'')
    body.SetName("obstacle")    
    for i,box in enumerate(msg.boxes):        
        values = [box.center.x,
                  box.center.y,
                  box.center.z,
                  box.extents.x,
                  box.extents.y,
                  box.extents.z,
                  ]        
        all_boxes[i, :] = values
    
    centroid = np.mean(all_boxes[:,:3], axis=0)
    all_boxes[:,:3] -= centroid
    body.InitFromBoxes(all_boxes, True)
    
    T = body.GetTransform()    
    T[:3,-1] = centroid
    body.SetTransform(T)        
    return body
    
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print "Usage: %s bagname, environment_file_name" % sys.argv[0]
        sys.exit(1)
    
    bagname, env_name = sys.argv[1:]
    env = load_env_from_bag(bagname)
    env.Save(env_name)
    