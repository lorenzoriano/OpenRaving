#! /usr/bin/python

import utils
import random
import sys
import openravepy

def generate_n_objects(env, num_objects, dims = (0.1, 0.1, 0.3)):
    tables = ["table1", "table2", "table3", "table4"]
    
    created = 0
    for obj_num in range(num_objects):
        random.shuffle(tables)
        #do one attempt per table
        b = None
        for table in tables:
            table_body = env.GetKinBody(table)
            b = utils.create_collision_free_random_object(table_body, env, dims)
            if b is not None:
                print "Object %s created on surface %s" % (b.GetName(), table)
                created += 1
                break
        if b is None:
            print "No new object could be created on iteration %d" % obj_num
        
    return created
        
if __name__ == "__main__":

    usage_str =  "Usage: %s num_of_objects environment_save_file [base_size=0.1]" % sys.argv[0]
    
    if len(sys.argv) == 3:
        num_objects, filename = sys.argv[1:]
        dims =  (0.1, 0.1, 0.3)
    elif len(sys.argv) == 4:
        num_objects, filename, base_size = sys.argv[1:]
        dims =  (float(base_size), float(base_size), 0.3)
    else:
        print usage_str
        sys.exit(1)
        
    env = openravepy.Environment()
    env.Load("bare_bone.dae")
    
    num_objects = int(num_objects)
    num_created = generate_n_objects(env, num_objects, dims)
    print "I managed to create %d objects" % num_created
    env.Save(filename + ".dae")