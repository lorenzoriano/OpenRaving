#! /usr/bin/python

import openravepy
import sys

class PDDLWorld(object):
    """A convenient representation for a PDDL description of a problem domain.
    """
    
    def __init__(self):
        self.objects = []
        self.obj_locations = []
        self.gp_locations = []
        self.robot = None
        self.table = None
    
    def load_environment(self, env):
        """Loads an environment and fills all the structures.
        
        It assumes that the objects have names random_object[xxx]        
        """
        
        assert isinstance(env, openravepy.Environment)

        self.objects = [b.GetName()
                        for b in env.GetBodies()
                        if b.GetName().startswith("random_")]
        
        self.obj_locations = ["loc_" + o for o in self.objects]
        self.gp_locations = ["gp_" + o for o in self.objects]
        
        self.table = "table6"  #TODO: this is fixed!!
        
    def write_header(self):
        """Returns the pddl header, right not fixed        
        """
        return "(define (problem robotics1) (:domain robotics)" + "\n"
    
    def write_objects(self):
        all_str = "(:objects" + "\n"
        
        #objects
        all_str += " ".join(self.objects) + "\n\n"
        
        #locations
        all_str += " ".join(self.obj_locations) + "\n"
        all_str += " ".join(self.gp_locations) + "\n"
        all_str += "\n"
        
        #table
        all_str += self.table + "\n"
        all_str += "blf_" + self.table + "\n"
        all_str += "\n"
        
        #robot
        all_str += "robotInitLoc\n"
        
        all_str += ")\n"
        return all_str
    
    def write_init(self):
        all_str = "(:init" + "\n\n"
        
        all_str += "\n".join("(Object %s)" % o for o in self.objects) + "\n"
        
        #locations
        all_str += "\n".join("(Location %s)" % l for l in self.obj_locations) + "\n"
        all_str += "\n".join("(Location %s)" % l for l in self.gp_locations) + "\n"
        all_str += "\n"
        all_str += "(Location " + self.table + ")\n"
        all_str += "(Location " + "blf_" + self.table + ")\n"
        all_str += "(Location " + "robotInitLoc" + ")\n"
        all_str += "\n"
        
        #At predicate
        all_str += "\n".join("(At %s %s)" % (o, l)
                             for (o, l) in zip(self.objects, self.obj_locations)) + "\n"
        all_str +=  "(RobotAt robotInitLoc)\n"
        all_str += "\n"
        
        all_str += "(IsAccessPointFor " + "blf_"+self.table + " " +  self.table+")" + "\n\n"
        
        #reachable from
        all_str += "\n".join("(IsGP %s %s)" % (l, o)
                             for (l, o) in zip(self.gp_locations, self.objects)) + "\n"
        
        all_str += "\n)\n\n"
        return all_str
    
    def write_pddl(self):
        all_str = ( self.write_header() +
                    self.write_objects() +
                    self.write_init()
        )
        return all_str
    
    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Usage: %s name_of_worldfile"
        sys.exit(1)
    
    worldfile = sys.argv[1]
    env = openravepy.Environment()
    env.Load(worldfile)
    
    pddl = PDDLWorld()
    pddl.load_environment(env)
    res = pddl.write_pddl()
    print res
    
    
        