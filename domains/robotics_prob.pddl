(define (problem robotics1) (:domain robotics)
(:objects 
  obj1 loc1 loc2 robotInitLoc gp_obj1 gp_obj2 blf_loc1 blf_loc2)

(:init
(Object obj1)
(Location loc1)
(Location loc2)
(Location robotInitLoc)
(Location gp_obj1)
(Location blf_loc1)
(Location blf_loc2)

(At obj1 loc1)
(RobotAt robotInitLoc)

(IsGP gp_obj1 obj1)

(IsAccessPointFor blf_loc1 loc1)
(IsAccessPointFor blf_loc2 loc2)

)
(:goal (At obj1 loc2))
)
