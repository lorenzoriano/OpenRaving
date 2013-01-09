(define (problem robotics1) (:domain robotics)
(:objects 
  obj1 obj2 obj3
  loc1 loc2 loc3 
  loc1f loc2f loc3f 
  robotInitLoc 
  gp_obj1 gp_obj2 gp_obj3
  blf_loc1 blf_loc2 blf_loc3 
  blf_loc1f blf_loc2f blf_loc3f 
  )

(:init
(Object obj1)
(Object obj2)
(Object obj3)
(Location loc1)
(Location loc2)
(Location loc3)
(Location loc1f)
(Location loc2f)
(Location loc3f)
(Location robotInitLoc)
(Location gp_obj1)
(Location gp_obj2)
(Location gp_obj3)
(Location blf_loc1)
(Location blf_loc2)
(Location blf_loc3)
(Location blf_loc1f)
(Location blf_loc2f)
(Location blf_loc3f)

(At obj1 loc1)
(At obj2 loc2)
(At obj3 loc3)
(RobotAt robotInitLoc)

(IsGP gp_obj1 obj1)
(IsGP gp_obj2 obj2)
(IsGP gp_obj3 obj3)

(IsAccessPointFor blf_loc1 loc1)
(IsAccessPointFor blf_loc2 loc2)
(IsAccessPointFor blf_loc3 loc3)
(IsAccessPointFor blf_loc1f loc1f)
(IsAccessPointFor blf_loc2f loc2f)
(IsAccessPointFor blf_loc3f loc3f)

(Obstructs obj3 obj2)
(Obstructs obj2 obj1)

)

(:goal (and (At obj1 loc1f) (At obj2 loc2f) (At obj3 loc3f)))
)
