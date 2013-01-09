(define (problem robotics1) (:domain robotics)
(:objects 
  obj obj0 obj1 obj2

  loc loc0 loc1 loc2

  table
  robotInitLoc 
  blf_table
  p0 p1 p2 p3 

  gp_obj gp_obj0 gp_obj5 
  )

(:init

(Object obj)
(Object obj0)
(Object obj1)
(Object obj2)
(Location loc)
(Location loc0)
(Location loc1)
(Location loc2)

(Location table)
(Location robotInitLoc)
(Location blf_table)
(Location p0)
(Location p1)
(Location p2)
(Location p3)
(Location  gp_obj)
(Location  gp_obj0)

(At obj loc)
(At obj0 loc0)
(At obj1 loc1)
(At obj2 loc2)

(RobotAt robotInitLoc)
(IsAccessPointFor blf_table table)

(Obstructs p1 obj obj0)
(Obstructs p1 obj1 obj)

(ReachableFrom p1 obj)
(ReachableFrom p1 obj1)
(ReachableFrom p1 obj0)

(P1 obj0)
(P2 obj0)

)

(:goal (and (At obj0 table) (not (P2 obj0))))
)
