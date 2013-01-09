(define (problem robotics1) (:domain robotics)
(:objects 
  obj1 obj2 obj3
  loc1 loc2 loc3 
  loc1f loc2f loc3f 
  robotInitLoc 
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

(At obj1 loc1)
(At obj2 loc2)
(At obj3 loc3)
(RobotAt robotInitLoc)


)
(:goal (and (At obj1 loc1f) (At obj2 loc2f) (At obj3 loc3f)))
)
