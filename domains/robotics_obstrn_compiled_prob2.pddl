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
(Object obj3)
(Object obj4)
(Object obj5)
(Object obj6)
(Object obj7)
(Object obj8)
(Object obj9)
(Object obj10)



(Location loc1)
(Location loc2)
(Location loc3)
(Location robotInitLoc)

(Location blf_loc1)

(At obj1 loc1)
(At obj2 loc2)
(At obj3 loc3)

(RobotAt robotInitLoc)


(IsAccessPointFor blf_loc1 loc1)

obstructs(p0,obj3,obj1)
obstructs(p1,obj,obj2)
obstructs(p2,obj,obj3)
obstructs(p2,obj2,obj3)
obstructs(p3,obj9,obj3)
obstructs(p4,obj0,obj3)
obstructs(p4,obj9,obj3)
obstructs(p5,obj19,obj4)
obstructs(p6,obj5,obj4)
obstructs(p7,obj6,obj7)
obstructs(p8,obj0,obj7)
obstructs(p9,obj17,obj21)
obstructs(p9,obj18,obj21)
obstructs(p10,obj20,obj21)
obstructs(p10,obj25,obj21)
obstructs(p11,obj18,obj21)
obstructs(p12,obj23,obj22)
obstructs(p12,obj19,obj22)
obstructs(p13,obj19,obj22)
obstructs(p14,obj24,obj25)
obstructs(p14,obj18,obj25)
obstructs(p15,obj18,obj25)
obstructs(p16,obj20,obj25)
obstructs(p17,obj24,obj25)
obstructs(p17,obj20,obj25)
obstructs(p18,obj18,obj26)
obstructs(p18,obj21,obj26)
obstructs(p19,obj20,obj26)
obstructs(p19,obj23,obj26)
obstructs(p20,obj23,obj26)
obstructs(p21,obj6,obj27)
obstructs(p21,obj30,obj27)
obstructs(p21,obj34,obj27)
obstructs(p22,obj32,obj27)
obstructs(p22,obj33,obj27)
obstructs(p23,obj33,obj27)
obstructs(p24,obj29,obj27)
obstructs(p25,obj6,obj34)
obstructs(p25,obj7,obj34)
obstructs(p26,obj30,obj34)
obstructs(p27,obj33,obj34)
obstructs(p27,obj27,obj34)
obstructs(p28,obj12,obj16)
obstructs(p29,obj,obj2)


)

(:goal (and (At obj1 loc1f) (At obj2 loc2f)))
)
