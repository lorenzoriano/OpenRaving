(define (problem canworld1) (:domain robotics)
(:objects
object0 object1 object2 object3 object4 object5  - object

loc_object0 loc_object1 loc_object2 loc_object3 loc_object4 loc_object5  gp_object0 gp_object1 gp_object2 gp_object3 gp_object4 gp_object5 - location
table6  blf_table6 robotInitLoc - location
)
(:init
(At object0 loc_object0)
(At object1 loc_object1)
(At object2 loc_object2)
(At object3 loc_object3)
(At object4 loc_object4)
(At object5 loc_object5)
(RobotAt robotInitLoc)

(empty gripper)
(IsAccessPointFor blf_table6 table6)

(IsGP gp_object0 object0)
(IsGP gp_object1 object1)
(IsGP gp_object2 object2)
(IsGP gp_object3 object3)
(IsGP gp_object4 object4)
(IsGP gp_object5 object5)

)



(:goal (and (At object5 table6)))

)
