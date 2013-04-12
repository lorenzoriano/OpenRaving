(define (domain robotics)
  (:requirements :strips :equality)
  (:constants gripper)
  (:predicates
	   (P1 ?obj)
	   (P2 ?obj)
	   (Location ?loc)
	   (Object ?obj)
	   (At ?obj ?loc)
           (RobotAt ?loc)
	   (InGripper ?obj)
	   (IsAccessPointFor ?loc ?targetLocation)
	   (Obstructs ?loc ?obj ?obj)
	   (IsGP ?loc ?obj)
	   (empty ?gripper)

  )


  (:action moveTo
     :parameters(?l1 ?l2)
     :precondition (and (Location ?l1) (Location ?l2) (RobotAt ?l1))
     :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
  )

 (:action putDown
  :parameters(?o1 ?l1 ?l2)
  :precondition (and (Object ?o1) (Location ?l1) (Location ?l2) 
  		     (InGripper ?o1) (RobotAt
  ?l2) (IsAccessPointFor ?l2 ?l1))
  :effect (and (not (InGripper ?o1)) (At ?o1 ?l1) (empty gripper))
 )

 (:action grasp
     :parameters(?o1 ?l1 ?l2)
     :precondition(and (Object ?o1) (Location ?l1) (Location ?l2) (At  ?o1 ?l1) 
     		       (RobotAt ?l2) (IsGP ?l2 ?o1)
		       (empty gripper)
		       (forall (?o) (imply (Object ?o) 
		       	       	    (not (Obstructs ?l2 ?o ?o1)) ))
		       )
     :effect (and (InGripper ?o1) (not (At ?o1 ?l1))
     	     (forall (?o) (forall (?l) (not (Obstructs ?l ?o1 ?o))))
	     (not (empty gripper)))
 )
)
