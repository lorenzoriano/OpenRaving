(define (domain robotics)
  (:requirements :strips :equality :typing)
  (:constants gripper)
  (:types object location)
  (:predicates
	   (At ?obj - object ?loc - location)
           (RobotAt ?loc - location)
	   (InGripper ?obj - object)
	   (IsAccessPointFor ?loc - location ?targetLocation - location)
	   (Obstructs ?loc - location ?obj - object ?obj - object)
	   (IsGP ?loc - location ?obj - object)
	   (empty ?gripper)

  )


  (:action moveTo
     :parameters(?l1 - location ?l2 - location)
     :precondition (and  (RobotAt ?l1))
     :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
  )

 (:action putDown
  :parameters(?o1 - object ?l1 - location ?l2 - location )
  :precondition (and 
  		     (InGripper ?o1) (RobotAt
  ?l2) (IsAccessPointFor ?l2 ?l1))
  :effect (and (not (InGripper ?o1)) (At ?o1 ?l1) (empty gripper))
 )

 (:action grasp
     :parameters(?o1 - object ?l1  - location ?l2 - location )
     :precondition(and (At  ?o1 ?l1) 
     		       (RobotAt ?l2) (IsAccessPointFor ?l2 ?l1)
		       (empty gripper)
		       (forall (?o - object)
		       	       	    (not (Obstructs ?l2 ?o ?o1)) )
		       )
     :effect (and (InGripper ?o1) (not (At ?o1 ?l1))
     	     (forall (?o - object) (forall (?l - location) (not (Obstructs ?l ?o1 ?o))))
	     (not (empty gripper)))
 )
)
