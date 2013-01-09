(define (domain robotics)
  (:requirements :strips :equality)
  (:predicates
	   (Location ?loc)
	   (Object ?obj)
	   (At ?obj ?loc)
           (RobotAt ?loc)
	   (InGripper ?obj)
  )

  (:action moveTo
     :parameters(?l1 ?l2)
     :precondition (and (Location ?l1) (Location ?l2) (RobotAt ?l1))
     :effect (and (not (RobotAt ?l1)) (RobotAt ?l2))
  )

 (:action putDown
  :parameters(?o1 ?l1)
  :precondition (and 
  		(Object ?o1) (Location ?l1) (Location ?l2)
  		(InGripper ?o1) 
		(RobotAt AccessPointFor(?l1)))
		;;robot at precon for putdown trajectory with dest l1
  :effect (and (not (InGripper ?o1)) (At ?o1 ?l1) )
 )

 (:action grasp
     :parameters(?o1 ?l1)
     :precondition(and 
         (Object ?o1) (Location ?l1) (At  ?o1 ?l1) 
         (RobotAt GP(?o1)) 
	 ;; robot at precon for grasping trajectory existence with
     dest loc of o1.
	 ;;one of the advantages of unint fns: need only one
     	 ;;robotAt. required posn varies acc to object.
     	 (forall (?o) (imply (Object ?o) (not (InGripper ?o))) )
	 (forall (?o) (imply (Object ?o) (not (Obstructs ?o ?o1)) ))
	 ;; obstructs computed as: Intersects(A, RgnForGrasping(o1))
	 ;; Can be done as part of preprocessing
     )
     :effect (and (InGripper ?o1) (not (At ?o1 ?l1))
     	     (forall (?o) (imply (Object ?o) (not (Obstructs ?o1 ?o))))
     )
 )
)
