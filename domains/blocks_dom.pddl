(define (domain blocks)
  (:requirements :strips :equality)
  (:predicates (on ?obj1 ?obj2)
	   (Block ?obj)
	   (Red ?obj)
           (Blue ?obj)
	   (topmost ?obj)
	   (onTable ?obj)
  )
  (:action move
     :parameters(?b1 ?b2 ?b3)
     :precondition(and (topmost ?b1) (topmost ?b3) (on ?b1 ?b2))
     :effect (and (not (topmost ?b3)) (topmost ?b2) (not (on ?b1 ?b2)) (on ?b1 ?b3) )
  )

 (:action moveFromTable
  :parameters(?b1 ?b2)
  :precondition (and (topmost ?b1) (topmost ?b2) (onTable ?b1) (not( = ?b1 ?b2)))
  :effect (and (not (topmost ?b2)) (not (onTable ?b1)) (on ?b1 ?b2) )
 )

 (:action moveToTable
     :parameters(?b1 ?b2)
     :precondition(and (topmost ?b1) (on ?b1 ?b2))
     :effect (and (topmost ?b2) (not (on ?b1 ?b2)) (onTable ?b1))
 )
)
