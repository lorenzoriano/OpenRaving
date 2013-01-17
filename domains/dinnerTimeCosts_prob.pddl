(define (problem dinnerTime1) 
(:domain robotics)
(:objects
	random_object random_object0 random_object1 random_object2
	random_object3 random_object4   
	
	loc_random_object loc_random_object0 loc_random_object1
	loc_random_object2 loc_random_object3 loc_random_object4 

	gp_random_object gp_random_object0 gp_random_object1
	gp_random_object2 gp_random_object3 gp_random_object4  

	targettable sourcetable
	tray1 
	trayLoc1 
	trayLoc2	
	
	blf_tray1
	blf_targettable
	blf_sourcetable	
	blf_trayLoc1 
	;blf_trayLoc2	
	None
	
	robotInitLoc
)

(:init

(Object random_object)
(Object random_object0)
(Object random_object1)
(Object random_object2)
(Object random_object3)
(Object random_object4)
(Object tray1)
(Object None)
(Tray tray1)

(Location loc_random_object)
(Location loc_random_object0)
(Location loc_random_object1)
(Location loc_random_object2)
(Location loc_random_object3)
(Location loc_random_object4)
(Location trayLoc1)
(Location trayLoc2)


(Location gp_random_object)
(Location gp_random_object0)
(Location gp_random_object1)
(Location gp_random_object2)
(Location gp_random_object3)
(Location gp_random_object4)

(Location targettable)
(Location sourcetable)
(Location blf_targettable)
(Location blf_sourcetable)
(Location robotInitLoc)
(Location blf_trayLoc1)
(Location blf_trayLoc2)
(Location door)

(At random_object loc_random_object)
(At random_object0 loc_random_object0)
(At random_object1 loc_random_object1)
(At random_object2 loc_random_object2)
(At random_object3 loc_random_object3)
(At random_object4 loc_random_object4)
(At tray1 trayLoc1)
(RobotAt robotInitLoc)

(IsAccessPointFor blf_sourcetable sourcetable)
(IsAccessPointFor blf_targettable targettable)
(IsAccessPointFor blf_trayLoc1 trayLoc1)
(IsAccessPointFor blf_trayLoc2 trayLoc2)


(IsAccessPointForTray blf_tray1 tray1)

(IsGP gp_random_object random_object)
(IsGP gp_random_object0 random_object0)
(IsGP gp_random_object1 random_object1)
(IsGP gp_random_object2 random_object2)
(IsGP gp_random_object3 random_object3)
(IsGP gp_random_object4 random_object4)

(Topmost None tray1)
)

(:goal (and (OnTray random_object1) (OnTray random_object2) 
       (At tray1 trayLoc2) ))
(:metric minimize (total-cost) )
)


