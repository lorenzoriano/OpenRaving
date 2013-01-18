(define (problem dinnerTime1) 
(:domain robotics)
(:objects
	random_object11 random_object12 random_object21
	random_object22 random_object31   random_object32
	
	loc_random_object11 loc_random_object21 loc_random_object12
	loc_random_object22 loc_random_object31 loc_random_object32 

	gp_random_object11 gp_random_object21 gp_random_object12
	gp_random_object22 gp_random_object31 gp_random_object32  

	targettable sourcetable
	tray 
	trayLoc1 
	trayLoc2	
	
	blf_tray
	blf_targettable
	blf_sourcetable	
	blf_trayLoc1 
	;blf_trayLoc2	
	None
	
	table6
	
	blf_table6

	robotInitLoc
)

(:init

(Object random_object11)
(Object random_object21)
(Object random_object12)
(Object random_object22)
(Object random_object31)
(Object random_object32)
(Object tray)
(Object None)
(IsTray tray)

(Location loc_random_object11)
(Location loc_random_object21)
(Location loc_random_object12)
(Location loc_random_object22)
(Location loc_random_object31)
(Location loc_random_object32)
(Location trayLoc1)
(Location trayLoc2)
(Location table6)
(TempArea table6)

(Location gp_random_object11)
(Location gp_random_object21)
(Location gp_random_object12)
(Location gp_random_object22)
(Location gp_random_object31)
(Location gp_random_object32)

(Location targettable)
(Location sourcetable)
(Location blf_targettable)
(Location blf_sourcetable)
(Location robotInitLoc)
(Location blf_trayLoc1)
(Location blf_trayLoc2)
(Location blf_table6)
(Location door)

(At random_object11 loc_random_object11)
(At random_object21 loc_random_object21)
(At random_object12 loc_random_object12)
(At random_object22 loc_random_object22)
(At random_object31 loc_random_object31)
(At random_object32 loc_random_object32)
(At tray trayLoc1)
(RobotAt robotInitLoc)

(IsAccessPointFor blf_sourcetable sourcetable)
(IsAccessPointFor blf_targettable targettable)
(IsAccessPointFor blf_trayLoc1 trayLoc1)
(IsAccessPointFor blf_trayLoc2 trayLoc2)
(IsAccessPointFor blf_table6 table6)


(IsAccessPointForTray blf_tray tray)

(IsGP gp_random_object11 random_object11)
(IsGP gp_random_object21 random_object21)
(IsGP gp_random_object12 random_object12)
(IsGP gp_random_object22 random_object22)
(IsGP gp_random_object31 random_object31)
(IsGP gp_random_object32 random_object32)

(Topmost None tray)
)

(:goal (and (OnTray random_object22)(OnTray random_object12) (OnTray random_object32)(InGripper tray))
)
)

