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


	table1
	blf_table1

	table2
	blf_table2

	table3
	blf_table3

	table4
	blf_table4
	
	table6
	blf_table6

	robotInitLoc
)

(:init
(= (total-cost) 0)

(empty gripper)
(Object random_object11)
(Object random_object21)
(Object random_object12)
(Object random_object22)
(Object random_object31)
(Object random_object32)
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
(Location table1)
(Location table2)
(Location table3)
(Location table4)
(Location table1)
(TempArea table6)
(TempArea table1)
(TempArea table2)
(TempArea table3)
(TempArea table4)
(TrayLocation trayLoc1)
(TrayLocation trayLoc2)

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
(Location blf_table1)
(Location blf_table2)
(Location blf_table3)
(Location blf_table4)



(InRoom1 loc_random_object11)
(InRoom1 loc_random_object21)
(InRoom1 loc_random_object12)
(InRoom1 loc_random_object22)
(InRoom1 loc_random_object31)
(InRoom1 loc_random_object32)
(InRoom1 gp_random_object11)
(InRoom1 gp_random_object21)
(InRoom1 gp_random_object12)
(InRoom1 gp_random_object22)
(InRoom1 gp_random_object31)
(InRoom1 gp_random_object32)

(InRoom1 trayLoc1)
(InRoom1 table6)
(InRoom1 robotInitLoc)
(InRoom1 blf_trayLoc1)
(InRoom1 blf_table6)

(InRoom2 table1)
(InRoom2 blf_table1)
(InRoom2 table2)
(InRoom2 blf_table2)
(InRoom2 table3)
(InRoom2 blf_table3)
(InRoom2 table4)
(InRoom2 blf_table4)
(InRoom2 trayLoc2)
(InRoom2 blf_trayLoc2)

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
(IsAccessPointFor blf_table1 table1)
(IsAccessPointFor blf_table2 table2)
(IsAccessPointFor blf_table3 table3)
(IsAccessPointFor blf_table4 table4)


(IsAccessPointForTray blf_tray tray)

(IsGP gp_random_object11 random_object11)
(IsGP gp_random_object21 random_object21)
(IsGP gp_random_object12 random_object12)
(IsGP gp_random_object22 random_object22)
(IsGP gp_random_object31 random_object31)
(IsGP gp_random_object32 random_object32)

(Smaller random_object11 random_object12)
(Smaller random_object11 random_object21)
(Smaller random_object11 random_object22)
(Smaller random_object11 random_object31)
(Smaller random_object11 random_object32)

(Smaller random_object12 random_object11)
(Smaller random_object12 random_object21)
(Smaller random_object12 random_object22)
(Smaller random_object12 random_object31)
(Smaller random_object12 random_object32)

(Smaller random_object21 random_object12)
(Smaller random_object21 random_object11)
(Smaller random_object21 random_object22)
(Smaller random_object21 random_object31)
(Smaller random_object21 random_object32)

(Smaller random_object22 random_object12)
(Smaller random_object22 random_object21)
(Smaller random_object22 random_object11)
(Smaller random_object22 random_object31)
(Smaller random_object22 random_object32)

(Smaller random_object31 random_object12)
(Smaller random_object31 random_object21)
(Smaller random_object31 random_object22)
(Smaller random_object31 random_object11)
(Smaller random_object31 random_object32)

(Smaller random_object32 random_object12)
(Smaller random_object32 random_object21)
(Smaller random_object32 random_object22)
(Smaller random_object32 random_object31)
(Smaller random_object32 random_object11)

(Smaller random_object11 none)
(Smaller random_object21 none)
(Smaller  random_object31 none)
(Smaller  random_object12 none)
(Smaller  random_object22 none)
(Smaller  random_object32 none)

(Topmost None tray)
)

(:goal (and 
      	    (At random_object21 table4)
      	    (At random_object11 table4) 
            (At random_object31 table4)
	    )
)

  
(:metric minimize (total-cost))
)

