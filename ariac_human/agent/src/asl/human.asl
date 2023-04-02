/* Initial beliefs and rules */
// The location belief contains the location and the X,Y,Z coordinates
// Positions of the tables are with respect of the initial position of the screen when the simulation starts
// station1 = table bottom right, station2 = table top right, station4 = table top left, station3 = table bottom left
location(station1,  -5.0,  3.75, 0.0).   
location(station2, -10.8,  3.75, 0.0).   
location(station4, -10.8, -3.75, 0.0).  
location(station3,  -5.0, -3.75, 0.0). 

// Determine the first station to be visited
firstStation(station4).   

// Determine the next location to work (second parameter) based on the current one (first parameter)
next_location(station1,station2).
next_location(station2,station4).
next_location(station4,station3).
next_location(station3,station1).
previous_location(station1,station3).
previous_location(station2,station1).
previous_location(station4,station2).
previous_location(station3,station4).

/* Initial goals */
!standby.

/* Plans */
+!standby: true <- .wait("+human_start"); !start.
//+!standby: true <- !start.   // used for tests

+!start : firstStation(ST) & helpful <- .include("helpful.asl"); .print("Human agent started as HELPFUL"); !work(ST).
+!start : firstStation(ST) & indifferent <- .include("indifferent.asl"); .print("Human agent started as INDIFFERENT"); !work(ST).
+!start : firstStation(ST) & antagonistic <- .include("antagonistic.asl"); .print("Human agent started as ANTAGONISTIC"); !work(ST).

// Work pattern plans
+!work(Location) : location(Location, X, Y, Z) 
	<- 
		-working(_);
		+working(Location);
		.print("Now moving to: ", Location);
		move(X, Y, Z).


//+work_completed : working(Location) & next_location(Location,Next)
+work_completed(_) : working(Location) & previous_location(Location,Prev) & not counterClock
	<-
		.print("Clockwise - Work completed at ", Location);
		!work(Prev).


//+work_completed : working(Location) & next_location(Location,Next)
+work_completed(_) : working(Location) & next_location(Location,Next) & counterClock
	<-
		.print("Counterclock - Work completed at ", Location);
		!work(Next).


// Plan for when gantry is disabled
@disabled[atomic]
+gantry_disabled(_) : firstStation(ST) 
	<-
		.print("Gantry disabled: teleport requested.");
		.drop_all_desires; 
		teleport_safe; // teleport to the safe zone PENALIZING competitors
		.wait(10000); // waits for 10 seconds in the safe zone
		!!work(ST).  // restarts on the first station
		
// Plan for when AGV is too close
@danger[atomic]
+agv_danger(_) : firstStation(ST) 
	<-
		.print("AGV close: teleport requested.");
		.drop_all_desires; 
		teleport_agv; // teleport to the safe zone WITHOUT penalizing competitors
		.wait(5000); // waits for 5 seconds in the safe zone
		!!work(ST). // restarts on the first station
		
