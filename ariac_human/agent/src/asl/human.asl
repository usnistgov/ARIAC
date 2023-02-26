/* Initial beliefs and rules */
// The location belief contains the location and the X,Y,Z coordinates
// Positions of the tables are with respect of the initial position of the screen when the simulation starts
// station1 = table bottom right, station2 = table top right, station4 = table top left, station3 = table bottom left
location(station1,  -5.75,  3.0, 0.0).   //   -5.75,  3.0, 0.0
location(station2, -10.85,  3.0, 0.0).  //  -11.85, -3.0, 0.0 
location(station4, -10.85, -3.0, 0.0). //  -11.85, -3.0, 0.0 Y was -4.5
location(station3,  -5.75, -3.0, 0.0).

// safe zone, not being used by the agent at the moment
//location(safe, -15.29, -10.04, 0.0).

// Beliefs that determine where is the next location to work (second parameter) based on the current one (first parameter)
next_location(station1,station2).
next_location(station2,station4).
next_location(station4,station3).
next_location(station3,station1).

previous_location(station1,station3).
previous_location(station2,station1).
previous_location(station4,station2).
previous_location(station3,station4).

//LB: initial belief seems needed
gantry_position(0.0, 0.0, 0.0).

//////////////// Rules
//g_position(X, Y, Z) :- gantry_position(X,Y,Z)[source(S)] & (S == self | S == percept).

/* Initial goals */
!standby.

/* Plans */
+!standby: true <- .wait("+human_start"); !start.
//+!standby: true <- !start.

+!start : indifferent <- .include("indifferent.asl"); +counterClock; .print("Human agent started as INDIFFERENT"); !work(station3).
+!start : helpful <- .include("helpful.asl"); +counterClock; .print("Human agent started as HELPFUL"); !work(station3).
+!start : antagonistic <- .include("antagonistic.asl"); +counterClock; .print("Human agent started as ANTAGONISTIC"); !work(station3).

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
//		-movingToGantry;
		!work(Next).


// Plan for when gantry is disabled
@disabled[atomic]
+gantry_disabled(_) : working(Location)
	<-
		//stop_movement; // not needed, teleport stops the robot
		.print("Gantry disabled: teleport requested.");
		.drop_all_desires; 
		.drop_all_intentions;
		teleport_safe; // ask to be teleported to the safe zone
		.wait(8000); // waits for 8 seconds in the safe zone
		!!work(station3).  //!!work(Location). // resumes work pattern
		
