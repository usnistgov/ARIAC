@detected[atomic]
+gantry_detected(_) : working(Location) & previous_location(Location,Prev) & counterClock
	<- 
		stop_movement; // stops moving and cancel any navigation goals
		.drop_all_desires; 
		.print("Detected the Gantry robot in close proximity, turning Clockwise"); 
		.wait(1500); 
		-counterClock; 
		!!work(Prev).
		
@detectedCounter[atomic]
+gantry_detected(_) : working(Location) & next_location(Location,Next) & not counterClock
	<- 
		stop_movement; // stops moving and cancel any navigation goals
		.drop_all_desires; 
		.print("Detected the Gantry robot in close proximity, turning Counterclock"); 
		.wait(1500); 
		+counterClock;
		!!work(Next).
		
