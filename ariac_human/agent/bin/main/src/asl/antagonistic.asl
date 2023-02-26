+gantry_detected(_) : working(Location) & next_location(Location,Next) 
	<-
		stop_movement; // stops moving and cancel any navigation goals
		.print("Detected the Gantry robot in close proximity and moving towards me, I want to mess with it.");
		.drop_all_desires;
		.wait(1500);
		move_to_gantry;
		.wait("+work_completed(_)");  
		!!work(Next).
