import jason.asSyntax.*;
import jason.environment.*;
import java.util.logging.*;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.geometry_msgs.Point;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.msgs.std_msgs.Bool;
import ros.msgs.ariac_msgs.AGVStatus;  
import ros.msgs.ariac_msgs.HumanState; 
import ros.msgs.ariac_msgs.Robots;
import ros.tools.MessageUnpacker;
import com.fasterxml.jackson.databind.JsonNode;

public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("ariac_env."+RosEnv.class.getName());
    
	int cont = 0;
	int ctrDt = 0;
	int ctrUnsf = 0;

	double hpX = -15.0;
	double hpY = -10.0;

	double gpX = 0.0;
	double gpY = 0.0;
	double gpZ = 0.0;

	double lastHumanState_MsgT = System.currentTimeMillis();
	double lastUnsafeD_MsgT    = System.currentTimeMillis();
	double previousDistance = 50.0;
	double minAgvDist = 0.85;
	boolean isAproximating = false;
	
	boolean simulationStarted = false; 	
    
    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");	
		
		/* Subscribe for calculating the distance between the Gantry and the human */
		bridge.subscribe(SubscriptionRequestMsg.generate("/ariac_human/state") 
				.setType("ariac_msgs/msg/HumanState") 
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<HumanState> unpacker = new MessageUnpacker<HumanState>(HumanState.class);
					HumanState msg = unpacker.unpackRosMessage(data);

					gpX = msg.robot_position.x;   hpX = msg.human_position.x;
					gpY = msg.robot_position.y;   hpY = msg.human_position.y;
					gpZ = msg.robot_position.z;
					
					double distance_robotHuman = calculateDistanceRH(msg);
					double safe_distanceRH     = calSafeDistanceRH(msg);

					//Check if they are approximating (getting close from each other)
					if(distance_robotHuman < previousDistance)
						isAproximating = true;
					else 
						isAproximating = false;
					previousDistance = distance_robotHuman;

					long timeNow = System.currentTimeMillis();   //Time check

					if ((distance_robotHuman < safe_distanceRH*1.75) && 
						(timeNow-lastHumanState_MsgT > 20000) && (isAproximating == true)){
						//clearPercepts("human");
						lastHumanState_MsgT=timeNow;
						logger.info("SAFE["+ safe_distanceRH +"] I see the Gantry robot in " + distance_robotHuman +" meters: gantry_detected");
						Literal gDetectedLit = new LiteralImpl("gantry_detected"); 
						gDetectedLit.addTerm(new NumberTermImpl(ctrDt++)); 
						if(simulationStarted==true)
							addPercept("human",gDetectedLit); 
					}
				}
			} 
		); // END bridge.subscribe(..."/ariac_human/state") 
	
		/* Subscriber for getting the information that the Gantry has been disabled */
		bridge.subscribe(SubscriptionRequestMsg.generate("/ariac_human/unsafe_distance") 
				.setType("std_msgs/Bool")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					long timeNow = System.currentTimeMillis();   //Time check

					MessageUnpacker<PrimitiveMsg<Boolean>> unpacker = new MessageUnpacker<PrimitiveMsg<Boolean>>(PrimitiveMsg.class);
					PrimitiveMsg<Boolean> msg = unpacker.unpackRosMessage(data);
					if((simulationStarted==true) && (timeNow-lastUnsafeD_MsgT > 10000)){ 
						//clearPercepts("human");
						lastUnsafeD_MsgT = timeNow;

						if(msg.data){ 
							logger.info("Gantry has been disabled!");
							Literal gUnsafeLit = new LiteralImpl("gantry_disabled"); 
							gUnsafeLit.addTerm(new NumberTermImpl(ctrUnsf++)); 
							addPercept("human",gUnsafeLit); 
						}
						else{ 
							logger.info("UAV danger!");
							Literal gUnsafeLit = new LiteralImpl("agv_danger"); 
							gUnsafeLit.addTerm(new NumberTermImpl(ctrUnsf++)); 
							addPercept("human",gUnsafeLit); 
						}
					}
					
				}
			}
		); // END bridge.subscribe(..."/ariac_human/unsafe_distance") 
	
		/* Subscriber for move_base result */		
		bridge.subscribe(SubscriptionRequestMsg.generate("/ariac_human/position_reached")
				.setType("std_msgs/Bool")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<Boolean>> unpacker = new MessageUnpacker<PrimitiveMsg<Boolean>>(PrimitiveMsg.class);
					PrimitiveMsg<Boolean> msg = unpacker.unpackRosMessage(data);
					//clearPercepts("human");
					logger.info("Human reached waypoint	!");
					Literal movebase_result = new LiteralImpl("work_completed"); 
					movebase_result.addTerm(new NumberTermImpl(cont++)); 
					logger.info("cont: "+cont);
					if(simulationStarted==true)
							addPercept("human", movebase_result);
				}
			}
	    ); // END bridge.subscribe(..."/ariac_human/position_reached")
		
		/* Subscriber for getting the START message */
		bridge.subscribe(SubscriptionRequestMsg.generate("/ariac/start_human") 
				.setType("std_msgs/Bool")
				.setThrottleRate(1)
				.setQueueLength(1),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<Boolean>> unpacker = new MessageUnpacker<PrimitiveMsg<Boolean>>(PrimitiveMsg.class);
					PrimitiveMsg<Boolean> msg = unpacker.unpackRosMessage(data);
					//logger.info("Simulation will start!");
					if (msg.data){
						//clearPercepts("human");
						logger.info("Simulation started!");
						addPercept("human",Literal.parseLiteral("human_start"));
						simulationStarted = true; 
					}					
				}
			}
		); // END bridge.subscribe(..."/ariac/start_human")
	} // END init()

	// Calculate distance from AGV and Human
	public double calculateDistanceAgvH(int agvId, AGVStatus msg){
		double x1 = hpX;          //human_position.x;
		double x2 = -2.7 - msg.position; //robot_position.x;
		double y1 = hpY;          //human_position.y;
		double y2 = 0.0;          //robot_position.y is STATIC
		if(agvId==1)      y2 = 4.8;
		else if(agvId==2) y2 = 1.2;
		else if(agvId==3) y2 =-1.2;
		else if(agvId==4) y2 =-4.8;

		double dis=Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
		return dis;
	}

	// Calculate distance from Robot (Gantry) and Human
	public double calculateDistanceRH(HumanState msg){
		double x1 = msg.human_position.x;
		double x2 = msg.robot_position.x;
		double y1 = msg.human_position.y;
		double y2 = msg.robot_position.y;

		double dis=Math.sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
		return dis;
	}

	// SAFE-Distance calculation 
	public double calSafeDistanceRH(HumanState msg){
        double t_1 = 1.0;
        double t_2 = 1.5;
        double beta = 0.0;
        double delta = 1.2;

        double speed_h = Math.sqrt(Math.pow(msg.human_velocity.x, 2) + Math.pow(msg.human_velocity.y, 2));
        double speed_r = Math.sqrt(Math.pow(msg.robot_velocity.x, 2) + Math.pow(msg.robot_velocity.y, 2));

        double safe_distance = (speed_h * (t_1 + t_2)) + (speed_r * (t_1)) + beta + delta; 
		// Turn-around to avoid simulation problems:
		if(safe_distance > 5.0)
			return 5.0;
		else
			return safe_distance;
	}

	@Override
    public boolean executeAction(String agName, Structure act) {
		if (act.getFunctor().equals("move")) {
			NumberTerm lx = (NumberTerm) act.getTerm(0);
			NumberTerm ly = (NumberTerm) act.getTerm(1);
			NumberTerm lz = (NumberTerm) act.getTerm(2);
			try{
				move(lx.solve(),ly.solve(),lz.solve());
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
		else if (act.getFunctor().equals("stop_movement")) { 
			stop_moving();
		}
		else if (act.getFunctor().equals("teleport_safe")) { 
			teleport(true);
		}
		else if (act.getFunctor().equals("teleport_agv")) { 
			teleport(false);  
		}
		else if (act.getFunctor().equals("move_to_gantry")) { 
			move_to_gantry(); 
		}
		else {
			logger.info("PROBLEM: requested: "+act.getFunctor()+", but not implemented!");
		}
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }
    
	// MOVE request; published topic is read by movebaser_node.py
	public void move(double x, double y, double z) {
		Publisher move_base = new Publisher("/ariac_human/goal_position", "geometry_msgs/Point", bridge);	
		move_base.publish(new Point(x,y,z));
	}
	
	// STOP request; published topic is read by movebaser_node.py
	public void stop_moving() {
		logger.info("Human stop requested!");
		Publisher move_base = new Publisher("/ariac_human/stop", "std_msgs/Bool", bridge);		
		move_base.publish(new Bool(true)); 
	}
	
	// Published topic is read by movebaser_node.py; it than calls a service in the Gazebo plugin TeleportHuman
	public void teleport(boolean penalty) { 
		Publisher teleport_h = new Publisher("/ariac_human/go_home", "std_msgs/Bool", bridge);
		if(penalty)	
			teleport_h.publish(new Bool(true));	
		else
			teleport_h.publish(new Bool(false));	

		// Reset "smart" orientation variables
		lastHumanState_MsgT = System.currentTimeMillis();
		lastUnsafeD_MsgT = System.currentTimeMillis();
		previousDistance = 50.0;
		isAproximating = false;
	}

	// First STOP than MOVE; published topics are read by movebaser_node.py 
	public void move_to_gantry() {
		stop_moving();
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		move(gpX, gpY, gpZ);
	}

 	// public void hello_ros() {		
	// 	Publisher pub = new Publisher("/java_to_ros", "std_msgs/String", bridge);
	// 	for(int i = 0; i < 100; i++) {
	// 		pub.publish(new PrimitiveMsg<String>("hello from Jason " + i));
	// 		try {
	// 			Thread.sleep(1500);
	// 		} catch (InterruptedException e) {
	// 			e.printStackTrace();
	// 		}
	// 	}
	// }
	
   /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
}
