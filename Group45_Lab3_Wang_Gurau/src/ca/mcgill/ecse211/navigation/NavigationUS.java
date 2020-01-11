package ca.mcgill.ecse211.navigation;


import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * This class is our main navigation class, it will follow the path chosen 
 * and avoid the obstacles it detects
 * @author Antoine Wang
 * @author Tudor Gurau
 *
 */
public class NavigationUS implements Runnable {
	
	//Declare two motor fields for wheels
	public static EV3LargeRegulatedMotor rightMotor;
	public static EV3LargeRegulatedMotor leftMotor;

	//Declare another motor for sensor to rotate
	public static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));
	
	// Establish the sensor connection and sample
	private static final Port usPort = LocalEV3.get().getPort("S2");
	private static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	private static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	private static float[] usData = new float[usDistance.sampleSize()];
	
	//Odometer and its variable 
	//used in travelTo() method as posit
	private static Odometer odo;
	private static double currentX;
	private static double currentY;
	private static double currentT;
	private static double DTheta;
	
	//holds whether or not the robot is traveling
	private static boolean travellingStatus = false;
	
	private static boolean sensorStatus = true;
	
	
	//variables used in wallFallowing calculation and left/right predict
	private static double cur_X;
	private static double cur_Y;
	private static double cur_Theta;

	// Speeds for the motors
	private static final int FWDSPEED = 200;
	private static final int ROTATE_SPEED = 75;
	private static final int SENSOR_SPEED = 50;
	
	// initializing the 4 possible maps
	private static final int[][] Map1 = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final int[][] Map2 = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final int[][] Map3 = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
	private static final int[][] Map4 = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
	//private static final int[][] MapTest = new int[][] {{ 2, 1 }, { 1, 1 }, { 1, 2 }, { 2, 0 }};


	// initializing variable used in our methods
	private static SweepUS scan = new SweepUS(sensorMotor);
	private static int distance;
	private static final int safeDistance= 9;
	public static double Tile_Size;
	private static final int AVOID_DIST = 60; //maximum distance dodges
	private static final int SENSOR_TURN = 80;  // the degree that the sensor turns
	private static final int DELTA_SPEED = 40; // difference in speed in the bangbang controller during wall following
	private static double theta;
	//2 variable used in filter
	private static int filterControl;
	private static final int FILTER_OUT = 5;
	
	/**
	 * Constructor of Navigation class. Initializing left and right motor. Define tile size
	 * @param Tile_Size
	 * @param leftMotor1
	 * @param rightMotor1
	 */
	public NavigationUS(double Tile_Size, EV3LargeRegulatedMotor leftMotor1, EV3LargeRegulatedMotor rightMotor1) {
		NavigationUS.Tile_Size = Tile_Size;
		rightMotor = rightMotor1;
		leftMotor = leftMotor1;
		rightMotor.stop();
		leftMotor.stop();
		

	}
	/**
	 * The run method initiate ev3 to run in the pre_set route
	 * @return void
	 */
	public void run() {
		(new Thread(scan)).start();
		int[][] waypoints = Map3;
		for (int i = 0; i < Map3.length; i++) {   // this loop tries every points one by one

			try {
				TravelTo(Tile_Size * waypoints[i][0], Tile_Size * waypoints[i][1]);
				
			} catch (OdometerExceptions e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}
	/**
	 * This method make the robot go to the position (x,y)
	 * (x,y) is the coordination of input point on the grid board
	 * @param x
	 * @param y
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 * @return void
	 */
	public static void TravelTo(double x, double y) throws OdometerExceptions, InterruptedException {
		theta = 0.0;
		// getting our current position
		odo = Lab3Navigation.odometer;
		currentX = odo.getXYT()[0];
		currentY = odo.getXYT()[1];
		
		while (!withInError(currentX, currentY, x, y)) {
			
			travellingStatus = true;
			//updating our x and y
			currentX = odo.getXYT()[0];
			currentY = odo.getXYT()[1];
			
			// calculating the angle that we should be facing to the next point
			if (currentX == x) {
				if (currentY > y) {
					theta = Math.PI;
				} else if (currentY < y) {
					theta = 0;
				}

			} else if (currentY == y) {
				if (currentX > x) {
					theta = -Math.PI/2;
				} else if (currentX < x) {
					theta = Math.PI/2;
				}
			} else {
				theta = Math.atan((currentX - x) / (currentY - y));
				if (currentY > y) {             // Current Y is greater than target Y
					theta += Math.PI;           // So add 180 degree to make it turn to downward
				}
			}
			// turn to the angle calculated
			turnTo(Math.toDegrees(theta));
			
			// move forward by the required distance
			leftMotor.setSpeed(FWDSPEED);
			rightMotor.setSpeed(FWDSPEED);
			
			// calculating the Euclidian distance and convert it into degree and turn
			double dist = Math.sqrt(Math.pow((currentX - x), 2) + Math.pow((currentY - y), 2));
			leftMotor.rotate(convertDistance(Lab3Navigation.WHEEL_RAD, dist), true);
			rightMotor.rotate(convertDistance(Lab3Navigation.WHEEL_RAD, dist), true);
			
			// keep checking for obstacle and avoid them in case it found one
			while (!withInError(currentX, currentY, x, y)) {
				// update our current X and Y
				currentX = odo.getXYT()[0];
				currentY = odo.getXYT()[1];
				// getting data from the sensor
				usSensor.fetchSample(usData, 0); 
				distance = (int) (usData[0] * 100.0); 
				
				
				// rudimentary filter - toss out invalid samples corresponding to null
			    // signal.
			    if (distance > 255 && filterControl < FILTER_OUT) {
			      // bad value, do not set the distance var, however do increment the
			      // filter value                                                            
			      filterControl++;
			    } else if (distance > 255) {
			      // We have repeated large values, so there must actually be nothing
			      // there: leave the distance alone
			    } else {
			      // distance went below 255: reset filter and leave
			      // distance alone.
			      filterControl = 0;
			    }
			 // meet obstacle, determine which way it is going to dodge and start avoid
				if (distance < safeDistance) {
					scan.stopSensor();
					scan.setPos();
					if (predictPath() == 1){  //making decisions to turn left or right based on current position
						RightAvoid();
					}
					else if (predictPath() == 0){
						leftAvoid();
					}	
					scan.startSensor();
				break;
			}
				try {
					Thread.sleep(50);
				} catch (Exception e) {
					}
			}
			
	}
		// arrived to its destination
		travellingStatus = false;
		

	}
	/**
	 * This method calculated the minimal angle needed to turn to the destination
	 * and makes the robot turn
	 * @param theta angle heading to the destination point
	 */
	public static void turnTo(double theta) {
		currentT = odo.getXYT()[2];
		// calculating the turn that should be done
		DTheta = theta - currentT;
		//making sure the angle is between 0 and 360
		DTheta = (DTheta + 360) % 360;
		// making sure to turn by the minimal angle
		if (Math.abs(DTheta - 360) < DTheta) {
			DTheta -= 360;
		}
		// make the robot turn by DTheta
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Lab3Navigation.WHEEL_RAD, Lab3Navigation.TRACK, DTheta), true);
		rightMotor.rotate(-convertAngle(Lab3Navigation.WHEEL_RAD, Lab3Navigation.TRACK, DTheta), false);

	}
	/**
	 * This method returns whether or not the robot is traveling
	 * @return	status 
	 */
	public static boolean isNavigating() {
		return travellingStatus;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * This method return whether or not the robot reached to desired position
	 * if the Euclidian distance is less than 0.5cm then assume that the robot arrives at the spot
	 * @param Cx :currentX
	 * @param Cy : currentY
	 * @param x	:target x
	 * @param y	: target y
	 * @return boolean 
	 */
	private static boolean withInError(double Cx, double Cy, double x, double y) {
		double error = Math.sqrt(Math.pow((Cx - x), 2) + Math.pow((Cy - y), 2));

		return error < 0.5;
	}
	
	/**
	 * This method make the robot avoid the obstacle and take a route to the right
	 * After avoiding for 60cm in distance, stop the avoiding process
	 * @return void
	 */
	public static void RightAvoid() {
		//make the robot turn 90 degrees to the right
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(Lab3Navigation.WHEEL_RAD, Lab3Navigation.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab3Navigation.WHEEL_RAD, Lab3Navigation.TRACK, 90), false);
		
		// turn the sensor to face the obstacle
		sensorMotor.setSpeed(SENSOR_SPEED);
		sensorMotor.rotate(SENSOR_TURN, false);  
		
		
		Sound.beep(); // make a sound
		
		double xDoged=0, yDoged=0, prevX, prevY;		
		prevX = odo.getXYT()[0];
		prevY = odo.getXYT()[1];
		while(true){			
			cur_X = odo.getXYT()[0];
			cur_Y = odo.getXYT()[1];
			xDoged = xDoged + Math.abs(cur_X - prevX);  //here the entire avoidance distance is calculated
			yDoged = yDoged + Math.abs(cur_Y - prevY);
			prevX = cur_X;
			prevY = cur_Y;
			if (Math.hypot(xDoged, yDoged) > AVOID_DIST){  // when the avoidance distance is greater then 60, end the wall following algorithm
				sensorMotor.setSpeed(SENSOR_SPEED);
				sensorMotor.rotate(-SENSOR_TURN, false);  // turn the sensor back in front
				return;
			}		 
		    int currentDistance = 0;
		    while (currentDistance < safeDistance) {  
		    	usSensor.fetchSample(usData, 0); 
		    	currentDistance = (int) (usData[0] * 100.0);  // acquire data from the US-sensor
		    	int error = currentDistance - safeDistance;
		    	if(Math.abs(error) <= 3 )	                      // if the distance is smaller than error band, move forward
		    	{
		    		leftMotor.setSpeed(ROTATE_SPEED);
		    		rightMotor.setSpeed(ROTATE_SPEED);
		    		leftMotor.forward();
		    		rightMotor.forward();
		    	}
		    	else if(error < 0){	 // too close: jerk the cart to right
		    		leftMotor.setSpeed(ROTATE_SPEED);	
		    		rightMotor.setSpeed(ROTATE_SPEED);		
		    		leftMotor.forward();
		    		rightMotor.backward();                 
		    	}
		    	else{   //// too close: turn the cart to left
		    		leftMotor.setSpeed(ROTATE_SPEED -DELTA_SPEED);
		    		rightMotor.setSpeed(ROTATE_SPEED );
		    		leftMotor.forward();
		    		rightMotor.forward();
		    	}
		    }
		}
	}
	
	/**
	 * This method make the robot avoid the obstacle and take a route to the left
	 * After avoiding for 60cm in distance, stop the avoiding process
	 * @return void
	 */
	public static void leftAvoid() {
		//make the robot turn 90 degrees to the left
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(-convertAngle(Lab3Navigation.WHEEL_RAD, Lab3Navigation.TRACK, 90), true);
		rightMotor.rotate(convertAngle(Lab3Navigation.WHEEL_RAD, Lab3Navigation.TRACK, 90), false);
		
		// turn the sensor to face the obstacle
		sensorMotor.setSpeed(SENSOR_SPEED);
		sensorMotor.rotate(-SENSOR_TURN, false); 
		
		Sound.beep();		// make a sound
		
		double xDoged=0, yDoged=0, prevX, prevY;	       
		prevX = odo.getXYT()[0];
		prevY = odo.getXYT()[1];
		
		while(true){	
			cur_X = odo.getXYT()[0];
			cur_Y = odo.getXYT()[1];
			xDoged = xDoged + Math.abs(cur_X - prevX);  //here the entire avoidance distance is calculated
			yDoged = yDoged + Math.abs(cur_Y - prevY);
			prevX = cur_X;
			prevY = cur_Y;
			if(Math.hypot(xDoged, yDoged) > AVOID_DIST){  // when the avoidance distance is greater then 60, end the wall following algorithm
				sensorMotor.setSpeed(SENSOR_SPEED);
				sensorMotor.rotate(SENSOR_TURN, false);  //turn the sensor back in front
				return;
			}		 
			int currentDistance = 0;
	
			
			while (currentDistance < safeDistance) {
				usSensor.fetchSample(usData, 0); // acquire data from the US-sensor
				currentDistance = (int) (usData[0] * 100.0);
				int error = currentDistance - safeDistance;
				if(Math.abs(error) <= 3 ){       // if the distance is smaller than error band, move forward
					leftMotor.setSpeed(ROTATE_SPEED);
					rightMotor.setSpeed(ROTATE_SPEED);
					leftMotor.forward();
					rightMotor.forward();
				}
				else if(error < 0){	             // too close: jerk the cart to left
					leftMotor.setSpeed(ROTATE_SPEED);	
					rightMotor.setSpeed(ROTATE_SPEED);		
					leftMotor.backward();
					rightMotor.forward();               
				}
				else{                            // too far: jerk the cart to right
					leftMotor.setSpeed(ROTATE_SPEED); 
					rightMotor.setSpeed(ROTATE_SPEED -DELTA_SPEED);
					leftMotor.forward();
					rightMotor.forward();
				}
			}
		}
	}
	
	/**
	  * A method to determine whether the robot should dodge the block
	  * to the left or to the right (return 1 for right and 0 for left)
	  * based on the current position shown on the odometer
	  * @return A token 1,0 for right/left dodge
	  */
	public static int predictPath() {
		cur_X = odo.getXYT()[0];
		cur_Y = odo.getXYT()[1];
		cur_Theta = odo.getXYT()[2];
			
		if (cur_Theta > 350 || cur_Theta <= 10) {//going up
			if (cur_X <= Tile_Size ) { // In the left half board, so should dodge right
				return 1;              // 1 represents right dodge and 0 represents left dodge
			} 
			else {
				return 0;
			}
		} 
		else if(cur_Theta >= 80 && cur_Theta < 100){//going right
			if (cur_Y < Tile_Size) { // in the lower half, should dodge left
				return 0;
			} 
			else {
				return 1;
			}
		}
		else if(cur_Theta > 170 && cur_Theta < 190){//going down
			if (cur_X < Tile_Size) {
				return 0;
			} 
			else {
				return 1;
			}
		}
		else if(cur_Theta > 260 && cur_Theta < 280){//going left
			if (cur_Y <= Tile_Size) {
				return 1;
			} 
			else {
				return 0;
			}
		}
		else{
			return 1;
			}
		}

}
