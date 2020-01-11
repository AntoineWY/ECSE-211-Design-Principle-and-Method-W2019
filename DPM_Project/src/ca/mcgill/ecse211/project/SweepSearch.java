package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.project.SENSOR_MOTOR;

import java.util.Arrays;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This class is a new search class with a more effective way. 
 * <p>
 * The general procedure is the vehicle sweep for 90 degree and locate the nearest can
 * using the odometer (determining the angle) and the ultrasonic sensor (determine distance). 
 * Then the robot proceed and read the color, perform the weight detecting routine and issuing correct beeps.
 * Finally it returns to searchLL point holding the can inside of the gripper. <br>
 * Note that the weighting and the color reading routine is also integrated in this class. The robot will always 
 * choose a specific point in the search zone on which it will assess the can. 
 * 
 * @author Yinuo A Wang
 *
 */
public class SweepSearch {

	// -----------------------------------------------------------------------------
	// Constants
	// ----------------------------------------------------------------------------
		
	public static final int LLx = project.zone_LL_x; // lower left x coordinate of searching area, modify during demo
	public static final int LLy = project.zone_LL_y; // lower left y coordinate of searching area, modify during demo
	public static final int URx = project.zone_UR_x; // upper right x coordinate of searching area, modify during demo
	public static final int URy = project.zone_UR_y; // upper right y coordinate of searching area, modify during demo

	public static final int CORNER = project.corner;// starting corner received from the main class

	/**
	 * lower limit of the point of can that the robot will catch
	 */
	private static final int TR = 0; 
	
	/**
	 * largest distance to show there is a can. The aim of this bound is to avoiding reading the wall as a can
	 */
	public static final double WALL_DIST = project.TILE * 2;
	
	/**
	 * ultrasonic sampling rate
	 */
	private static final int UL_TIME = 20; 
	
	/**
	 * The speed the robot sweep itself while looking for the can
	 */
	private static final int SWEEP_SPEED = 100;

	/**
	 * field storing the closest can's distance
	 */
	private static double can_close; 
	/**
	 * field storing the closest can's angle
	 */
	private static double can_angle;

	// -----------------------------------------------------------------------------
	// class fields
	// ----------------------------------------------------------------------------
		
	private Odometer odometer; // odometer instance
	
	//Fields to hold ultrasonic reading
	private float[] usData;
	private SampleProvider usDistance;
	
	// can calibrator instance
	private CanCalibrator calibrator;

	private NavigationWithCorr nav; // navigation instance
	private Handling handler;// handing instance
	private WeightID2 weighter; // weight can instance

	
	// -----------------------------------------------------------------------------
	// constructor
	// ----------------------------------------------------------------------------
	/**
	 * Constructor of the class.<p>
	 * Instances of odometer and navigation leads the robot into the search zone and perform localization. Handling and WeightID2 instances
	 * are to acquire the can and perform weight reading. Lastly, the sample provider and data arrays of UL-sensor and light sensor are 
	 * passed in to read can's location and color. 
	 * @param odometer Instance of odometer class
	 * @param usDistance Sample provider of ultrasonic sensor
	 * @param usData data array holding ultrasonic sensor's reading
	 * @param lightColor Sample provider of light sensor
	 * @param lightData data array holding light sensor's reading
	 * @param nav Instance of navigation class
	 * @param handler Instance of Handling class
	 * @param weighter Instance of weightID2 class
	 */
	public SweepSearch(Odometer odometer, SampleProvider usDistance, float[] usData, SampleProvider lightColor,
			float[] lightData, NavigationWithCorr nav, Handling handler, WeightID2 weighter) {
		this.odometer = odometer;
		this.usData = usData;
		this.usDistance = usDistance;
		this.nav = nav;
		this.handler = handler;
		this.weighter = weighter;
		
		// A canCalibrator instance is constructed for reading the can's color
		calibrator = new CanCalibrator(lightColor, lightData);

	}
	
	// -----------------------------------------------------------------------------
	// Public method
	// ----------------------------------------------------------------------------

	/**
	 * Main method of the search class<P>
	 * 1. select 2 points on the search zone, at each point select the closest can <br> 
	 * 2. Starting point: robot at search zone, localized at either 180 or 0 degree <br> 
	 * 3. Sweep to locate can<br>
	 * 4. if color is read and the point of the can is high enough, issue beeps and heading to the searchLL<br>
	 * 5. perform weight identification, issue beeps
	 * 6. if color point is low or color is undetermined, heading to another point in search zone 
	 * and search again<br>
	 * @see {@link MapPlanner#toSearchZone(NavigationWithCorr, Odometer)}
	 * The exact point selected for search  
	 */
	public void search(int[] point) {
        //Find the can, go to it, read the color and bring it back to search point
		int canColor = assessCan();
		
		//reset motor speed after sweep slowly for the can
		project.LEFT_MOTOR.setSpeed(300);
		project.RIGHT_MOTOR.setSpeed(300);
		
		if (canColor > TR) { //  a valuable enough can
			
			//back to the point
			project.LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close), true);
			project.RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close), false);

			if (CORNER == 2){ // robot is at the ur corner
				nav.turnTo(180);
			}else{// robot at the LL corner
				nav.turnTo(0);
			}

			SENSOR_MOTOR.rotate(-10);// loosen the claw to weight, makes the difference between heavy and light can's more significant
			boolean isHeavy = weighter.weightAt(point[0], point[1]); // weight + localization
			SENSOR_MOTOR.rotate(10);// tighten the claw
			
			//issue appropriate beeps
			if (isHeavy) {
				longBeepNTimes(canColor);
			} else {
				shortBeepNTimes(canColor);
			}

		} else { // if read on white part, do localize first, reoriented the can
					// and read again		
			project.LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close), true);
			project.RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close), false);
			
			SENSOR_MOTOR.rotate(-10);// loosen the claw to weight
			
			if (CORNER == 2){ // robot is at the ur corner
				nav.turnTo(180);
			}else{// robot at the LL corner
				nav.turnTo(0);
			}
			
			boolean isHeavy = weighter.weightAt(point[0], point[1]); // weight + localization
			SENSOR_MOTOR.rotate(10);// tighten the claw
			
			canColor = detectColor();
			if (isHeavy) {
				longBeepNTimes(canColor);
			} else {
				shortBeepNTimes(canColor);
			}
		}

		if (canColor < TR) { // if the color of the can worth too low points or is undetermined, heading to another point and search again
			
			handler.release(false); // release the low point can
			project.LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, 5), true); // back off																							// off
			project.RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, 5), false);

			SENSOR_MOTOR.rotateTo(90);// close the claw so that no can will enter on the way

			if (CORNER == 2){ // robot is at the ur corner, go to the point on its lower left
				point[0]--;
				point[1]--;
				nav.travelTo(point[0], point[1]);
				nav.turnTo(180);
				nav.localizeOnTheWay(point[0], point[1]);
			}else{ // robot is at the LL corner, go to the point on its upper right
				point[0]++;
				point[1]++;
				nav.travelTo(point[0], point[1]);
				nav.turnTo(0);
				nav.localizeOnTheWay(point[0], point[1]);
			}
			
		    // restore the claw to original position
			SENSOR_MOTOR.rotateTo(0);
		 
			//search again
			canColor = assessCan();

			//bring the can back
			project.LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close), true);
			project.RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close), false);

			if (CORNER == 2){ // robot is at the ur corner of search zone
				nav.turnTo(180);
			}else{
				nav.turnTo(0);
			}
			 
			boolean isHeavy = weighter.weightAt(point[0], point[1]); // weight +
																	// localization

			if (canColor < 0) {
				canColor = 3; // if the color is still unknown, guess it is a yellow can
			}

			if (isHeavy) {
				longBeepNTimes(canColor);
			} else {
				shortBeepNTimes(canColor);
			}
		}

	}

	/**
	 * This method rotate the machine for 90 degree, read the direction of the
	 * closest can and approach it. Then it grips the can, perform color
	 * detection.
	 * 
	 * @return colorIndex The identified color index of cans 
	 */
	public int assessCan() {
		
		sweep90Degree(); // Sweep the robot 90 degree, keep the direction and distance of the closest can in the class field
		
		approachCan(); // Heading to the can, place the can in the gripper
		
		handler.catchCan(true); // close the gripper to hold the can
		
		int color = detectColor(); //perform color reading 
		
		return color;

	}

	/**
	 * The method sweep the mechine for 90 degrees and detecting a sector of area for the closest can<p>
	 * During the turning process,
	 * it keeps polling the distance and updating the angle every time a smaller distance is caught. 
	 * After finishing one revolution, the robot is able to distinguish the closest can and its direction and distance.
	 */
	public void sweep90Degree() {

		project.LEFT_MOTOR.setSpeed(SWEEP_SPEED);
		project.RIGHT_MOTOR.setSpeed(SWEEP_SPEED);
		project.LEFT_MOTOR.rotate(NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, 90), true);
		project.RIGHT_MOTOR.rotate(-NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, 90), true);

		can_close = WALL_DIST;
		// A ultrasonic poller loop which keeps reading and updating the angle and distance
		while (project.LEFT_MOTOR.isMoving() || project.RIGHT_MOTOR.isMoving()) {

			long initTime = System.currentTimeMillis();

			double distance = medianFilter();
			if (distance < can_close) {
				can_close = distance;
				can_angle = odometer.getXYT()[2];
			}

			long endTime = System.currentTimeMillis();
			if ((endTime - initTime) < UL_TIME) {
				try {
					Thread.sleep(UL_TIME - (endTime - initTime));
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

		}
		
		//Resolving odometer error
		if ((25 < can_angle && can_angle < 60) ||
				(205 < can_angle && can_angle < 240) ) { //  corrections to the odometer reading 
			can_angle += 5;                              // to match with the actual can direction
		} else if (can_angle < 25 ||
				(180 < can_angle && can_angle < 205) ) {
			can_angle += 9;
		}
		
		// Set the correct heading of the odometer in the end 
		if (CORNER == 2){
			odometer.setTheta(270);
		}else{
			odometer.setTheta(90);
		}
	}

	/**
	 * Turn to the can, approach it by travelling the distance indicated by the ultrasonic sensor.
	 * The claw is also opened to place the can inside of the gripper
	 */
	public void approachCan() {
		nav.turnTo(can_angle); // Need to be changed since the delta theta on
								// the ul position is not tested

		SENSOR_MOTOR.rotateTo(-50);// open the claw
		project.LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close ), true);
		project.RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, can_close), false);

	}

	/**
	 * This method polls the color sensor reading, comparing it with the color
	 * RGB theoretical value. <p>
	 * First the method uses the canlibrate() method from the canCalibrator class which uses the read RGB
	 * value to reflect the color. It takes 7 readings at one times and then the majority of the color among
	 * 7 result is returned as the classified color. If there is no majority, return undetermined (-1);
	 * 
	 * @return int index of the classified
	 * @see {@link CanCalibrator#Calibrate()}
	 */
	public int detectColor() {
		// Create a array of length 7
		int[] colorResult = new int[7];
		for (int i = 0; i < colorResult.length; i++) {
			colorResult[i] = -1;
		}

		// Test 7 times
		for (int i = 0; i < 7; i++) {
			// Poll, process RGB, and classify
			colorResult[i] = calibrator.Calibrate();
		}
		// Color sensor motor return to Original position

		Arrays.sort(colorResult);

		// Below is the process of finding the majority of the legit elements
		// (-1 values are excluded)
		int prev = colorResult[0];
		int count = 1;
		for (int i = 1; i < colorResult.length; i++) {
			if (colorResult[i] == prev && colorResult[i] != -1) {
				count++;
				if (count > 7 / 2 - 1) {
					return colorconvert(colorResult[i]); // Find the majority
															// value (being
															// detected after
															// enough times and
					// return that index)
				}
			} else {
				count = 1;
				prev = colorResult[i];
			}
		}

		return -1; // No majority or no legit values to return
	}

	// -----------------------------------------------------------------------------
	// Private method
	// ----------------------------------------------------------------------------
	/**
	 * Helper method to convert the color indexes we defined into those defined by the requirement
	 * @param int The color index we defined (RGBY = 0123)
	 * @return tr requirement indexes (RYGB = 4321)
	 */
	private int colorconvert(int tr) {
		switch (tr) {
		case 0: // red required
			return 4;
		case 1: // green required
			return 2;
		case 3: // yellow required
			return 3;
		case 2: // blue required
			return 1;
		}
		return -1;
	}

	/**
	 * Issuing long beeps (1000ms) with specified amount of times
	 * Will be called for heavy can
	 * @param time Amount of beeps needed defined by the color of the can
	 */
	private void longBeepNTimes(int time) {
		for (int i = 0; i < time; i++) {
			Sound.playTone(659, 1000); // Note E5
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	/**
	 * Issuing short beeps (500ms) with specified amount of times
	 * Will be called for light can
	 * @param time Amount of beeps needed defined by the color of the can
	 */
	private void shortBeepNTimes(int time) {
		for (int i = 0; i < time; i++) {
			Sound.playTone(659, 250); // Note E5
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	/**
	   * This is a median filter. The filter takes 5 consecutive readings from the ultrasonic sensor,
	   * amplifies them to increase sensor sensitivity, sorts them, and picks the median to minimize the
	   * influence of false negatives and false positives in sensor readings, if any. The sensor is very
	   * likely to report false negatives.
	   * 
	   * @return the median of the five readings, sorted from small to large
	   */
	private double medianFilter() {
		double[] arr = new double[5]; // store readings
		for (int i = 0; i < 5; i++) { // take 5 readingss
			usDistance.fetchSample(usData, 0); // store reading in buffer
			arr[i] = usData[0] * 100.0; // signal amplification
		}
		Arrays.sort(arr); // sort readingss
		return arr[2]; // take median value
	}
}
