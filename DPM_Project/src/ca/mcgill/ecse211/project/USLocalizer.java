package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
//import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.project.*;
/**
 * <p>
 * This class implements localization with the ultrasonic sensor. The main method is localize().
 * Helper methods are added at the end to make conversions easier.
 * 
 * <p>
 * The robot is to be placed at the lower left tile of the demo platform, on the 45 degree line
 * (from the top-right corner to the bottom-left corner). The robot localizes to a heading of 0
 * degrees. The robot uses the convention that North = 0 degrees and clockwise turning = increase in
 * heading degrees. The falling edge detection is the default method. From testing, we find that falling edge works best and
 * resulting in less failure and error
 * 
 */
public class USLocalizer {
	
	// -----------------------------------------------------------------------------
    // Constants
	// -----------------------------------------------------------------------------
		
	/**
	 * The threshold distance (in cm) for detecting a falling edge
	 */
	private static final double DISTANCE = 40;
	
	/**
	 * The error band to ensure an actual falling edge is met
	 */
	private static final int TOLERANCE = 3;

	/**
	 * rotation speed of the vehicle
	 */
	private static final int ROTATION_SPEED = 300;
	
	// -----------------------------------------------------------------------------
    // Fields
	// -----------------------------------------------------------------------------
	
	/**
	 * Angle deviated from actual 0 heading, determined through calculation
	 */
	private double deltaTheta;

	private Odometer odometer;  // odometer instance

	private EV3LargeRegulatedMotor leftMotor, rightMotor; // motors

	// Instance of the Ultrasonic sensor to read distance
	private SampleProvider usDistance;
	private float[] usData;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------
	
	/**
	 * This is the constructor for the USLocalizer.
	 * 
	 * @param odo the Odometer of the robot
	 * @param leftMotor the Left Motor of the robot
	 * @param rightMotor the Right Motor of the robot
	 * @param localizationType the Localization type (rising or falling edge)
	 * @param usDistance the Sample Provider of the ultrasonic sensor
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			 SampleProvider usDistance) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}

	// -----------------------------------------------------------------------------
	// Public Method
	// -----------------------------------------------------------------------------

	/**
	 * This method localizes the robot's position using the falling edge<p>
	 * The method is used no matter the robot is heading to or away from the wall. The robot will first turn to a 
	 * open area then enter the falling edge detecting routine. It records the two odometer theta reading when it reads a falling edge.
	 * Later the angle of the actual deviation from the 0 degree reading is calculated. The robot will turn to approximately the 0 
	 * degree heading.
	 * 
	 */
	public void localize() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (fetchUS() < 60) {
			leftMotor.backward();
			rightMotor.forward();
		}
		odometer.setTheta(0);
		// Rotate to the first wall
		while (fetchUS() > DISTANCE + TOLERANCE) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// record angle
		angleA = odometer.getXYT()[2];
		
		// rotate out of the wall range
		leftMotor.rotate(250, true);
		rightMotor.rotate(-250, false);

		// rotate out of the wall range
		while (fetchUS() <= DISTANCE + TOLERANCE) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second wall
		while (fetchUS() > DISTANCE -  TOLERANCE) {
			leftMotor.forward();
			rightMotor.backward();
		}
		angleB = odometer.getXYT()[2];
		

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2] ;

		// rotate robot to the theta = 0.0 and we account for small error
		leftMotor.rotate(-NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, turningAngle ), true);
		rightMotor.rotate(NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, turningAngle ), false);

		// set odometer to theta = 0
		odometer.setTheta(0);
		odometer.Theta = odometer.getXYT()[2];

	}

	// -----------------------------------------------------------------------------
	// Private Method
	// -----------------------------------------------------------------------------
	/**
	 * This method gets the distance from the ultrasonic sensor
	 * 
	 * @return distance detected by US sensor
	 */
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}


}