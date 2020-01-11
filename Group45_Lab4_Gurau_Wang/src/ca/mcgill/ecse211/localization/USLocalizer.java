package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.lab4.*;
/**
 * 
 * @author Tudor Gurau
 * @author Antoine Wang
 *
 */
public class USLocalizer {

	// vehicle constants
	private static final int ROTATION_SPEED = 80;
	private double deltaTheta;

	private Odometer odometer;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean isRisingEdge;
	private SampleProvider usDistance;


	// set threshold distance and tolerance for both rising and falling edge
	private static final double distance_falling = 18.00;
	private static final int tolerance_falling = 2;
	
	private static final double distance_rising = 30.00;
	private static final int tolerance_rising = 4;

	/**
	 * Constructor to initialize variables
	 * 
	 * @param Odometer
	 * @param EV3LargeRegulatedMotor
	 * @param EV3LargeRegulatedMotor
	 * @param boolean
	 * @param SampleProvider
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			boolean localizationType, SampleProvider usDistance) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.isRisingEdge = localizationType;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}

	/**
	 * A method to determine which localization method to write
	 * 
	 */
	public void localize() {
		if (isRisingEdge) {
			localizeRisingEdge();
		} else {
			localizeFallingEdge();
		}
	}

	/**
	 * A method to localize position using the rising edge
	 * 
	 */
	public void localizeRisingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to the wall
		while (fetchUS() > distance_rising) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate until it sees the open space
		while (fetchUS() < distance_rising + tolerance_rising) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.buzz();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate the other way all the way until it sees the wall
		while (fetchUS() > distance_rising - tolerance_rising) {
			leftMotor.forward();
			rightMotor.backward();
		}
		// rotate until it sees open space
		while (fetchUS() < distance_rising + tolerance_rising) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.buzz();
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2 + 180;
		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2 + 180;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 using turning angle and we account for small
		// error
		leftMotor.rotate(-convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, turningAngle + 2	), true);
		rightMotor.rotate(convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, turningAngle + 2), false);

		// set theta = 0.0
		odometer.setXYT(0.0, 0.0, 0.0);
		
	}

	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void localizeFallingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (fetchUS() < distance_falling + tolerance_falling) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate to the first wall
		while (fetchUS() > distance_falling) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// record angle
		angleA = odometer.getXYT()[2];
		Sound.buzz();

		// rotate out of the wall range
		while (fetchUS() < distance_falling + tolerance_falling) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second wall
		while (fetchUS() > distance_falling) {
			leftMotor.forward();
			rightMotor.backward();
		}
		angleB = odometer.getXYT()[2];
		Sound.buzz();

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 and we account for small error
		leftMotor.rotate(-convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, turningAngle + 1.5), true);
		rightMotor.rotate(convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, turningAngle + 1.5), false);

		// set odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}

	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return distance detected by US sensor
	 */
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius wheel radius
	 * @param distance distance traveled
	 * @return degree that wheel should turn
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius wheel radius
	 * @param distance_falling distance traveled
	 * @param angle angle in degree
	 * @return degree that each wheel needs to turn
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}