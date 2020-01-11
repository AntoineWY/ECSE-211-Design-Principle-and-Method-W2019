package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.lab4.DPM_Lab4;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * 
 * @author Tudor Gurau
 * @author Antoine Wang
 *
 */

public class Navigation extends Thread {

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private double deltax;
	private double deltay;

	// current location of the vehicle
	private double currx;
	private double curry;
	private double currTheta;

	// set constants
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 80;
	private static final double TILE_SIZE = 30.48;

	private boolean navigate = true;

	// constructor for navigation
	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	// main run method for navigation
	public void run() {


		//PATH 1 
		//		travelTo(0.0, 2 * TILE_SIZE);
		//		travelTo(TILE_SIZE, TILE_SIZE);
		//		travelTo(2 * TILE_SIZE, 2 * TILE_SIZE);
		//		travelTo(2*TILE_SIZE, TILE_SIZE);
		//		travelTo(TILE_SIZE, 0.0);



		//PATH 2 
		travelTo(TILE_SIZE, TILE_SIZE);
		travelTo(0.0, 2 * TILE_SIZE);
		travelTo(2 * TILE_SIZE, 2 * TILE_SIZE);
		travelTo(2*TILE_SIZE, TILE_SIZE);
		travelTo(TILE_SIZE, 0.0);



		//PATH 3 
		//		travelTo(TILE_SIZE, 0.0);
		//		travelTo(2*TILE_SIZE, TILE_SIZE);
		//		travelTo(2 * TILE_SIZE, 2 * TILE_SIZE);
		//		travelTo(0.0, 2 * TILE_SIZE);
		//		travelTo(TILE_SIZE, TILE_SIZE);



		//PATH 4 
		//		travelTo(0.0,TILE_SIZE);
		//		travelTo(TILE_SIZE,2*TILE_SIZE);
		//		travelTo(TILE_SIZE, 0.0);
		//		travelTo(2*TILE_SIZE, TILE_SIZE);
		//		travelTo(2 * TILE_SIZE, 2 * TILE_SIZE);



	}

	/**
	 * A method to drive our vehicle to a certain Cartesian coordinate
	 * 
	 * @param x
	 *            X-Coordinate
	 * @param y
	 *            Y-Coordinate
	 */
	public void travelTo(double x, double y) {

		currx = odometer.getXYT()[0];
		curry = odometer.getXYT()[1];

		deltax = x - currx;
		deltay = y - curry;

		// Calculate the angle to turn around
		currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
		double mTheta = Math.atan2(deltax, deltay) - currTheta;

		double hypot = Math.hypot(deltax, deltay);

		// Turn to the correct angle towards the endpoint
		turnTo(mTheta);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		leftMotor.rotate(convertDistance(DPM_Lab4.WHEEL_RAD, hypot), true);
		rightMotor.rotate(convertDistance(DPM_Lab4.WHEEL_RAD, hypot), false);

		// stop vehicle
		leftMotor.stop(true);
		rightMotor.stop(true);
	}

	/**
	 * A method to turn our vehicle to a certain angle
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {

		// ensures minimum angle for turning
		if (theta > Math.PI) {
			theta -= 2 * Math.PI;
		} else if (theta < -Math.PI) {
			theta += 2 * Math.PI;
		}

		// set Speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// rotate motors at set speed

		// if angle is negative, turn to the left
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, -(theta * 180) / Math.PI), true);
			rightMotor.rotate(convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, -(theta * 180) / Math.PI), false);

		} else {
			// angle is positive, turn to the right
			leftMotor.rotate(convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, (theta * 180) / Math.PI), true);
			rightMotor.rotate(-convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, (theta * 180) / Math.PI), false);
		}
	}

	/**
	 * A method to determine whether another thread has called travelTo and turnTo
	 * methods or not
	 * 
	 * @return
	 */
	boolean isNavigating() throws OdometerExceptions {
		return navigate;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}