package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.TRACK;
import static ca.mcgill.ecse211.project.project.WHEEL_RAD;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

/**
 * This is the class which perform the weight identification.
 * <p>
 * 
 * This class uses a modified weighting algorithm using tachocount. The
 * fundamental idea is that heavy can produces more error during a 360 degree
 * full turn, thus to correct the orientation of the vehicle more distance needs
 * to be turned.<br>
 * 
 * @author Yunhao Hu
 *
 */
public class WeightID2 {
	// -----------------------------------------------------------------------------
	// Constants
	// -----------------------------------------------------------------------------
	/**
	 * Experimentally determined threshold distinguishing heavy and light can
	 */
	private static final double THRESHOLD = -30;

	// -----------------------------------------------------------------------------
	// fields
	// -----------------------------------------------------------------------------
	private SampleProvider LeftcsSensor;
	private float[] LeftcolorValue;

	private NavigationWithCorr nav;
	private DoubleLightLocalization dll;

	// -----------------------------------------------------------------------------
	// Constructor
	// -----------------------------------------------------------------------------
/**
 * Constructor of the WeightID2 class. 
 * @param LeftcsSensor sample provider of left light sensor
 * @param LeftcolorValue data array to hold left senspr reading
 * @param dll An instance of DoubleLightLocalization 
 * @param nav An instance of NavigationWithCorr
 */
	public WeightID2(SampleProvider LeftcsSensor, float[] LeftcolorValue, DoubleLightLocalization dll,
			NavigationWithCorr nav) {
		this.LeftcsSensor = LeftcsSensor;
		this.LeftcolorValue = LeftcolorValue;
		this.dll = dll;
		this.nav = nav;
	}
	
	// -----------------------------------------------------------------------------
	// Public method
	// -----------------------------------------------------------------------------

	/**
	 * The robot would localize on the point x, y The robot travels forward
	 * until a black line <br>The robot rotate on itself for 360 if the robot needs
	 * additional rotation of over 30 tacho count it is heavy
	 * 
	 * @param search_point_x
	 *            target x location for localization
	 * @param search_point_y
	 *            target y location for localization
	 * @return boolean determine whether the can is heavy (true) or light
	 *         (false)
	 */
	public boolean weightAt(double search_point_x, double search_point_y) {

		nav.localizeOnTheWay(search_point_x, search_point_y); // the robot
																// localize on
																// the search
																// point
		dll.travelToLine(); // travel to the line in front of it heading at 0
							// degree

		project.LEFT_MOTOR.setSpeed(400);// fast spin to show weight difference
											// through error
		project.RIGHT_MOTOR.setSpeed(400);
		project.LEFT_MOTOR.rotate(NavigationWithCorr.convertAngle(WHEEL_RAD, TRACK, 360), true);// rotate
																								// 360
																								// degree
		project.RIGHT_MOTOR.rotate(-NavigationWithCorr.convertAngle(WHEEL_RAD, TRACK, 360), false);

		int index = project.LEFT_MOTOR.getTachoCount(); // get initial
														// tachocount
		project.LEFT_MOTOR.setSpeed(150);
		project.RIGHT_MOTOR.setSpeed(150);
		while (fetchSampleLeft() > 0.30) { // since the error is always
											// insufficient turning, the left
											// wheel will fix the
			project.LEFT_MOTOR.forward(); // heading by keep turning until a
											// line is met
			project.RIGHT_MOTOR.backward();
		}
		project.LEFT_MOTOR.stop();
		project.RIGHT_MOTOR.stop(false);

		index = index - project.LEFT_MOTOR.getTachoCount(); // get difference of
															// tachocount: this
															// is the degree the
															// left wheel need
															// to turn to
															// correct the robot
															// to zero degree
		System.out.println(index);
		dll.travelToLine(); // Fix the robot to original 0 degree heading

		// back off to the original point
		LEFT_MOTOR.rotate(
				-NavigationWithCorr.convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), true);
		RIGHT_MOTOR.rotate(
				-NavigationWithCorr.convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), false);

		if (index <= this.THRESHOLD) { // more degree turn, tacho difference is
										// more negative
			return true;
		}
		return false;
	}

	
	// -----------------------------------------------------------------------------
	// Public method
	// -----------------------------------------------------------------------------
	/**
	 * A mean filter for light intensity reflected from the ground
	 * <p>
	 * polls the left light sensor for the ground for 10 readings Then takes the
	 * mean
	 * 
	 * @return current sample of the color
	 */
	private double fetchSampleLeft() {
		double filterSum = 0;
		for (int i = 0; i < 10; i++) {
			LeftcsSensor.fetchSample(LeftcolorValue, 0);
			filterSum += LeftcolorValue[0];
		}
		return filterSum / 10;
	}
}
