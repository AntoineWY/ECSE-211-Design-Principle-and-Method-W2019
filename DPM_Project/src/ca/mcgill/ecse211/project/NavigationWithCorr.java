package ca.mcgill.ecse211.project;

/* this class will be responsible for navigating to a point and turning to minimal angles*/

// non-static imports
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

// static imports from Lab3 class
import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.TILE;
import static ca.mcgill.ecse211.project.project.TRACK;
import static ca.mcgill.ecse211.project.project.WHEEL_RAD;

import java.util.Arrays;



/**
 * This class implements the modified navigator. Some of the existing motor instances and a few
 * constants not dependent on navigation. Helper methods are added at the
 * end to make conversions easier.
 * 
 * <p>
 * The front set as 0 degrees Theta, the right
 * as the +x direction, and the front as the +y direction. Theta increases when turning in the
 * clockwise direction. The robot first turns to face the point it is travelling to, and then
 * travels to that point.
 * 
 * <p>
 * Modifications are mainly the re-localizing ability implemented during the navigation. Also,
 * the navigation has memory of what points have been reached. This is done by using the global stack to 
 * store the points.
 * 
 * @author Antoine Wang
 * @author Raymond Yang
 * @author Erica De Petrillo
 */
public class NavigationWithCorr{

  // -----------------------------------------------------------------------------
  // Constants
  // ----------------------------------------------------------------------------
	
  /**
   * distance recorded by the us sensor at which there must be a can.
   */
  private static final float CAN = 10; 
	 
  /**
   * A constant factor that can be applied to convert angular units in degrees to radians
   */
  private static final double TO_RAD = Math.PI / 180.0;

  /**
   * A constant factor that can be applied to convert angular units in radians to degrees
   */
  private static final double TO_DEG = 180.0 / Math.PI;

  /**
   * The speed at which the robot moves straight (in deg/sec)
   */
  private static final int FWDSPEED = 300;

  /**
   * The speed at which the robot turns in a stationary fashion (in deg/sec)
   */
  private static final int TURNSPEED = 200;

  /**
   * Angle correction for Quadrant 1 and 4. Arctan returns the correct angle and the only adjustment
   * needed is to turn it into an angle that starts from Theta = 0 and increases clockwise
   */
  private static final int Q1Q4COR = 90;

  /**
   * Angle correction for Quadrant 2 and 3. Arctan returns the incorrect angle and the adjustment
   * needed is to add pi to the angle and turn it into an angle that starts from Theta = 0 and
   * increases clockwise
   */
  private static final int Q2Q3COR = 270;

  /**
   * The center of the board platform that the EV3 runs on. This is used in determining which
   * quadrant the destination coordinate is in relative to the robot's current location and whether
   * arctan needs correction
   */
  private static final int CENTER = 0;

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  private static final int SMOOTH_ACCELERATION = 500;

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  private static final int TURN_ACCELERATION = 500;

  /**
   * A revolution of half of a circle in degrees
   */
  private static final int HALF_CIRCLE = 180;

  /**
   * A full revolution of a circle in degrees
   */
  private static final int FULL_CIRCLE = 360;

  /**
   * The heading/Theta value of the robot initially
   */
  private static final int INITIAL_ANGLE = 0;

  
  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  /**
   * The odometer instance
   */
  private Odometer odo;

  /**
   * Records odometer data returned, in a double precision array, specifying X, Y, and Theta values
   */
  private double position[];

  /**
   * A volatile boolean variable to indicate whether robot is currently travelling along a path,
   * from one way point to the next. The volatile keyword tells the thread to check the current
   * value of this variable in the main memory first
   */
  private volatile boolean isNavigating;
  
  //Field for receiving light sensor data
  private SampleProvider LeftidColour;
  private SampleProvider RightidColour;
  private float[] LeftcolorValue;
  private float[] RightcolorValue;
  
  //An instance of DoubleLightLocalization class
  private DoubleLightLocalization dll;

  


  // -----------------------------------------------------------------------------
  // Constructor
  // -----------------------------------------------------------------------------

  /**
   * Constructor for this class, sets up the odometer instance to allow access to position data and
   * initializes the isNavigating flag to false
   * 
   * @param odometer - the odometer instance passed from Lab3, gives access to retrieve position
   *        data
   */
  public NavigationWithCorr(Odometer odometer, SampleProvider left, SampleProvider right, 
		  float[] leftdata, float[] rightdata, DoubleLightLocalization dll) {
	  
    this.odo = odometer;
    this.isNavigating = false;
    this.LeftidColour = left;
    this.RightidColour = right;
    this.LeftcolorValue = leftdata;
    this.RightcolorValue = rightdata;
    this.dll = dll;

  }

  // -----------------------------------------------------------------------------
  // Public Methods
  // -----------------------------------------------------------------------------

  /**
   * This method is a series of travelTo which converts the diagonal path between starting and ending point
   * into an L shape route.
   * <p>
   * since the robot always goes in orthogonal directions, the line correction routine is useful to 
   * travel and locate the robot onto every grid point arrived during the path. 
   * <p>
   * Later not used for the accumulating error resulting from every TravelTo() segment on the way. To save time, constant line correction
   * is also revoked
   * @param cur_x robot's current X value
   * @param cur_y robot's current Y value
   * @param final_x x coordinate of destination point
   * @param final_y y coordinate of destination point
   * @deprecated
   */
  public void navigateTo(double cur_x, double cur_y, double final_x, double final_y){
	  if(cur_x > final_x){
		  while(cur_x > final_x){
			  try {
				Thread.sleep(200);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			  travelTo(cur_x -1, cur_y);
				  cur_x--;
		  
		  }
	  }else if(cur_x < final_x ){
		  while(cur_x < final_x){
			  try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			  travelTo(cur_x + 1, cur_y);
				  cur_x++;
		 
		  }
	  }else{	//equal
	  }
	  
	  cur_x = final_x;
	  
	  if(cur_y > final_y){
		  while(cur_y > final_y){
			  try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			  travelTo(cur_x, cur_y - 1);
				  cur_y--;
		  
		  }
	  }else if(cur_y < final_y ){
		  while(cur_y < final_y){
			  try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			 travelTo(cur_x, cur_y + 1);
				  cur_y++;
		  
		  }
	  }else{	//equal
	  }
  }

  /**
   * Controls the robot to travel to the coordinate (x,y) with the robot's initial starting location
   * as the origin. This is done by retrieving current position data of the robot and calculating
   * the new heading the robot needs to have, as well as the distance the robot needs to travel to
   * reach its next destination. A minimum angle approach is taken, meaning that the robot will turn
   * the smallest angle possible to adjust its heading.
   * 
   * <p>
   * There is a logic implemented in this method to determine the angle the robot needs to turn,
   * clockwise, to reach its new heading. This logic is necessary largely due to the values returned
   * by arctan function. The arctan function only returns values ranging from -pi/2 to pi/2, and
   * real values can be + or - pi from the returned value.
   * 

   * @param x - the x coordinate with the robot as the origin (0,0)
   * @param y - the y coordinate with the robot as the origin (0,0)
   */
  public void travelTo(double x, double y) {
	  
	    System.out.println("going to" + x +", "+ y); // debug purpose

	  	

	    // convert input coordinates x and y into distances in cm
	    x = x * TILE;
	    y = y * TILE;

	    isNavigating = true; // update navigating status
	    position = odo.getXYT(); // get current position data from odometer

	    // position[0] = x, position[1] = y, position[2] = theta
	    double dx = x - position[0]; // displacement in x
	    double dy = y - position[1]; // displacment in y
	    double ds = Math.hypot(dx, dy); // calculates the hypotenuse of dx and dy --> gives the
	                                    // displacement robot will
	                                    // need to travel to get to destination
	    
	    try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    double dTheta = Math.atan(dy / dx) * TO_DEG;// calculates the angle the new displacement will
	                                                // be. Value in the
	                                                // range of [-90,90] degrees

	    try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    // Convention: north = 0 degrees; Theta increases clockwise
	    // The following logic is used to determine the angle the robot needs to turn
	    if (dTheta >= CENTER && dx >= CENTER) {
	      // 1st quadrant
	      dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
	    } else if (dTheta >= CENTER && dx < CENTER) {
	      // 3rd quadrant, need to correct arctan value
	      dTheta = Q2Q3COR - dTheta; // clockwise angle robot needs to turn
	    } else if (dTheta < CENTER && dx >= CENTER) {
	      // 4th quadrant
	      dTheta = Q1Q4COR - dTheta; // clockwise angle robot needs to turn
	    } else if (dTheta < CENTER && dx < CENTER) {
	      // 2nd quadrant, need to correct arctan value
	      dTheta = Q2Q3COR - dTheta; // absolute angle
	    }

	    turnTo(dTheta); // robot turns at minimum angle
	    try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	


	    // smooth acceleration so that wheels do not slip
	    RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
	    LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);

	    // sets both motors to forward speed
	    RIGHT_MOTOR.setSpeed(FWDSPEED);
	    LEFT_MOTOR.setSpeed(FWDSPEED);
    
	
	    // rotates both motors for a fixed number of degrees equivalent to ds, the
	    // distance from the robot's current location to the next destination point,
	    // equivalent as travelling straight. The boolean flag parameter indicates
	    // whether method returns immediately, to allow simultaneous execution of both
	    // rotate() methods. The method waits for the right motor to complete.
	  
	    LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), true);
	    RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, ds), false);
	  
	 
	    isNavigating = false; // update navigation status
	  }
  
  

 
  /**
   * Turns the robot at a fixed position to face the next destination, changes the robot heading.
   * The minimum angle is calculated by determining whether the clockwise angle Theta is greater
   * than half of a full circles
   * 
   * @param Theta - the clockwise angle to turn from Theta = 0
   */
  public void turnTo(double Theta) {

    isNavigating = true; // update navigating status

    // smoother turn acceleration to avoid wheels slipping
    LEFT_MOTOR.setAcceleration(TURN_ACCELERATION);
    RIGHT_MOTOR.setAcceleration(TURN_ACCELERATION);
 
    // ensure angle is positive and within 360
    double minTheta = ((Theta - odo.getXYT()[2]) + FULL_CIRCLE) % FULL_CIRCLE;

    if (minTheta > INITIAL_ANGLE && minTheta <= HALF_CIRCLE) {
      // angle is already minimum angle, robot should turn clockwise
      RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    } else if (minTheta > HALF_CIRCLE && minTheta < FULL_CIRCLE) {
      // angle is not minimum angle, robot should turn counter-clockwise to the
      // complementary angle of a full circle 360 degrees
      minTheta = FULL_CIRCLE - minTheta;
      RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    }

    isNavigating = false; // update navigation status
  }

  /**
   * Wrapper method to determine whether robot is currently navigating by checking class variable
   * {@code isNavigating}
   * 
   * @return true if another thread has called travelTo() or turnTo() and the method has yet to
   *         return, false otherwise
   */
  public boolean isNavigating() {
    return isNavigating;
  }
  
  
  /**
   * The method perform a simpler version double light localization. Then correct the odometer reading with the 
   * x,y coordinates inputs. The theta value is also corrected with caliOdoTheta().
   * @param x - X coordinate of the current point which will be updated on the odometer
   * @param y - y coordinate of the current point which will be updated on the odometer
   * @see {@link DoubleLightLocalization#DoubleLocalizer()}
   * @see {@link #caliOdoTheta()}
   */
  public void localizeOnTheWay(double x, double y){
	  
	  dll.travelToLine(); // fixed itself on to the first grid line
	  
	  //Back off to lay the center onto the line
	  LEFT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), true);
	  RIGHT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), false);
	  
	  // turn 90 degree left
	  DoubleLightLocalization.reorientRobot(Math.PI/2);
	  
	  // fixed itself on to the 2nd grid line
	  dll.travelToLine();
	  
	  //Back off to lay the center onto the line
	  LEFT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), true);
	  RIGHT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), false);
	  
	  //turn back to original heading
	  DoubleLightLocalization.reorientRobot(-Math.PI/2);
	  
	  //correct odometer value
	  caliOdoTheta();
	  odo.setX(x * TILE);
	  odo.setY(y * TILE);
	  
	  
  }
 

  /**
   * The method will only be called when the robot localize AT THE DESIGNATED POINTS NEAR THE TUNNEL!
   * <P>
   * Perform a simpler version double light localization. Detail please refer to DoubleLightLocalization class.
   * Then calibrate the odometer readings based on the coordinates passed. The theta value is also corrected with caliOdoTheta().<br>
   * The most important function is that this method memorized the point it localized at by pushing the coordinates
   * into the global stack.
   * @param x - X coordinate of the current point which will be updated on the odometer
   * @param y - y coordinate of the current point which will be updated on the odometer
   * @see {@link DoubleLightLocalization#DoubleLocalizer()}
   * @see {@link #caliOdoTheta()}
   * @see {@link project#keyPoints}
   */
  public void locaNearTunnel(double x, double y){
	  dll.travelToLine(); // fixed itself on to the first grid line
	  
	  //Back off to lay the center onto the line
	  LEFT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), true);
	  RIGHT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), false);
	  
	  // turn 90 degree left
	  DoubleLightLocalization.reorientRobot(Math.PI/2);
	  
	  // fixed itself on to the 2nd grid line
	  dll.travelToLine();
	  
	  //Back off to lay the center onto the line
	  LEFT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), true);
	  RIGHT_MOTOR.rotate(-convertDistance(project.WHEEL_RAD, DoubleLightLocalization.SENSOR_TOWHEEL), false);
	  
	  //turn back to original heading
	  DoubleLightLocalization.reorientRobot(-Math.PI/2);
	  
	  //Correct odometer
	  caliOdoTheta();
	  odo.setX(x * TILE);
	  odo.setY(y * TILE);
	  
	  // Push the point in, useful when the robot travelling back home
	  project.keyPoints.push(new int[]{ (int)x , (int)y });
	  
	  
  }
  
  /**
   * This method is to drive the vehicle through the tunnel. 
   * The traveling distance is 4 tile long, making sure the start and end points are all 1 tile away from the tunnel.
   * Before entering the tunnel the robot will do a line correction then traverse the rest of the distance
   */
  
  public void throughTunnel(){
	 
	  dll.travelToLine(); //line correction

	  RIGHT_MOTOR.setAcceleration(1000);
	  LEFT_MOTOR.setAcceleration(1000);
	  
	  RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, TILE * 4 - DoubleLightLocalization.SENSOR_TOWHEEL), true);
	  LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(WHEEL_RAD, TILE * 4 - DoubleLightLocalization.SENSOR_TOWHEEL), false);
  }
  
  
  /**
   * Useful method of re-calibrating the robot's heading by changing theta value on the odometer.
   * <p>
   * This method is used when the robot passes and corrects itself on a grid line, or finished a localization routine.
   * Its theta value will be corrected into orthogonal values based on its current heading on the odometer
   */
  public void caliOdoTheta(){
	  if (odo.getXYT()[2] > 60 && odo.getXYT()[2] < 120){
		  odo.setTheta(90);
	  }else if (odo.getXYT()[2] > 150 && odo.getXYT()[2] < 210){
		  odo.setTheta(180);
	  }else if (odo.getXYT()[2] > 240 && odo.getXYT()[2] < 300){
		  odo.setTheta(270);
	  }else if (odo.getXYT()[2] > 330 || odo.getXYT()[2] < 30){
		  odo.setTheta(0);

	  }
	  try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
  }
  
  /**
   * This is a static method allows the conversion of a distance to the total rotation of each wheel
   * need to cover that distance.
   * 
   * (Distance / Wheel Circumference) = Number of wheel rotations. Number of rotations * 360.0
   * degrees = Total number of degrees needed to turn.
   * 
   * @param radius - Radius of the wheel
   * @param distance - Distance of path
   * @return an integer indicating the total rotation angle for wheel to cover the distance
   */
  public static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This is a static method that converts the angle needed to turn at a corner to the equivalent
   * total rotation. This method first converts the degrees of rotation, radius of wheels, and width
   * of robot to distance needed to cover by the wheel, then the method calls another static method
   * in process to convert distance to the number of degrees of rotation.
   * 
   * @param radius - the radius of the wheels
   * @param width - the track of the robot
   * @param angle - the angle for the turn
   * @return an int indicating the total rotation sufficient for wheel to cover turn angle
   */
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
 
}

