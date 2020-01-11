package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import ca.mcgill.ecse211.odometer.*;
/**
 * This class performs localization using two sensors located at the side of the robot. The main method of this class
 * is localize(). It takes in no input and perform a regulated routine to locate the robot onto the point. Some of the 
 * method like static reorientRobot(theta) are also useful in other classes.
 * 
 * <p>
 * The robot perform the localization in the following steps:<br>
 * 1. travel forward. when one side of the light sensor polled a black line, the wheel on that side stops.
 * <br>
 * 2. the wheel on the other side forwards until it polled a black line. Now robot is parallel with that line.
 * <br>
 * 3. calibrate odometer
 * <br>
 * 4. Back off until the centre of the robot returns to the third quadrant of the origin 
 * <br>
 * 5. turn to 90 degree heading, perform the same task in step 1,2, calibrate odometer. 
 * <br>
 * 6. Back off until the centre of the robot returns to the third quadrant of the origin 
 * <br>
 * 7. Base on the odometer's reading, navigate to the origin point.
 * @author Antoine Wang
 * @author Yun Hao Hu
 *
 */

public class DoubleLightLocalization {
//----------------------------------------------------Constants---------------------------------------------------------------------------------
 /**
  * Distance of from the rotating center of the robot to the sensor
  */
  public static final double SENSOR_TOWHEEL = 12.6; 
  
  /**
   * Distance that the robot back off to the third quadrant which is slightly bigger then SENSOR_TOWHEEL distance
   * @see {@link #SENSOR_TOWHEEL}
   */
  private static final double BACK_DIST = SENSOR_TOWHEEL + 3; 
  
  
//----------------------------------------------------Fields------------------------------------------------------------------------------------  
  // odometer passed in for navigating to the original point
  private Odometer odometer;
  //sensor sample provider and arrays holding the sensor value passed in to detect black line
  private SampleProvider LeftidColour;
  private SampleProvider RightidColour;
  private float[] LeftcolorValue;
  private float[] RightcolorValue;
  
  
//----------------------------------------------------Constructor-------------------------------------------------------------------------------    
  /**
   * Constructor of this class
   * @param odometer Instance of the Odometer class 
   * @param navigator Instance of the Navigation class
   * @param LeftcsSensor Left side color sensor
   * @param RightcsSensor Right side color sensor
   */
  public DoubleLightLocalization(Odometer odometer, SampleProvider LeftcsSensor,
      SampleProvider RightcsSensor, float[] LeftcolorValue, float[] RightcolorValue) {

    this.odometer = odometer;
    this.LeftidColour = LeftcsSensor;
    this.RightidColour = RightcsSensor;
    this.LeftcolorValue = LeftcolorValue;
    this.RightcolorValue = RightcolorValue;
  }
  
  
//----------------------------------------------------Public Methods-----------------------------------------------------------------------------  
  /**
   * Localize the machine using 2 light sensors:
   * <p>
   * 1. With the robot heading roughly in positive y direction, drive forward until both sensor detected x axis
   * <br>
   * 2. Reset odometer value (Y coord) and the heading according to X axis
   * <br>
   * 3. Drive back 15cm to ensure the machine is in the 3rd quadrant relative to the origin
   * <br>
   * 4. turn 90 degree to the right. With the robot heading roughly in positive x direction, drive forward until both sensor detected y axis
   * <br>
   * 5. Reset odometer value (X coord) and the heading according to Y axis
   * <br>
   * 6. Drive back 15cm to ensure the machine is in the 3rd quadrant relative to the origin, then navigate to the original point.
   * <br>
   * 7. At the end of the method, the vehicle should be located at (0,0) with a heading of 0 degree
   */

  public void DoubleLocalizer() {

	// Drive to X axis and initialize Y
    travelToLine();
    odometer.setY(SENSOR_TOWHEEL + project.TILE);
    odometer.setTheta(0);
    
    //Drive back 20cm and turn 90 degree facing Y axis
    RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, BACK_DIST), true);
    LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, BACK_DIST), false);
    
    reorientRobot(Math.PI/2);
    // Drive to y axis and initialize x
    travelToLine();
    odometer.setX(SENSOR_TOWHEEL + project.TILE);
    odometer.setTheta(90);
    
    // Drive back for 20cm and then localize at the origin
    RIGHT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, BACK_DIST), true);
    LEFT_MOTOR.rotate(-NavigationWithCorr.convertDistance(project.WHEEL_RAD, BACK_DIST), false);
    travelToOrigin();
    
  
    
  }

  /**
   * Travel to the front until the 2 light sensors poll a black line
   * After the method the vehicle should be locate in orthogonal headings (0, 90, 270, 180) on the grid line
   * the steps are:<br>
   * 1. travel forward. when one side of the light sensor polled a black line, the wheel on that side stops. <br>
   * 2. the wheel on the other side forwards until it polled a black line. Now robot is parallel with that line.
   * 
   */
  public void travelToLine() {

    project.LEFT_MOTOR.setAcceleration(6000); // default acceleration of EV3 motor to offset accelerating behavior
    project.RIGHT_MOTOR.setAcceleration(6000);
    
    project.LEFT_MOTOR.setSpeed(220);
    project.RIGHT_MOTOR.setSpeed(220);


    boolean counter_left = true;  // counter means if the motor needs correct. Here true means
    boolean counter_right = true; // the motor is yet to be corrected thus need to drive forward to detect the line
    while (counter_left || counter_right) {
      if(counter_left) {
        project.LEFT_MOTOR.forward();
        
      }
      
      if(counter_right) {
        project.RIGHT_MOTOR.forward();
      
      }
       
      if(fetchSampleLeft() < 0.33 && counter_left) {  // when left sensor read the line, left is considered
        project.LEFT_MOTOR.stop(true);                // fixed thus left wheel stops
        project.RIGHT_MOTOR.stop();
       
        counter_left = false;
        try {
          Thread.sleep(200);
        }
        catch(Exception e) {
          
        }
        
        project.RIGHT_MOTOR.forward();
        if(fetchSampleLeft() < 0.33 && !counter_right) {  // Make sure that leftt wheel still stays on the line
          project.LEFT_MOTOR.stop(true);
          project.RIGHT_MOTOR.stop();
          break;      
        }
      }
       
      if(fetchSampleRight() < 0.33 && counter_right) {    // when right sensor read the line, left is considered
        project.RIGHT_MOTOR.stop(true);                   // fixed thus right wheel stops
        project.LEFT_MOTOR.stop();
       
        counter_right = false;
        try {
          Thread.sleep(200);
        }
        catch(Exception e) {
          
        }

        project.LEFT_MOTOR.forward();
        if(fetchSampleRight() < 0.33 && !counter_left) {
          project.LEFT_MOTOR.stop(true);
          project.RIGHT_MOTOR.stop();
          break;
        }
      }
    }
      
  }

  
  /**
   * This method turns the robot by the magnitude indicated by the input parameter
   * The input angle will always be brought down to the minimum angle needed to turn and
   * it can adjust itself in both cw and ccw directions. 
   * @param theta RADIAN value of the angle that the robot will adjust its orientation
   * 		For adjusting clockwise the parameter should be positive, for ccw adjustment please input a negative radian
   */
  public static void reorientRobot(double theta) {

    // ensures minimum angle for turning
    if (theta > Math.PI) {
        theta -= 2 * Math.PI;
    } else if (theta < -Math.PI) {
        theta += 2 * Math.PI;
    }

    // if angle is negative, turn to the left
    if (theta < 0) {
      double angle = theta * 180 / Math.PI;
      project.LEFT_MOTOR.rotate(-NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, -angle ), true);
      project.RIGHT_MOTOR.rotate(NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, -angle ), false);

    } else {
    	 double angle = theta * 180 / Math.PI;
        // angle is positive, turn to the right
      project.LEFT_MOTOR.rotate(NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, angle), true);
      project.RIGHT_MOTOR.rotate(-NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, angle), false);
    }
}
  
  /**
   * This method takes in the robot's current location and navigate the robot to the (0,0) point
   * Then it adjust the robot's orientation into zero degree (facing positive Y axis)
   */
  public void travelToOrigin() {
    double currx;
    double curry;
    double currTheta;
    double deltax;
    double deltay;
    
    //Get current location from odometer
    currx = odometer.getXYT()[0];
    curry = odometer.getXYT()[1];

    deltax = project.TILE - currx;
    deltay = project.TILE - curry;

    // Calculate the angle to turn around
    currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
    double mTheta = Math.atan2(deltax, deltay) - currTheta;

    //Calculate the distance from original point
    double hypot = Math.hypot(deltax, deltay);

    // Turn to the correct angle towards the endpoint
    reorientRobot(mTheta);

    //Setting the speed and proceed to the origin point
    project.LEFT_MOTOR.setSpeed(180);
    project.RIGHT_MOTOR.setSpeed(180);
    project.LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, hypot), true);
    project.RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, hypot), false);
    
    //Adjusting itself at 0 degree heading by turning in the opposite direction to offset the current odometer reading
    LEFT_MOTOR.rotate(-NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, odometer.getXYT()[2]), true);
    RIGHT_MOTOR.rotate(NavigationWithCorr.convertAngle(project.WHEEL_RAD, project.TRACK, odometer.getXYT()[2]), false);

    // stop vehicle
    project.LEFT_MOTOR.stop(true);
    project.RIGHT_MOTOR.stop(false);
    
    
  }
  
//----------------------------------------------------Private Methods-----------------------------------------------------------------------------  

  /**
   * A mean filter for light intensity reflected from the ground
   * <p>
   * polls the left light sensor for the ground for 10 readings
   * Then takes the mean
   * @return current sample of the color
   */
  private double fetchSampleLeft() {
    double filterSum = 0;
    for(int i = 0; i < 10; i++) {
      LeftidColour.fetchSample(LeftcolorValue, 0);
      filterSum += LeftcolorValue[0];
    }
    return filterSum / 10;
  }

  /**
   * A mean filter for light intensity reflected from the ground
   * <p>
   * polls the right light sensor for the ground for 10 readings
   * Then takes the mean
   * @return current sample of the color
   */
  private double fetchSampleRight() {
    double filterSum = 0;
    for(int i = 0; i < 10; i++) {
      RightidColour.fetchSample(RightcolorValue, 0);
      filterSum += RightcolorValue[0];
    }
    return filterSum / 10;
  }
 
}
