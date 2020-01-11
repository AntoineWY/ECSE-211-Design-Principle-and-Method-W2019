package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;
/**
 * This class implements the BangBang Controller for Lab1 on the EV3 platform
 * @author Yinuo A Wang
 * @author Tudor Gurau
 */
public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;
  
  private int filterControl;
  private static final int DELTA = 78;                // the magnitude of the speed change of wheels
  
  private static final int FILTER_OUT = 5;           //Counter for the filter

/**
 * Controller constructor for BangBang
 * @param bandCenter distance from wall to robot (cm)
 * @param bandwidth acceptable error from bandCenter \(cm)
 * @param motorLow degrees of turning per second for wheels (Low Speed)
 * @param motorHigh degrees of turning per second for wheels (High Speed)
 */
  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    this.filterControl = 0;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }
/**
 * processes the information received from sensor and adjusts the speed of the motors accordingly
 * @param distance in cm received from sensor
 */
  @Override
  public void processUSData(int distance) {
    this.distance = distance;
      // rudimentary filter - toss out invalid samples corresponding to null
      // signal.
      // (n.b. this was not included in the Bang-bang controller, but easily
      // could have).
      //
      if (distance >= 110 && filterControl < FILTER_OUT) {
        // bad value, do not set the distance var, however do increment the
        // filter value
        filterControl++;
      } else if (distance >= 100) {
        // We have repeated large values, so there must actually be nothing
        // there: leave the distance alone
        this.distance = distance;
      } else {
        // distance went below 225: reset filter and leave
        // distance alone.
        filterControl = 0;
        this.distance = distance;
      }
      
    int error = this.distance - this.bandCenter;
    if(Math.abs(error) <= this.bandwidth)	                   // Distance if with in the error band: move forward
	{
    	WallFollowingLab.leftMotor.setSpeed(this.motorHigh);
    	WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
	}
	else if(error < 0) 					                        //distance from the wall is too small: turning right
	{	
		WallFollowingLab.leftMotor.setSpeed(this.motorHigh+DELTA );	
		WallFollowingLab.rightMotor.setSpeed(this.motorHigh-DELTA);	
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.backward();                 //making right wheel turning back makes a sharp turn
                                                                //meaning when turning the cart will not move vertically
		
	}
	else									                    //distance from the wall is too large: turning left
	{
		WallFollowingLab.leftMotor.setSpeed(this.motorHigh-DELTA-0); //Minor adjustment to make left wheel turn slower: result in a 
		                                                              //smaller turning radius
		WallFollowingLab.rightMotor.setSpeed(this.motorHigh+DELTA);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
  }
/**
 * receives the data from the sensor and changes the distance variable
 */
  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
