package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This class implements the P Controller for Lab1 on the EV3 platform
 * @author Yinuo A Wang
 * @author Tudor Gurau
 */
public class PController implements UltrasonicController {

  /* Constants */
  //private static final int MOTOR_SPEED = 200;
  private static final int FILTER_OUT = 5;
  private static final double PCONST = 4;
  private static final int MAX_CORRECTION = 180;

  private final int bandCenter;
  private final int bandwidth;
  private int distance;
  private int filterControl;
  private static final int MOTOR_SPEED = 200;


/**
 * Controller constructor for P Controller
 * @param bandCenter distance from wall to robot (cm)
 * @param bandwidth acceptable error from bandCenter (cm)
 */
  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }


  /**
   * This is a helper class calculating the adjusted correction based on the error.
   * If the calculated correction gets too big (the input error value is not reasonable)
   * Assign the correction to 50
   * @param error int distance deviated from bandcenter
   * @return propCorrection int speed adjustment the wall-follower makes when turning
   */
  public int errorCal(int error){                             
	  int propCorrection;
	  propCorrection = (int)(Math.abs(error)*PCONST);
	  if(propCorrection >= MAX_CORRECTION){          // avoid huge changes which jerks the wall-follower to one side
		  propCorrection = 78; 
	  }
	  return propCorrection;
  }
  
  /**
   * processes the information received from sensor and adjusts the speed of the motors accordingly
   * @param distance in cm received from sensor
   */
  @Override
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 250 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value                                                            
      filterControl++;
    } else if (distance >= 250) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 180: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    int error = this.distance - this.bandCenter;
    if(Math.abs(error) <= this.bandwidth )	                    //If distance is within the error band, keep straight
	{
    	WallFollowingLab.leftMotor.setSpeed(this.MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(this.MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
	}
	else if(error < 0) 					                        //distance from the wall is too small: turning right
	{	

		WallFollowingLab.leftMotor.setSpeed(this.MOTOR_SPEED+ errorCal(error));	
		WallFollowingLab.rightMotor.setSpeed(Math.abs(this.MOTOR_SPEED-errorCal(error)+10));		
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.backward();                 //making right wheel turning back makes a sharp turn
		                                                        //meaning when turning the cart will not move vertically
		WallFollowingLab.rightMotor.rotate(3, true); 
	}
	else									                    //distance from the wall is too large: turning left
	{
		WallFollowingLab.leftMotor.setSpeed(this.MOTOR_SPEED-errorCal(error) +15); //minor adjustment to slow down the left wheel more 
		                                                                            //results in a smaller turning radius (currently is 15 after tests)
		WallFollowingLab.rightMotor.setSpeed(this.MOTOR_SPEED+errorCal(error));
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
