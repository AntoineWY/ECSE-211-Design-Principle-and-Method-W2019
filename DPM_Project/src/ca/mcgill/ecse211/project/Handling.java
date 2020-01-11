package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.project.SENSOR_MOTOR;


/**
 *  This class is responsible for retrieving the can and releasing it
 *  <p>
 *  The Handling class generally serves as a collection of method which operate the sensor motor. In total 3 methods are 
 *  contained in this class. The catchCan(boolean) and release(boolean) methods are implemented for the vehicle to close
 *  or open the gripper on the front of the car so that a can is brought in or released.<br>
 *  The private approach() method is a help method driving motors towards the can for catching
 *  
 *  @author Yun Hao Hu
 * 
 */
public class Handling{
//----------------------------------------------------Constants---------------------------------------------------------------------------------
 /**
  * Speed for the sensor motor which also controls the gripper to rotate
  */
  private static final float SENSOR_SPEED = 250; 
 /**
  * Speed for the vehicle to move to or away from the can
  */
  private static final float MOTOR_SPEED = 200; 
  
//----------------------------------------------------Public Methods---------------------------------------------------------------------------------  
  /**
   * Based on the boolean passed, drive forward or directly brings the can in using the sensor motor
   * to rotate the gripper inside, bringing the can in
   * <br>
   * Note that this method will only be called when the claw for the gripper is initially open outward 
   * (for example, heading to -50 degree)
   * @param isClose whether the can is close enough for catch
   */
  public void catchCan(boolean isClose) {
        SENSOR_MOTOR.setSpeed(SENSOR_SPEED);
       
        if(!isClose) {
          approach();
        }
        SENSOR_MOTOR.rotate(140, true);
    }
  


  /**
   * Open claw to release the can
   * <br> The boolean indicates whether the can and the vehicle has arrived to the starting block. If it is, after release them
   * back off to proceed to the next search<br>
   * The back-off distance is calculated by the distance that the robot enters the starting block
   * @param isStartingBlock whether the vehicle is in the starting block right now
   * @see {@link MapPlanner#backToStartingBlock(NavigationWithCorr)}
   */
  public void release(boolean isStartingBlock) {
    SENSOR_MOTOR.rotateTo(-40);
    if (isStartingBlock = true){ //  if in the starting block, release the can and back off for 13 cm 
    	project.LEFT_MOTOR.setSpeed(MOTOR_SPEED); // the 13cm distance is roughly the diagonal of 0.3 square
    	project.RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
    	project.LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, -13), true);
    	project.RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, -13), false);
        SENSOR_MOTOR.rotateTo(0);
    }
    
  }
//----------------------------------------------------Private Methods---------------------------------------------------------------------------------  
  /**
   * move forward until the can is placed at the center of the gripper for catching
   */
  private void approach() {
    project.LEFT_MOTOR.setSpeed(MOTOR_SPEED);
    project.RIGHT_MOTOR.setSpeed(MOTOR_SPEED);
    project.LEFT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, 10), true);
    project.RIGHT_MOTOR.rotate(NavigationWithCorr.convertDistance(project.WHEEL_RAD, 10), false);
  }
}
