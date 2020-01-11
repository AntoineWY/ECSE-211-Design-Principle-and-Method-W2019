package Test;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;

/**
 * Class for testing motor turning accuracy. Turns the motor degrees specified by user.
 * 
 * @author Raymond H. Yang
 * @Date: 2019/03/04
 */
public class turnMotor {

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------
  
  /**
   * Motor instance, specify port motor is connected to.
   */
  private static final EV3LargeRegulatedMotor MOTOR2 =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final NXTRegulatedMotor MOTOR =
	      new NXTRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * The amount of turning in degrees
   */
  private static final int DEG = 440;
  
  /**
   * The acceleration of the motor
   */
  private static final int ACCELERATION = 6000;

  /**
   * The speed of the motor
   */
  private static final int SPEED = 250;
  
  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------
  
  /**
   * Main method
   * @param args - None
   */
  public static void main(String args[]) {
    
    // set acceleration and speed
    MOTOR.setAcceleration(ACCELERATION);
    MOTOR.setSpeed(SPEED);
    
    // reset count of degrees rotated
    MOTOR.resetTachoCount();
    
    // record current tacho count
    int initialCount = MOTOR.getTachoCount();
    
    // signal status
    System.out.println("Standby...");

    // rotate the motor
    int i = 0;
    while(i<5){
    	MOTOR.rotate(90);        
    	MOTOR.rotate(-150);
        
    	MOTOR.rotate(60);
    	i++;
    }
    MOTOR.rotateTo(0);
    System.exit(0);
    // record current tacho count
    int finalCount = MOTOR.getTachoCount();
    
    int buttonChoice;
   
    buttonChoice = Button.waitForAnyPress();
//    while (buttonChoice != Button.ID_ESCAPE){
//    	MOTOR.setSpeed(200);
//    	MOTOR.forward();
//    	if(!MOTOR.isStalled()){
//    		Sound.beepSequenceUp();
//    		break;
//    	}
//    }
      
    
    try {
		Thread.sleep(500);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
    // signal completion
    System.out.println("Complete: " + DEG + " degrees");

    // record rotation
    System.out.println("Recorded Rotation: " + (finalCount - initialCount) + " degrees");
    
  }
}
