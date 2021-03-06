package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class implement the odometer that will be used to measure the cart's position
 * during square driving
 * @author Yinuo A Wang
 * @author Tudor Gurau
 */
public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  private final double TRACK; // Length between the two wheels
  private final double WHEEL_RAD; // radius of the wheel

  
  private double dL;      //Change of displacement on left wheel
  private double dR;      //Change of displacement on right wheel
  
  private int oldLCount;  //A holder variable: Holding the entire degrees turned on left wheel
  private int oldRCount;  //A holder variable: Holding the entire degrees turned on right wheel
  private double Theta;   // current heading angle
  

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;


  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;
   

    while (true) {
        updateStart = System.currentTimeMillis();

        leftMotorTachoCount = leftMotor.getTachoCount();
        rightMotorTachoCount = rightMotor.getTachoCount();
        
        double dX, dY, dTheta,dRad, dDis;

        // TODO Calculate new robot position based on tachometer counts
       
        //calculates displacement on left and right motors
        dL = Math.PI*WHEEL_RAD*(leftMotorTachoCount - oldLCount)/180;
        dR = Math.PI*WHEEL_RAD*(rightMotorTachoCount - oldRCount)/180;
        
        //Update TachoCount
        oldLCount = leftMotorTachoCount;
        oldRCount = rightMotorTachoCount;
        
        //Angle calculation
        dDis = 0.5*(dL+dR);
        dRad = (dL-dR)/TRACK;
        

        //Change in displacement and angle (Theta plus half of the increment value for more accurate results)
        dX = dDis*Math.sin(Theta + dRad/2.0);
        dY = dDis*Math.cos(Theta + dRad/2.0);
        dTheta = dRad*180/Math.PI;
        Theta = Theta+dRad;

        // TODO Update odometer values with new calculated values
        odo.update(dX, dY, dTheta);

        // this ensures that the odometer only runs once every period
        updateEnd = System.currentTimeMillis();
        if (updateEnd - updateStart < ODOMETER_PERIOD) {
          try {
            Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
          } catch (InterruptedException e) {
            // there is nothing to be done
          }
        }
      }
  }

}
