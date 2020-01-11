package Test;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * Class for touch sensor testing. Robot stops when touch sensor activated.
 * 
 * @author Raymond H. Yang
 */
public class touchSensor {

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  /**
   * The instance of the left wheel large EV3 motor. The left motor is connected to port A on the
   * EV3 brick.
   */
  private static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The instance of the right wheel large EV3 motor. The right motor is connected to port D on the
   * EV3 brick.
   */
  private static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * Port for touch sensor
   */
  private static final Port TS_PORT = LocalEV3.get().getPort("S1");

  /**
   * The acceleration for the motors
   */
  private static final int ACCELERATION = 6000;

  /**
   * The target speed to achieve
   */
  private static final int SPEED = 250;

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  /**
   * Main method for test
   * 
   * @param args - None
   */
  public static void main(String args[]) {

    // sensor initialization
    @SuppressWarnings("resource")
    SampleProvider tsSample = (new EV3TouchSensor(TS_PORT)).getMode("Touch");
    float[] tsData = new float[tsSample.sampleSize()];

    System.out.println("Click any button to proceed or ESC to quit");

    // user ready
    if (Button.waitForAnyPress() != Button.ID_ESCAPE) {

      // start moving
      setDevice(ACCELERATION, SPEED);

      // while robot is moving, check if touch sensor activated
      while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {

        // obtain reading
        tsSample.fetchSample(tsData, 0);

        // check if pressed detected
        // 0: not detected 1: detected
        if (tsData[0] == 1) {

          // stop moving
          stop();

          // indicate detections
          Sound.beepSequenceUp();

          System.out.println("Activated. Press any button to exit.");

          // press any button to exit
          Button.waitForAnyEvent();
          System.exit(0);

        }
      }
    } else {
      // esc to quit
      System.exit(0);
    }
  }

  /**
   * Sets acceleration and speed for both motors. Start the robot.
   * 
   * @param acceleration - acceleration as integer
   * @param speed - speed as integer
   */
  private static void setDevice(int acceleration, int speed) {
    LEFT_MOTOR.setAcceleration(acceleration);
    RIGHT_MOTOR.setAcceleration(acceleration);
    LEFT_MOTOR.setSpeed(speed);
    RIGHT_MOTOR.setSpeed(speed);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  /**
   * Stop robot immediately
   */
  private static void stop() {
    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop(false);
  }

}
